# Copyright (c) 2025 Bart van de Lint, Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Auto-Parking Ram Air Kite Simulation Example

Demonstrates how to use RamAirKite.jl to run a simulation with cascaded
steering-line position→speed→torque control. A DiscretePID heading
controller tracks a zero heading setpoint (loitering), while inner control
loops convert the steering setpoint into a torque command via cascaded
position and speed PIDs on the delta-length of the steering lines. The
simulation uses the "ram" physical model by default, which includes a
bridle system and four wing segment twist surfaces.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Timers
tic()
@info "Loading packages..."
using MakieControlPlots
using MakieControlPlots: plot, plotx
using LaTeXStrings
using RamAirKite
using SymbolicAWEModels
using DiscretePIDs
using LinearAlgebra
using Statistics
using Printf

toc()

# User changeable parameters
PLOT = true                  # Whether to plot results at the end
PRINT_PROGRESS = false       # Whether to print progress during simulation
PHYSICAL_MODEL = "ram"       # Options: "ram", "simple_ram", "4_attach_ram"
SIM_TIME = 30.0              # Total simulation time [s]
DT = 0.01                    # Time step [s]
V_WIND = 12.51               # Wind speed [m/s]
UPWIND_DIR = -90.0           # Upwind direction [deg]
TETHER_LENGTH = 50.0         # Tether length [m]
ELEVATION = 76.5             # Initial elevation angle [deg]
AERO_Z_OFFSET = 1.0          # Body-frame z-offset for VSM panels [m]
PROFILE_LAW = 3              # Wind profile law (3 = EXPLOG)
REMAKE_CACHE = false         # Force rebuild of compiled model cache
VSM_INTERVAL = 7             # VSM update interval
MAX_STEERING = 1.5           # Steering limit [m]
HEADING_P = 1.0              # Heading PID proportional gain
HEADING_I = 1.7         # Heading PID integral time (false = off)
HEADING_D = 0.4             # Heading PID derivative time

# Cascaded position + speed controller for steering lines
POSITION_P = 8.0             # Position PID proportional gain
POSITION_I = 2.0             # Position PID integral time [s]
POSITION_D = 0.0005             # Position PID derivative time (0 = off)
POSITION_UMIN = -1.2         # Minimum speed setpoint [m/s]
POSITION_UMAX = 1.2          # Maximum speed setpoint [m/s]
SPEED_P = 12                # Speed PID proportional gain
SPEED_I = 0.08                # Speed PID integral time [s]
SPEED_D = 0.0                # Speed PID derivative time (0 = off)
SPEED_TAU = 0.05             # Low-pass filter time constant for speed [s]
TORQUE_UMIN = -40.0          # Minimum torque output [Nm]
TORQUE_UMAX = 40.0           # Maximum torque output [Nm]

@info "Creating ram air kite model..."
set_data_path(ram_air_data_path())
set = Settings("system.yaml")
set.physical_model = PHYSICAL_MODEL
set.v_wind = V_WIND
set.upwind_dir = UPWIND_DIR
set.profile_law = PROFILE_LAW
set.l_tether = TETHER_LENGTH

# 1. system structure
sys_struct = create_sys_struct(set)

# 2. model
sam = SymbolicAWEModel(set, sys_struct)

# edit sys_struct before init!
sys_struct.transforms[1].elevation = deg2rad(ELEVATION)
sys_struct.wings[1].aero_z_offset = AERO_Z_OFFSET
# sys_struct.tethers[:steering_left].init_stretch_frac = 1.005
# sys_struct.tethers[:steering_right].init_stretch_frac = 1.005
sys_struct.winches[:power_winch].brake = true
# sys_struct.winches[:steering_right_winch].brake = true
# sys_struct.winches[:steering_left_winch].brake = true

for point in sam.sys_struct.points
    point.body_frame_damping .= 0.0
end
for segment in sam.sys_struct.segments
    segment.compression_frac = 0.01
end
for twist_surface in sam.sys_struct.twist_surfaces
    twist_surface.moment_frac = 0.0
end

depower = 0.01
sys_struct.tethers[:steering_left].init_stretch_frac = 1.0 + depower
sys_struct.tethers[:steering_right].init_stretch_frac = 1.0 + depower

# 3. init
@info "Initializing model..."
init!(sam; remake=REMAKE_CACHE)

find_steady_state!(sam; dt=0.05, vsm_interval=7)
toc("Steady state found after: ")

# Plot initial configuration
# fig = plot(sam.sys_struct)

# Run heading-tracking simulation
@info "Running simulation..."
dt = DT
steps = Int(round(SIM_TIME / dt))
torque_damp = 0.9

logger = Logger(sam, steps + 1)
sys_state = SysState(sam)
sys_state.time = 0.0

steady_torque = calc_steady_torque(sam)

for twist_surface in sam.sys_struct.twist_surfaces
    twist_surface.damping = 200.0
end

heading_pid = DiscretePID(; K=HEADING_P, Ti=HEADING_I, Td=HEADING_D, Ts=dt, umin=(-MAX_STEERING), umax=MAX_STEERING)
pos_pid = DiscretePID(; K=POSITION_P, Ti=POSITION_I, Td=POSITION_D, Ts=dt, umin=POSITION_UMIN, umax=POSITION_UMAX)
speed_pid = DiscretePID(; K=SPEED_P, Ti=SPEED_I, Td=SPEED_D, Ts=dt, umin=TORQUE_UMIN, umax=TORQUE_UMAX)

# Track steering torque for logging
steering_torque_history = Float64[]
dl_setpoint_history = Float64[]
steering_speed_history = Float64[]  # filtered v_reelout difference
sizehint!(steering_torque_history, steps)
sizehint!(dl_setpoint_history, steps)
sizehint!(steering_speed_history, steps)

v_reelout_diff_filt = Ref(0.0)
alpha = dt / (dt + SPEED_TAU)  # low-pass filter coefficient

last_time = time()
for step in 1:steps
        t = step * dt

        current_heading = sam.sys_struct.wings[1].heading
        steering = heading_pid(0, current_heading, 0.0)

        # Cascaded position → speed → torque control
        local l_diff = sys_state.l_tether[3] - sys_state.l_tether[4]
        v_reelout_diff = sys_state.v_reelout[2] - sys_state.v_reelout[3]
        v_reelout_diff_filt[] = alpha * v_reelout_diff + (1 - alpha) * v_reelout_diff_filt[]
        push!(steering_speed_history, v_reelout_diff_filt[])
        speed_setpoint = pos_pid(steering, l_diff, 0.0)
        torque = speed_pid(speed_setpoint, v_reelout_diff_filt[], 0.0)
        push!(steering_torque_history, torque)
        push!(dl_setpoint_history, steering)
        set_values = [0.0, torque, -torque]

        global steady_torque = torque_damp * steady_torque + (1 - torque_damp) * calc_steady_torque(sam)
        set_torques = steady_torque .+ set_values

        next_step!(sam; set_values=set_torques, dt, vsm_interval=VSM_INTERVAL)

        update_sys_state!(sys_state, sam)
        sys_state.time = t
        log!(logger, sys_state)

        if step % 10 == 0
            now = time()
            realtime_factor = (10 * dt) / (now - last_time)
            global last_time = now
            if PRINT_PROGRESS
                @info "step $step / $steps, $(round(realtime_factor; digits=2)) times realtime"
            end
        end
    end

mkpath(get_data_path())
save_log(logger, "tmp_run")
syslog = load_log("tmp_run")
sl = syslog.syslog

aero_force_norm = norm.(eachrow(sl.aero_force_b))
l_diff = [sl.l_tether[i][3] - sl.l_tether[i][4] for i in 1:length(sl.time)]

if PLOT
    p=plotx(
        sl.time,
        rad2deg.(sl.elevation),
        rad2deg.(sl.azimuth),
        rad2deg.(sl.heading),
        dl_setpoint_history,
        rad2deg.(sl.AoA),
        sl.v_app,
        aero_force_norm;
        xlabel=L"\mathrm{Time}~[\mathrm{s}]",
        ylabels=[
            L"\mathrm{elevation}~[°]",
            L"\mathrm{azimuth}~[°]",
            L"\mathrm{heading}~[°]",
            L"\Delta l_{\mathrm{set}}~[\mathrm{m}]",
            L"\mathrm{AoA}~[°]",
            L"v_{\mathrm{a}}~[\mathrm{ms^{-1}}]",
            L"\mathrm{aeroforce}~[\mathrm{N}]",
        ],
        ysize=18,
        fig="Ram air kite",
    )
    display(p)

    delta_labels = [L"\Delta l_{\mathrm{set}}~[\mathrm{m}]", L"\Delta l~[\mathrm{m}]"]
    all_labels = [delta_labels, nothing]
    p2 = plotx(
        sl.time,
        [dl_setpoint_history, l_diff],
        steering_torque_history;
        xlabel=L"\mathrm{Time}~[\mathrm{s}]",
        ylabels=[L"\Delta l~[\mathrm{m}]", L"\mathrm{torque}~[\mathrm{Nm}]"],
        labels=all_labels,
        ysize=18,
        fig="Delta-l setpoint vs actual",
    )
    display(p2)
end

rms_azimuth = rad2deg(sqrt(mean(sl.azimuth .^ 2)))
@printf("Azimuth RMS error: %.2f°\n", rms_azimuth)
nothing

# Interactive replay
# replay(syslog, sam.sys_struct)
