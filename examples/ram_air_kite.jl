# Copyright (c) 2025 Bart van de Lint
# SPDX-License-Identifier: MPL-2.0

"""
Ram Air Kite Simulation Example

Demonstrates how to use RamAirKite.jl to run a simulation with cascaded
steering-line position→speed→torque control, tracking a sinusoidal heading
setpoint. The simulation uses the "ram" physical model by default, which
includes a bridle system and four wing section groups.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Timers
tic()
@info "Loading packages..."
using GLMakie
import MakieControlPlots as mcp
using LaTeXStrings
using RamAirKite
using SymbolicAWEModels
using DiscretePIDs
using LinearAlgebra
using StructArrays

toc()

# User changeable parameters
PHYSICAL_MODEL = "ram"      # Options: "ram", "simple_ram", "4_attach_ram"
SIM_TIME = 15.0             # Total simulation time [s]
RECORD_VIDEO = false         # Whether to record a video of the simulation (can be slow)
DT = 0.01                   # Time step [s] (must be small for cascaded control)
V_WIND = 12.51             # Wind speed [m/s]
UPWIND_DIR = -90.0          # Upwind direction [deg]
TETHER_LENGTH = 25.0        # Tether length [m]
ELEVATION = 74.0            # Initial elevation angle [deg]
AERO_Z_OFFSET = 0.0         # Body-frame z-offset for VSM panels [m]
PROFILE_LAW = 3             # Wind profile law (3 = EXPLOG)
REMAKE_CACHE = false        # Force rebuild of compiled model cache
VSM_INTERVAL = 7            # VSM update interval
MAX_HEADING = 20.0          # Heading setpoint amplitude [deg]
HEADING_PERIOD = 5.0        # Heading setpoint period [s]
MAX_STEERING = 1.5           # Steering limit [m] (position setpoint)
HEADING_P = 0.7              # Heading PID proportional gain
HEADING_I = 1.5              # Heading PID integral time (false = off)
HEADING_D = 0.43             # Heading PID derivative time

# Cascaded position + speed controller for steering lines
POSITION_P = 10.0             # Position PID proportional gain
POSITION_I = 2.0             # Position PID integral time [s]
POSITION_D = 0.0005          # Position PID derivative time (0 = off)
POSITION_UMIN = -1.2         # Minimum speed setpoint [m/s]
POSITION_UMAX = 1.2          # Maximum speed setpoint [m/s]
SPEED_P = 14                # Speed PID proportional gain
SPEED_I = 0.08                # Speed PID integral time [s]
SPEED_D = 0.0                # Speed PID derivative time (0 = off)
SPEED_TAU = 0.16             # Low-pass filter time constant for speed [s]
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
for group in sam.sys_struct.groups
    group.moment_frac = 0.0
end

depower = 0.0
sys_struct.tethers[:steering_left].init_stretch_frac = 1.0 - depower
sys_struct.tethers[:steering_right].init_stretch_frac = 1.0 - depower

# 3. init
@info "Initializing model..."
init!(sam; remake=REMAKE_CACHE)

find_steady_state!(sam; dt=0.05, vsm_interval=0)

depower_len = sys_struct.tethers[:steering_left].len - sys_struct.tethers[:power_left].len
@info "Depowered by $(round(depower_len; digits=2)) m"

# Plot initial configuration
fig = Makie.plot(sam.sys_struct)

# Run heading-tracking simulation
@info "Running simulation..."
dt = DT
steps = Int(round(SIM_TIME / dt))
torque_damp = 0.9

logger = Logger(sam, steps + 1)
sys_state = SysState(sam)
sys_state.time = 0.0

steady_torque = calc_steady_torque(sam)

for group in sam.sys_struct.groups
    group.damping = 200.0
end

heading_pid = DiscretePID(; K=HEADING_P, Ti=HEADING_I, Td=HEADING_D, Ts=dt,
                          umin=-MAX_STEERING, umax=MAX_STEERING)
pos_pid = DiscretePID(; K=POSITION_P, Ti=POSITION_I, Td=POSITION_D, Ts=dt,
                       umin=POSITION_UMIN, umax=POSITION_UMAX)
speed_pid = DiscretePID(; K=SPEED_P, Ti=SPEED_I, Td=SPEED_D, Ts=dt,
                         umin=TORQUE_UMIN, umax=TORQUE_UMAX)

# Track steering torque and delta-l for logging
steering_torque_history = Float64[]
dl_setpoint_history = Float64[]
steering_speed_history = Float64[]  # filtered v_reelout difference
sizehint!(steering_torque_history, steps)
sizehint!(dl_setpoint_history, steps)
sizehint!(steering_speed_history, steps)

max_heading = deg2rad(MAX_HEADING)
angular_freq = 2π / HEADING_PERIOD
heading_setpoint = Float64[]

v_reelout_diff_filt = Ref(0.0)
alpha = dt / (dt + SPEED_TAU)  # low-pass filter coefficient

last_time = time()
for step in 1:steps
    t = step * dt

    target_heading = max_heading * sin(angular_freq * t)
    current_heading = sam.sys_struct.wings[1].heading
    push!(heading_setpoint, target_heading)

    # Outer loop: heading PID outputs a steering position setpoint (delta-l)
    steering = heading_pid(target_heading, current_heading, 0.0)

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

    global steady_torque = torque_damp * steady_torque +
                            (1 - torque_damp) * calc_steady_torque(sam)
    set_torques = steady_torque .+ set_values

    next_step!(sam; set_values=set_torques, dt, vsm_interval=VSM_INTERVAL)

    update_sys_state!(sys_state, sam)
    sys_state.time = t
    log!(logger, sys_state)

    if step % 10 == 0
        now = time()
        realtime_factor = (10 * dt) / (now - last_time)
        global last_time = now
        @info "step $step / $steps, $(round(realtime_factor; digits=2)) times realtime"
    end
end

mkpath(get_data_path())
save_log(logger, "tmp_run")
syslog = load_log("tmp_run")

# Plot results and show replay
fig = Makie.plot(sam.sys_struct, syslog;
           plot_heading=true, plot_tether=true, setpoints=Dict(:heading => heading_setpoint))
display(GLMakie.Screen(), fig)

# Plot heading setpoint vs actual heading using MakieControlPlots
sl = syslog.syslog
time_vec = sl.time[1:length(heading_setpoint)]
p = mcp.plot(time_vec, [rad2deg.(heading_setpoint), rad2deg.(sl.heading[1:length(heading_setpoint)])];
          xlabel=L"\mathrm{Time}~[s]",
          ylabel=L"\mathrm{Heading}~[°]",
          labels=["Setpoint", "Actual"],
          ysize=18, fig="Heading setpoint vs actual")
display(p)

# Interactive replay with frame skipping for faster playback
REPLAY_SKIP = 5              # Only render every Nth frame
sub_idx = 1:REPLAY_SKIP:length(syslog.syslog)
sub_data = collect(syslog.syslog[sub_idx])
sub_syslog = StructArrays.StructArray(sub_data)
sub_log = SysLog{length(first(sub_data).X)}("tmp_run_sub", syslog.colmeta, sub_syslog)
if RECORD_VIDEO
    video_path = joinpath("output", "ram_air_kite_simulation.mp4")
    mkpath(dirname(video_path))
    @info "Recording video to $video_path (this may take a while)..."
    SymbolicAWEModels.record(syslog, sam.sys_struct, video_path; framerate=Int(round(1 / DT)))
end
scene = SymbolicAWEModels.replay(sub_log, sam.sys_struct; replay_speed=2.0)
display(GLMakie.Screen(), scene)


