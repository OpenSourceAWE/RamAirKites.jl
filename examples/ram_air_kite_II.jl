# Copyright (c) 2025 Bart van de Lint
# SPDX-License-Identifier: MPL-2.0

"""
Ram Air Kite Simulation Example

Demonstrates how to use RamAirKite.jl to run a simulation steered by a
DiscretePID that tracks a sinusoidal heading setpoint. The simulation uses
the "ram" physical model by default, which includes a bridle system and 2
wing groups.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Timers
tic()
@info "Loading packages..."
using MakieControlPlots
using MakieControlPlots: plot
using RamAirKite
using SymbolicAWEModels
using DiscretePIDs
using LinearAlgebra

toc()

# User changeable parameters
PHYSICAL_MODEL = "ram"      # Options: "ram", "simple_ram", "4_attach_ram"
SIM_TIME = 5.0             # Total simulation time [s]
DT = 0.05                   # Time step [s]
V_WIND = 15.51             # Wind speed [m/s]
UPWIND_DIR = -90.0          # Upwind direction [deg]
TETHER_LENGTH = 50.0        # Tether length [m]
PROFILE_LAW = 3             # Wind profile law (3 = EXPLOG)
REMAKE_CACHE = false        # Force rebuild of compiled model cache
VSM_INTERVAL = 20           # VSM update interval
MAX_HEADING = 0.0          # Heading setpoint amplitude [deg]
HEADING_PERIOD = 5.0        # Heading setpoint period [s]
MAX_STEERING = 2.0          # Steering torque limit [Nm]
HEADING_P = 0.5             # Heading PID proportional gain
HEADING_I = 0.0           # Heading PID integral time (false = off)
HEADING_D = 0.0             # Heading PID derivative time

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
sys_struct.transforms[1].elevation = deg2rad(70)
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

depower = 0.009
sys_struct.tethers[:steering_left].init_stretch_frac = 1.0 - depower
sys_struct.tethers[:steering_right].init_stretch_frac = 1.0 - depower

# 3. init
@info "Initializing model..."
init!(sam; remake=REMAKE_CACHE)

find_steady_state!(sam; dt=0.05, vsm_interval=0)
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

for group in sam.sys_struct.groups
    group.damping = 200.0
end

heading_pid = DiscretePID(; K=HEADING_P, Ti=HEADING_I, Td=HEADING_D, Ts=dt,
                          umin=-MAX_STEERING, umax=MAX_STEERING)
max_heading = deg2rad(MAX_HEADING)
angular_freq = 2π / HEADING_PERIOD
heading_setpoint = Float64[]

last_time = time()
try
    for step in 1:steps
        t = step * dt

        target_heading = max_heading * sin(angular_freq * t)
        current_heading = sam.sys_struct.wings[1].heading
        steering = heading_pid(target_heading, current_heading, 0.0)
        push!(heading_setpoint, target_heading)
        set_values = [0.0, steering, -steering]

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
catch e
    if e isa AssertionError || e isa InterruptException
        println("Sim failed")
    else
        rethrow(e)
    end
end

mkpath(get_data_path())
save_log(logger, "tmp_run")
syslog = load_log("tmp_run")
nothing

# Plot results and show replay
# fig = plot(sam.sys_struct, syslog;
#            plot_heading=true, setpoints=Dict(:heading => heading_setpoint))
# display(fig)

# Interactive replay
# replay(syslog, sam.sys_struct)

