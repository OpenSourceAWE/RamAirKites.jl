# Copyright (c) 2025 Bart van de Lint
# SPDX-License-Identifier: MPL-2.0

"""
Ram Air Kite Simulation Example

Demonstrates how to use RamAirKite.jl to run a simulation with sinusoidal
steering oscillation. The simulation uses the "ram" physical model by default,
which includes a bridle system and 2 wing groups.
"""

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Timers
tic()
@info "Loading packages..."
using GLMakie
using RamAirKite
using SymbolicAWEModels
using LinearAlgebra

toc()

# Create simulation configuration
config = RamAirSimConfig(
    physical_model = "ram",      # Options: "ram", "simple_ram", "4_attach_ram"
    sim_time = 10.0,             # Total simulation time [s]
    dt = 0.05,                   # Time step [s]
    v_wind = 15.51,              # Wind speed [m/s]
    tether_length = 50.0,        # Tether length [m]
    vsm_interval = 20,            # VSM update interval
    steering_freq = 0.5,         # Steering oscillation frequency [Hz]
    steering_magnitude = 1.0,    # Steering torque magnitude [Nm]
)

@info "Creating ram air kite model..."
set_data_path(ram_air_data_path())
set = Settings("system.yaml")
set.physical_model = config.physical_model
set.v_wind = config.v_wind
set.upwind_dir = config.upwind_dir
set.profile_law = config.profile_law
set.l_tether = config.tether_length

# 1. system structure
sys_struct = create_sys_struct(set)

# 2. model
sam = SymbolicAWEModel(set, sys_struct)

# edit sys_struct before init!
sys_struct.transforms[1].elevation = deg2rad(85)
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

# 3. init
@info "Initializing model..."
init!(sam; remake=config.remake_cache)

# Plot initial configuration
fig = plot(sam.sys_struct)

# Run oscillating simulation
@info "Running simulation..."
dt = config.dt
steps = Int(round(config.sim_time / dt))
torque_damp = 0.9

logger = Logger(sam, steps + 1)
sys_state = SysState(sam)
sys_state.time = 0.0

steady_torque = calc_steady_torque(sam)

for group in sam.sys_struct.groups
    group.damping = 200.0
end

try
    for step in 1:steps
        @show step
        t = step * dt

        steering = config.steering_magnitude * sin(2π * config.steering_freq * t) +
                   config.steering_bias
        set_values = [0.0, steering, -steering]

        global steady_torque = torque_damp * steady_torque +
                               (1 - torque_damp) * calc_steady_torque(sam)
        set_torques = steady_torque .+ set_values

        next_step!(sam; set_values=set_torques, dt, vsm_interval=config.vsm_interval)

        vsm_wing = sam.sys_struct.wings[1].vsm_wing
        mid = length(vsm_wing.refined_sections) ÷ 2
        group_twist = [g.twist for g in sam.sys_struct.groups[sam.sys_struct.wings[1].group_idxs]]

        update_sys_state!(sys_state, sam)
        sys_state.time = t
        log!(logger, sys_state)
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

# Plot results and show replay
fig = plot(sam.sys_struct, syslog)
display(fig)

# Interactive replay
replay(syslog, sam.sys_struct)

