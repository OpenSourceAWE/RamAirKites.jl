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

toc()

# Create simulation configuration
config = RamAirSimConfig(
    physical_model = "ram",      # Options: "ram", "simple_ram", "4_attach_ram"
    sim_time = 10.0,             # Total simulation time [s]
    dt = 0.05,                   # Time step [s]
    v_wind = 15.51,              # Wind speed [m/s]
    tether_length = 50.0,        # Tether length [m]
    vsm_interval = 30,            # VSM update interval
    steering_freq = 0.5,         # Steering oscillation frequency [Hz]
    steering_magnitude = 1.0,    # Steering torque magnitude [Nm]
)

# Create the model
@info "Creating ram air kite model..."
sam = create_ram_air_model(config)

# Initialize the model
@info "Initializing model..."
init!(sam; remake=config.remake_cache)

# Plot initial configuration
# fig = plot(sam.sys_struct)
# display(fig)

function print_forces()
    for segment in sam.sys_struct.segments
        println("Segment $(segment.idx): Force = $(segment.force)")
    end
end
function brake(on::Bool)
    for winch in sam.sys_struct.winches
        winch.brake = on
    end
end

#= segment_idxs = sam.sys_struct.tethers[1].segment_idxs
forces = sam.sys_struct.segments[segment_idxs[1]].force =#

# Find steady state (disable gravity so VSM converges from aerodynamic equilibrium)
@info "Finding steady state..."
sam.set.abs_tol = 0.0005
sam.set.rel_tol = 0.0005
sam.set.dtmax = 0.025
brake(true)
find_steady_state!(sam; dt=1/300)

# # Run oscillating simulation
# @info "Running simulation..."
# syslog, _ = sim_oscillate!(sam;
#     dt = config.dt,
#     total_time = config.sim_time,
#     vsm_interval = config.vsm_interval,
#     steering_freq = config.steering_freq,
#     steering_magnitude = config.steering_magnitude,
#     bias = config.steering_bias,
#     prn = true)

# # Plot results and show replay
# fig = plot(sam.sys_struct, syslog)
# scr = display(fig)
# wait(scr)

# # Interactive replay
# replay(syslog, sam.sys_struct)
