# Copyright (c) 2025 Bart van de Lint
# SPDX-License-Identifier: MPL-2.0

"""
Ram air kite simulation functions.
Provides high-level functions for creating and running ram air kite simulations.
"""

"""
    ram_air_data_path()

Return the path to the ram air kite data directory bundled with RamAirKite.jl.
"""
function ram_air_data_path()
    return joinpath(pkgdir(@__MODULE__), "data", "ram_air_kite")
end

"""
    RamAirSimConfig

Configuration structure for ram air kite simulations.

# Fields
- `physical_model::String`: Model variant - "ram", "simple_ram", or "4_attach_ram"
- `sim_time::Float64`: Total simulation time in seconds
- `dt::Float64`: Time step size in seconds
- `v_wind::Float64`: Wind speed at reference height in m/s
- `upwind_dir::Float64`: Upwind direction in degrees
- `tether_length::Float64`: Initial tether length in meters
- `elevation::Union{Nothing, Float64}`: Initial elevation angle in degrees (nothing = use default)
- `wing_type::SymbolicAWEModels.WingType`: QUATERNION or REFINE
- `remake_cache::Bool`: Force rebuild of compiled model cache
- `vsm_interval::Int`: VSM update interval (1 = every step, 3 = every 3rd step)
- `steering_freq::Float64`: Frequency of sinusoidal steering oscillation in Hz
- `steering_magnitude::Float64`: Magnitude of steering input in Nm
- `steering_bias::Float64`: Bias torque added to steering input in Nm
- `profile_law::Int`: Wind profile law (3 = EXPLOG)
- `brake::Bool`: Whether winch brake is engaged
"""
Base.@kwdef mutable struct RamAirSimConfig
    # Model variant: "ram", "simple_ram", or "4_attach_ram"
    physical_model::String = "ram"

    # Simulation parameters
    sim_time::Float64 = 10.0
    dt::Float64 = 0.05

    # Wind parameters
    v_wind::Float64 = 15.51
    upwind_dir::Float64 = -90.0

    # Tether configuration
    tether_length::Float64 = 50.0
    elevation::Union{Nothing, Float64} = nothing

    # Model options
    wing_type::SymbolicAWEModels.WingType = SymbolicAWEModels.QUATERNION
    remake_cache::Bool = false
    vsm_interval::Int = 3

    # Steering control (sinusoidal oscillation)
    steering_freq::Float64 = 0.5     # Hz - full left-right cycle frequency
    steering_magnitude::Float64 = 1.0 # Nm - magnitude of steering input
    steering_bias::Float64 = 0.2      # Nm - bias torque

    # Other model options
    profile_law::Int = 3  # Wind profile law (3 = EXPLOG)
    brake::Bool = true    # Winch brake engaged
end

