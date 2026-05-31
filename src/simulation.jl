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

"""
    create_ram_air_model(config::RamAirSimConfig; data_path=nothing)
    create_ram_air_model(; kwargs...)

Create a ram air kite SymbolicAWEModel with the given configuration.

# Arguments
- `config::RamAirSimConfig`: Configuration struct
- `data_path`: Path to ram air kite data directory (default: RamAirKite bundled data)

# Returns
- `sam`: SymbolicAWEModel
"""
function create_ram_air_model(config::RamAirSimConfig; data_path=nothing)
    # Use bundled data by default
    if isnothing(data_path)
        data_path = ram_air_data_path()
    end

    wing_type_str = config.wing_type == SymbolicAWEModels.REFINE ? "REFINE" : "QUATERNION"
    @info "Creating ram air kite model" physical_model=config.physical_model wing_type=wing_type_str data_path

    # Load settings
    set_data_path(data_path)
    set = Settings("system.yaml")
    set.physical_model = config.physical_model
    set.v_wind = config.v_wind
    set.upwind_dir = config.upwind_dir
    set.profile_law = config.profile_law

    # Create model
    sys_struct = create_sys_struct(set)
    sam = SymbolicAWEModel(set, sys_struct)

    # Adjust tether length
    adjust_tether_length!(sam, config.tether_length)

    # Adjust elevation if provided
    if config.elevation !== nothing
        adjust_elevation!(sam, config.elevation)
    end

    return sam
end

function create_ram_air_model(; kwargs...)
    config = RamAirSimConfig(; kwargs...)
    return create_ram_air_model(config)
end

"""
    run_ram_air_simulation(config::RamAirSimConfig; show_progress=true, find_steady=true, data_path=nothing)
    run_ram_air_simulation(; kwargs...)

Run a ram air kite simulation with the given configuration.

Uses sinusoidal steering oscillation for control, similar to the original
ram_air_kite.jl example in SymbolicAWEModels.

# Arguments
- `config::RamAirSimConfig`: Configuration struct
- `show_progress`: Show progress updates (default: true)
- `find_steady`: Find steady state before starting simulation (default: true)
- `data_path`: Path to data directory (default: bundled data)

# Returns
- `(sam, syslog)`: Tuple of SymbolicAWEModel and logged data
"""
function run_ram_air_simulation(config::RamAirSimConfig;
                                show_progress=true,
                                find_steady=true,
                                data_path=nothing)
    # Create model
    sam = create_ram_air_model(config; data_path)

    # Initialize model
    wing_type_str = config.wing_type == SymbolicAWEModels.REFINE ? "REFINE" : "QUATERNION"
    @info "Initializing $wing_type_str model..."
    SymbolicAWEModels.init!(sam; remake=config.remake_cache)

    # Find steady state if requested
    if find_steady
        @info "Finding steady state..."
        find_steady_state!(sam; dt=config.dt)
    end

    # Set winch brake
    if !isempty(sam.sys_struct.winches)
        sam.sys_struct.winches[1].brake = config.brake
    end

    # Adjust steering bias for 4_attach_ram model
    steering_bias = config.steering_bias
    if config.physical_model == "4_attach_ram"
        steering_bias = 0.05
    end

    @info "Starting simulation" sim_time=config.sim_time dt=config.dt vsm_interval=config.vsm_interval
    sim_start_time = time()

    # Run oscillating simulation
    syslog, _ = sim_oscillate!(sam;
        dt=config.dt,
        total_time=config.sim_time,
        vsm_interval=config.vsm_interval,
        steering_freq=config.steering_freq,
        steering_magnitude=config.steering_magnitude,
        bias=steering_bias,
        prn=show_progress)

    # Report performance
    total_wall_time = time() - sim_start_time
    final_times_realtime = config.sim_time / total_wall_time
    @info "Simulation completed" wall_time=round(total_wall_time, digits=2) times_realtime=round(final_times_realtime, digits=2)

    # Save and load log
    log_name = "ram_air_$(config.physical_model)_$(lowercase(wing_type_str))_$(Dates.format(Dates.now(), "yyyy_mm_dd_HH_MM_SS"))"
    save_log(syslog, log_name)
    syslog = load_log(log_name)

    return sam, syslog
end

function run_ram_air_simulation(; kwargs...)
    config = RamAirSimConfig(; kwargs...)
    return run_ram_air_simulation(config)
end
