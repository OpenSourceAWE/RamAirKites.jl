# Copyright (c) 2025 Bart van de Lint
# SPDX-License-Identifier: MPL-2.0

"""
    RamAirKite

Package for simulation of generic ram-air kites built on SymbolicAWEModels.jl.
Provides model setup utilities, simulation functions, and configuration structs
for "ram", "simple_ram", and "4_attach_ram" physical models.

Unlike V3Kite.jl which uses KCU-specific steering/depower percentages,
RamAirKite uses generic 3-winch torque control.
"""
module RamAirKite

using SymbolicAWEModels
using VortexStepMethod
using KiteUtils
using LinearAlgebra
using Statistics
using UnPack
using Dates

# Re-export commonly used types from SymbolicAWEModels
export SymbolicAWEModel, SystemStructure, Logger, SysState
export load_sys_struct_from_yaml, set_data_path, get_data_path
export init!, next_step!, update_sys_state!, log!, save_log, load_log
export find_steady_state!, sim_oscillate!
export REFINE, QUATERNION, WING

# Include submodules
include("predefined_structures.jl")
include("model_setup.jl")
include("simulation.jl")
include("simulation_utils.jl")

# Model setup exports
export segment_stretch_stats

# Predefined structure factory exports
export create_ram_sys_struct, create_simple_ram_sys_struct
export create_4_attach_ram_sys_struct, create_tether_sys_struct
export create_sys_struct

# Simulation exports
export ram_air_data_path
export sim_turn!, copy_to_simple!

end # module
