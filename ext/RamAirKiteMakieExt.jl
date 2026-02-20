# Copyright (c) 2025 Bart van de Lint
# SPDX-License-Identifier: MPL-2.0

"""
RamAirKite Makie Extension

Provides visualization functions for ram air kite simulation results when GLMakie is available.
Re-exports the plotting functions from SymbolicAWEModels.
"""
module RamAirKiteMakieExt

using RamAirKite
using GLMakie
import RamAirKite: SymbolicAWEModels

# Re-export SymbolicAWEModels plotting functions when GLMakie is loaded
# The actual plotting is handled by SymbolicAWEModelsMakieExt

end # module
