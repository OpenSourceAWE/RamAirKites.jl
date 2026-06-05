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
