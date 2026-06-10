# SPDX-FileCopyrightText: 2026 Bart van de Lint, Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

using GLMakie
using RamAirKite
using RamAirKite: ram_air_data_path
using KiteUtils: Settings, set_data_path

set_data_path(ram_air_data_path())
set = Settings("system.yaml")
set.physical_model = "ram"
set.v_wind = 12.51
set.upwind_dir = -90.0
set.l_tether = 25.0

sys_struct = create_sys_struct(set)
sam = SymbolicAWEModel(set, sys_struct)

init!(sam; remake=false)
find_steady_state!(sam; dt=0.05, vsm_interval=0)

# Warm up plotting
fig = Makie.plot(sam.sys_struct)