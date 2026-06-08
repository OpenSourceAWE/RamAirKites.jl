# RamAirKite.jl

[![Test](https://github.com/OpenSourceAWE/RamAirKite.jl/actions/workflows/CI.yml/badge.svg)](https://github.com/OpenSourceAWE/RamAirKite.jl/actions/workflows/CI.yml)
[![License: MPL-2.0](https://img.shields.io/badge/License-MPL--2.0-blue.svg)](https://opensource.org/licenses/MPL-2.0)

A Julia package for simulation of generic ram-air kites built on [SymbolicAWEModels.jl](https://github.com/OpenSourceAWE/SymbolicAWEModels.jl).

## Overview

RamAirKite.jl provides model setup utilities, simulation functions, and configuration structs for simulating ram-air kites. Unlike [V3Kite.jl](https://github.com/OpenSourceAWE/V3Kite.jl) which is tailored for the TU Delft V3 kite with KCU-specific steering/depower percentages, RamAirKite is designed for generic ram-air kite simulations with direct torque control.

## Installation

It is suggested to install this package from git:
```bash
git clone https://github.com/OpenSourceAWE/RamAirKite.jl.git
cd RamAirKite.jl/bin
./install
```
After the installation, launch Julia with
```
cd ..
./bin/run_julia
```
and then, in the Julia REPL run the example with
```julia
include("examples/ram_air_kite.jl")
```

## Quick Start

```julia
using RamAirKite
using SymbolicAWEModels
using GLMakie  # For visualization

# Configure settings
set_data_path(ram_air_data_path())
set = Settings("system.yaml")
set.physical_model = "ram"   # "ram", "simple_ram", or "4_attach_ram"
set.v_wind = 15.51           # m/s
set.l_tether = 50.0          # meters

# 1. system structure  2. model  3. init
sys_struct = create_sys_struct(set)
sam = SymbolicAWEModel(set, sys_struct)
# edit sys_struct here (transforms, tethers, group damping, ...) before init!
init!(sam)
```

See `examples/ram_air_kite.jl` for a full stepping loop with logging, steering,
and replay.

## Physical Model Variants

The package supports three physical model variants, each with different levels of complexity:

### `"ram"` (default)
Standard 4-group ram-air kite with bridle system:
- Full bridle geometry
- 4 deformable wing groups along span
- Suitable for most simulations

### `"simple_ram"`
Simplified model without bridle:
- Direct tether connection to wing
- Single-segment tethers
- Faster simulation, reduced fidelity

### `"4_attach_ram"`
Detailed 4-point bridle attachment:
- 4 deformable wing groups
- Full pulley constraints
- Highest fidelity, slower simulation

## 3-Winch Control System

The ram-air kite uses a 3-winch torque control system:
1. **Power Left Winch**: Controls left side of power line
2. **Power Right Winch**: Controls right side of power line
3. **Steering Winch**: Differential steering control

For simulation, the package provides sinusoidal steering oscillation via `sim_oscillate!()`, which applies torque inputs to create periodic left-right steering motions.

## Configuration Options

```julia
RamAirSimConfig(
    # Model selection
    physical_model = "ram",              # Model variant
    wing_type = QUATERNION,              # QUATERNION or REFINE

    # Simulation parameters
    sim_time = 10.0,                     # Total time [s]
    dt = 0.05,                           # Time step [s]
    vsm_interval = 3,                    # VSM update frequency

    # Environment
    v_wind = 15.51,                      # Wind speed [m/s]
    upwind_dir = -90.0,                  # Upwind direction [deg]

    # Initial conditions
    tether_length = 50.0,                # Tether length [m]
    elevation = nothing,                  # Elevation [deg] or nothing

    # Steering control
    steering_freq = 0.5,                 # Oscillation frequency [Hz]
    steering_magnitude = 1.0,            # Torque magnitude [Nm]
    steering_bias = 0.2,                 # Torque bias [Nm]

    # Model options
    remake_cache = false,                # Force rebuild cache
    brake = true,                        # Winch brake engaged
)
```

## Running Examples

```bash
julia --project=examples examples/ram_air_kite.jl
```

## Dependencies

- **SymbolicAWEModels.jl**: Core symbolic modeling
- **VortexStepMethod.jl**: Aerodynamic calculations
- **KiteUtils.jl**: Common types and utilities
- **GLMakie.jl** (optional): Visualization

## Data Files

The package includes bundled data files in `data/ram_air_kite/`:
- `system.yaml` - System configuration
- `settings.yaml` - Simulation settings
- `vsm_settings.yaml` - Aerodynamic settings
- `ram_air_kite_body.obj` - 3D CAD model
- `ram_air_kite_foil.dat` - Airfoil shape
- `*_polar.csv` - Cl/Cd/Cm polars with deflection angles

## License

MPL-2.0
