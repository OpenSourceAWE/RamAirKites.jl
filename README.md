# RamAirKite.jl

A Julia package for simulation of generic ram-air kites built on [SymbolicAWEModels.jl](https://github.com/aenarete/SymbolicAWEModels.jl).

## Overview

RamAirKite.jl provides model setup utilities, simulation functions, and configuration structs for simulating ram-air kites. Unlike [V3Kite.jl](https://github.com/aenarete/V3Kite.jl) which is tailored for the TU Delft V3 kite with KCU-specific steering/depower percentages, RamAirKite is designed for generic ram-air kite simulations with direct torque control.

## Installation

```julia
using Pkg
Pkg.develop(path="/path/to/RamAirKite.jl")
```

## Quick Start

```julia
using RamAirKite
using GLMakie  # For visualization

# Create configuration
config = RamAirSimConfig(
    physical_model = "ram",      # "ram", "simple_ram", or "4_attach_ram"
    sim_time = 10.0,             # seconds
    v_wind = 15.51,              # m/s
    tether_length = 50.0,        # meters
    steering_freq = 0.5,         # Hz
    steering_magnitude = 1.0,    # Nm
)

# Run simulation
sam, syslog = run_ram_air_simulation(config)

# Visualize
plot(sam.sys_struct, syslog)
replay(syslog, sam.sys_struct)
```

## Physical Model Variants

The package supports three physical model variants, each with different levels of complexity:

### `"ram"` (default)
Standard 2-group ram-air kite with bridle system:
- Full bridle geometry
- 2 deformable wing groups (left/right)
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
