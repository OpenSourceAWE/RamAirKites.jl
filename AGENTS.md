# RamAirKite.jl Copilot Instructions

## Project Overview

RamAirKite.jl is a Julia package for simulating generic ram-air kites (airborne wind energy). It builds on [SymbolicAWEModels.jl](https://github.com/OpenSourceAWE/SymbolicAWEModels.jl) for the core physics engine and [VortexStepMethod.jl](https://github.com/OpenSourceAWE/VortexStepMethod.jl) for aerodynamic computations. RamAirKite uses generic 3-winch torque control.

## Architecture

### Source Files (in `src/`)

- **`RamAirKite.jl`** — Main module: re-exports from SymbolicAWEModels, includes submodules
- **`predefined_structures.jl`** — Factory functions for building `SystemStructure` objects:
  - `create_ram_sys_struct`: Primary "ram" model with stability-enhancing bridle (4 deformable groups, complex pulley bridle system, 4 main tethers, 3 winches)
  - `create_simple_ram_sys_struct`: Simplified version for faster setup
  - `create_4_attach_ram_sys_struct`: Model with 4 attachment points
  - `create_tether_sys_struct`: Tether-only model for validation
  - `create_sys_struct`: Dispatcher that selects the factory based on `set.physical_model`
- **`model_setup.jl`** — Utilities for adjusting model state:
  - `adjust_tether_length!(sam, length)`: Reposition tether points and winch
  - `adjust_elevation!(sam, elevation)`: Adjust elevation angle
  - `segment_stretch_stats`: Compute tether segment stretch ratios
- **`simulation.jl`** — High-level simulation functions:
  - `RamAirSimConfig`: Configuration struct with fields for physical model variant, wind, tether, steering, and VSM settings
  - `create_ram_air_model(config)`: Build and configure a `SymbolicAWEModel`
  - `run_ram_air_simulation(config)`: Full simulation pipeline (create, init, find steady state, run with steering oscillation)
- **`simulation_utils.jl`** — Lower-level utilities ported from SymbolicAWEModels:
  - `sim_turn!`: Run a turning maneuver simulation with one-sided steering impulse
  - `copy_to_simple!`: Copy state between model variants

### Extension (in `ext/`)

- **`RamAirKiteMakieExt.jl`** — GLMakie visualization extension. Loaded automatically when GLMakie is available. Re-exports plotting from SymbolicAWEModels.

### Configuration Files (in `data/ram_air_kite/`)

- **`settings.yaml`**: Main settings — system params (segments, sample_freq), initial conditions (l_tethers, elevations, v_reel_outs), solver (type, tolerances), kite (model, foil, physical_model, top_bridle_points), and bridle configuration
- **`system.yaml`**: Points to the settings file
- **`vsm_settings.yaml`**: VSM aerodynamic solver configuration — wing definitions (panels, distribution), solver settings (density, iterations, tolerances, relaxation)
- **Foil polars**: `ram_air_kite_foil_cl_polar.csv`, `ram_air_kite_foil_cd_polar.csv`, `ram_air_kite_foil_cm_polar.csv`

## Key Dependencies

| Dependency | Purpose |
|---|---|
| **SymbolicAWEModels.jl** | Core physics engine (symbolic AWE model with DAE solvers) |
| **VortexStepMethod.jl** | Aerodynamic panel method (VSM/LLT) |
| **KiteUtils.jl** | Settings types, state types, shared utilities |
| **GLMakie** (weakdep) | 3D visualization via extension |

All three main dependencies are local path-dependencies in `Project.toml`:
```toml
[sources]
SymbolicAWEModels = {path = "../SymbolicAWEModels.jl"}
VortexStepMethod = {path = "../VortexStepMethod.jl"}
```
Feel free to edit them directly during development. For CI or end-user installation, these should resolve to registered versions as specified in `[compat]`.

## Simulation Workflow

### Basic Usage

```julia
using RamAirKite

# 1. Create configuration
config = RamAirSimConfig(
    physical_model = "ram",       # "ram", "simple_ram", or "4_attach_ram"
    sim_time = 10.0,
    dt = 0.05,
    v_wind = 15.51,
    tether_length = 50.0,
    steering_freq = 0.5,
    steering_magnitude = 1.0,
)

# 2. Create and initialize model
sam = create_ram_air_model(config)
init!(sam; remake=false)

# 3. Find steady state (often disable gravity for VSM convergence)
find_steady_state!(sam; dt=0.05, vsm_interval=0)

# 4. Run simulation with sinusoidal steering
syslog, _ = sim_oscillate!(sam;
    dt = config.dt,
    total_time = config.sim_time,
    vsm_interval = config.vsm_interval,
    steering_freq = config.steering_freq,
    steering_magnitude = config.steering_magnitude,
    bias = config.steering_bias,
    prn = true)
```

### Alternative: High-Level API

```julia
sam, syslog = run_ram_air_simulation(config)
```

### Model Variants

- **`"ram"`**: Full model with bridle system and 2 wing groups (complex, realistic)
- **`"simple_ram"`**: Simplified model with reduced complexity (faster, for testing)
- **`"4_attach_ram"`**: Model with 4 attachment points on the kite

### Key Functions

| Function | Purpose |
|---|---|
| `init!(sam; remake=false)` | Initialize/reinitialize model (remake rebuilds MTK cache) |
| `find_steady_state!(sam; kwargs)` | Solve for aerodynamic equilibrium |
| `sim_oscillate!(sam; kwargs)` | Run sinusoidal steering oscillation simulation |
| `sim_turn!(sam; kwargs)` | Run turning maneuver with one-sided steering impulse |
| `adjust_tether_length!(sam, length)` | Change tether length at runtime |
| `adjust_elevation!(sam, elevation)` | Adjust elevation angle |
| `update_sys_state!(sam)` | Update system state from integrator |
| `log!(sam, syslog)` | Log current state |
| `save_log(syslog, path)` / `load_log(path)` | Persist/restore simulation logs |

## Wing Types

- `QUATERNION`: Quaternion-based orientation (faster, default)
- `REFINE`: Full refine-based orientation (more accurate, slower)

Set via `config.wing_type = SymbolicAWEModels.REFINE` or `QUATERNION`.

## Code Conventions

### Naming

- Types use `CamelCase`: `RamAirSimConfig`, `SystemStructure`
- Functions use `snake_case`: `create_ram_air_model`, `adjust_tether_length!`
- Mutating functions end with `!` per Julia convention
- Internal helpers use lowercase with underscores

### Struct Pattern

Configuration uses `Base.@kwdef mutable struct` for keyword-constructible mutable structs:
```julia
Base.@kwdef mutable struct RamAirSimConfig
    physical_model::String = "ram"
    sim_time::Float64 = 10.0
    # ...
end
```

### Re-exports

Public API types/functions from SymbolicAWEModels are re-exported from the main module:
```julia
using SymbolicAWEModels: SymbolicAWEModel, SystemStructure, Logger, SysState
export SymbolicAWEModel, SystemStructure, Logger, SysState
```

### Coordinate System

- NED (North-East-Down) reference frame
- Wing geometry defined in CAD frame, transformed to body frame via `cad_to_body_frame`
- Upwind direction in degrees (typically -90 for conventional orientation)

## Running Tests

```julia
# Full test suite
include("test/runtests.jl")

# Or from package mode
using Pkg; Pkg.test()
```

Tests exercise all three model variants (`ram`, `simple_ram`, `tether`) and include linearization checks using `ControlSystemsBase` and `ModelingToolkit`.

## Running Examples

```julia
# Activate the examples project first
using Pkg; Pkg.activate("examples")
include("examples/ram_air_kite.jl")
```

The example project has its own `Project.toml` with local path-dependencies.

## Development Workflow

### Local Dependencies

The key dependencies use local paths during development. After cloning adjacent repos:
```bash
git clone https://github.com/OpenSourceAWE/SymbolicAWEModels.jl.git
git clone https://github.com/OpenSourceAWE/VortexStepMethod.jl.git
```

The `Project.toml` `[sources]` section handles the path overrides.

### Adding New Model Variants

1. Add a factory function in `predefined_structures.jl` (following the pattern of `create_ram_sys_struct`)
2. Register it in `create_sys_struct` dispatcher
3. Add any new configuration fields to `Settings` via KiteUtils
4. Add test coverage in `test/runtests.jl`

### Adding New Simulation Config Options

Add fields to `RamAirSimConfig` in `simulation.jl`, then pass them through in `create_ram_air_model` and/or `run_ram_air_simulation`.

### Visualization

Plotting is handled by the `RamAirKiteMakieExt` extension (requires GLMakie):
```julia
using GLMakie, RamAirKite
fig = plot(sam.sys_struct)           # Plot system structure
fig = plot(sam.sys_struct, syslog)   # Plot with trajectory
replay(syslog, sam.sys_struct)       # Interactive replay
```

## External Dependencies Note

The `Project.toml` has `[sources]` overrides for local development of `SymbolicAWEModels` and `VortexStepMethod`. For CI or end-user installation, these should resolve to registered package versions as specified in `[compat]`.