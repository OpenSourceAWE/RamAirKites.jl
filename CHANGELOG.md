# Changelog

## [Unreleased]

### Changed
- Julia 1.11 is no longer supported: the package installs on Julia 1.12 and 1.13,
  the two versions CI tests.
- BREAKING: `SymbolicAWEModels` compat is `0.19`, with VortexStepMethod 6 and
  KiteUtils 0.13. It renames `groups` to `stations` and drops the `WING` and
  `QUASI_STATIC` dynamics types. A wing node is a `BODY_STATIC` point riding the
  wing body, wing membership is carried by its station, and `set.quasi_static` no
  longer selects anything. `WING` is no longer re-exported. The logged aerodynamic
  force is `aero_force_KA`, `SysState` has no `roll`, `pitch` or `yaw`, and
  `sys_struct.total_mass` is gone: each wing reports its own. Examples, tests and
  factory functions updated to match.
- `vsm_settings.yaml` drops the artificial damping keys VortexStepMethod 6 no
  longer reads.
- BREAKING: `plot`, `replay` and `record` on a `SystemStructure` need
  `MakieControlPlots` loaded, not just a Makie backend.
- `data/ram_air_kite/ram_air_kite_export.yaml` replaces its `materials` block
  with a `variables` mapping over the material columns, a wing no longer lists
  its `point_idxs`, and its bridle segments carry the `compression_frac` of 0.1
  that `create_ram_sys_struct` builds.
- The wing carries the kite's 0.9 kg itself as `extra_mass` and its 16 points none,
  so its own COM and inertia come from `ram_air_kite_body.obj`. Its body frame is set
  by reference points: y along the outer leading edges, z up to the inner stations,
  the origin between the inner leading edges. The YAML export and the `ram` and
  `4_attach_ram` factories give the same mass and frame; `simple_ram` gives the mass.
- The factories place their bridle points on the sections `obj_to_yaml` writes, by
  leading-edge distance along the span, instead of VortexStepMethod's `le_interp` and
  `te_interp`, which an OBJ wing no longer has. Points move by up to 0.11 m.
- The factory wings take their VSM solver settings from `vsm_settings.yaml`, as the
  YAML model does.
- The wing's airfoil polars are generated from `ram_air_kite_body.obj` and
  cached under `data/ram_air_kite/obj_geometry/`; `ram_air_kite_foil.dat` and
  the bundled `*_polar.csv` are no longer read.
- Reworked the "ram" model bridle: removed the wing-fixed "loose point" so each
  of the 4 stations now uses 4 deforming aerodynamic attachment points
  (previously 3 deforming points plus 1 fixed point).
- Retuned `examples/ram_air_kite.jl` (`AERO_Z_OFFSET`, `POSITION_P`, depower).
- `examples/ram_air_kite.jl` steers with `HEADING_P = 2.5`: the OBJ wing
  tracks the heading setpoint as the old wing did at 0.8.
- Moved bridle/spring property definitions from `SymbolicAWEModels` into
  `src/simulation_utils.jl`.

### Added
- `examples/ram_air_kite.jl` now warns and stops the run gracefully when a VSM
  solve fails mid-simulation, so the logged data can still be plotted.
- `examples/show_bridle.jl`: interactive 3D visualization of the ram-air kite
  bridle (hover + click), wired into `examples/menu.jl`.
- `examples/steering_test_ram_air.jl`: new example exercising steering.
- Support for loading the kite geometry from a YAML export
  (`data/ram_air_kite/ram_air_kite_export.yaml`), used by the examples and
  covered by `test/test-yaml-load.jl`.

