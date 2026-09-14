# Changelog

## [Unreleased]

### Changed
- BREAKING: `SymbolicAWEModels` compat is `0.17`, which renames `groups` to
  `stations` and drops the `WING` and `QUASI_STATIC` dynamics types. A wing
  node is a `BODY_STATIC` point riding the wing body, wing membership is
  carried by its station, and `set.quasi_static` no longer selects anything.
  `WING` is no longer re-exported. Examples, tests and factory functions
  updated to match.
- BREAKING: `plot`, `replay` and `record` on a `SystemStructure` need
  `MakieControlPlots` loaded, not just a Makie backend.
- `data/ram_air_kite/vsm_settings.yaml` declares `mesh: {n_sections: 4}`, so the
  ram model's four stations each drive their own aerodynamic strut again.
  SymbolicAWEModels 0.17 slices an `.obj` at panel resolution unless the wing
  settings say otherwise, which gave the wing 41 struts against its 4 stations
  and a parked kite that no longer settled.
- `data/ram_air_kite/ram_air_kite_export.yaml` replaces its `materials` block
  with a `variables` mapping over the material columns, and a wing no longer
  lists its `point_idxs`.
- Reworked the "ram" model bridle: removed the wing-fixed "loose point" so each
  of the 4 stations now uses 4 deforming aerodynamic attachment points
  (previously 3 deforming points plus 1 fixed point).
- Retuned `examples/ram_air_kite.jl` (`AERO_Z_OFFSET`, `POSITION_P`, depower).
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

