# Changelog

## [Unreleased]

### Changed
- Bumped `SymbolicAWEModels` compat to `0.12`, which renames `groups` to
  `twist_surfaces`; all examples, tests, and factory functions updated to match.
- Reworked the "ram" model bridle: removed the `WING`-fixed "loose point" so each
  of the 4 twist surfaces now uses 4 deforming aerodynamic attachment points
  (previously 3 deforming points plus 1 fixed point).
- Retuned `examples/ram_air_kite.jl` (`AERO_Z_OFFSET`, `POSITION_P`, depower).

### Added
- `examples/ram_air_kite.jl` now warns and stops the run gracefully when a VSM
  solve fails mid-simulation, so the logged data can still be plotted.
