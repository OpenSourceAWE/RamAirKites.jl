# Body frame damping

Here's the meaning of **`body_frame_damping`**:

## Physical Meaning

`body_frame_damping` is a **per-axis viscous damping coefficient** (units: N·s/m) that applies a damping force to point masses **proportional to their velocity relative to the wing**, expressed in the **wing's body-fixed reference frame**.

## How It Works

The damping force follows the standard viscous damping law $F = -c \cdot v$, but applied in a specific way:

1. **Relative velocity** is computed first — the point's velocity minus the wing's velocity:  
   `v_diff = v_point - v_wing`

2. **Transformed to body frame** using the rotation matrix $R_{b \to w}$:  
   `v_diff_b = R_b_to_w' * v_diff_w`

3. **Damping force computed per-axis** in body coordinates:  
   `F_damp_b = body_frame_damping .* v_diff_b`
   
   This means you can have different damping for x, y, z axes in the wing's local frame (e.g., damping normal to the wing surface differently from in-plane).

4. **Transformed back to world frame** and subtracted from the acceleration equation:  
   `acc ~ F / m - F_damp_world - F_world_damping`

## Why It's Used

In the RamAirKite models, `body_frame_damping` is set to `1.0` for bridle points in predefined_structures.jl, while `world_frame_damping = 0.0`. This **stabilizes the simulation** by:

- Damping vibrations of bridle lines in the body frame (relative to the wing)
- Allowing the kite to move freely with the wind (world-frame motion undamped)
- Preventing high-frequency oscillations in the bridle system without artificially slowing the kite's global motion

It's also set to `0.0` in test/example code (`point.body_frame_damping .= 0.0`) — likely to check that the simulation converges to steady state without needing artificial damping.

## Contrast with `world_frame_damping`

| Property | `body_frame_damping` | `world_frame_damping` |
|---|---|---|
| Velocity reference | Relative to wing | Absolute (inertial) |
| Coordinate frame | Wing body frame | World (NED) frame |
| Effect | Damps bridle oscillations | Damps global motion |
| Typical use in RamAirKite | `1.0` (on) | `0.0` (off) |