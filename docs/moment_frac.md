## Meaning of `group.moment_frac`

**`moment_frac`** is the **chordwise rotation point fraction** — a normalized position along the wing chord that defines the **pivot point** for moment calculations in a wing group.

### Physical Meaning

- **Range**: $0$ (leading edge) to $1$ (trailing edge)
- **Units**: dimensionless (fraction of chord)
- **What it does**: Determines **where** along the chord the aerodynamic and tether moments are computed

The pivot position is:

$$
\mathbf{r}_{\text{pivot}} = \mathbf{r}_{LE} + \text{moment\_frac} \cdot \mathbf{c}
$$

where $\mathbf{r}_{LE}$ is the leading-edge position and $\mathbf{c}$ is the chord vector.

### How It's Used

From `group_eqs.jl` in SymbolicAWEModels, `moment_frac` enters the moment arm calculation:

```julia
pos_offset = collect(
    get_pos_b(psys, point_idx) .-
    (gl + get_moment_frac(psys, group.idx) * gc)
)
```

This offset is then used to compute the tether moment:

```julia
tether_moment[i, group.idx] = r_group[i, group.idx] × tether_force[i, group.idx]
```

### In Your Test

```julia
group.moment_frac = 0.0
```

Setting `moment_frac = 0.0` means the moment pivot is at the **leading edge**. This effectively **zeros out the twist moments** from tether forces (since the moment arm about the LE is zero), which simplifies the initial equilibrium search by removing twist dynamics as a degree of freedom.