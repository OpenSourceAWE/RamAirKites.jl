## Number of points and segments

Here's the breakdown of why both count to **42** — it's a coincidence based on the specific geometry in `create_ram_sys_struct`.

**Points (42 total):**

| Source | Count |
|---|---|
| **Bridle points** — `create_bridle` × 2 (left + right), each adding 15 points: 6 wing attachment points + 9 dynamics/transform points | **30** |
| **Anchor points** — `add_tether!` × 4 (power_left, power_right, steering_left, steering_right), each adding 1 static anchor | **4** |
| **Intermediate tether points** — each tether has `n_segments=3` (from settings.yaml), so `expand_auto_tethers!` generates `n-1 = 2` intermediate dynamic points per tether, × 4 tethers | **8** |
| **Total points** | **42** |

**Segments (42 total):**

| Source | Count |
|---|---|
| **Bridle segments** — `create_bridle` × 2, each adding 15 segments | **30** |
| **Tether segments** — each tether generates `n_segments = 3` segments, × 4 tethers | **12** |
| **Total segments** | **42** |

**Why they're equal:** The bridle contributes the same number of points and segments (15 each per side) because `create_bridle` appends 15 points and 15 segments. Then the tethers add 8 more points but 12 more segments — these happen to bring both totals to the same number:

- **Points:** 30 + 4 + 8 = **42**
- **Segments:** 30 + 12 = **42**