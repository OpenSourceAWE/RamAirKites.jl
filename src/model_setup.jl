# Copyright (c) 2025 Bart van de Lint
# SPDX-License-Identifier: MPL-2.0

"""
Ram air kite model setup utilities.
"""

"""
    segment_stretch_stats(sys_struct::SystemStructure)

Calculate segment stretch statistics for the system structure.

# Returns
- `(max_stretch, mean_stretch, max_idx)`: Maximum relative stretch, mean relative stretch,
  and the index of the segment with maximum stretch
"""
function segment_stretch_stats(sys_struct::SymbolicAWEModels.SystemStructure)
    @unpack segments, points = sys_struct

    stretches = Float64[]
    for seg in segments
        p1_idx, p2_idx = seg.point_idxs
        p1 = points[p1_idx]
        p2 = points[p2_idx]
        actual_len = norm(p2.pos_w - p1.pos_w)
        rest_len = seg.l0
        if rest_len > 0
            relative_stretch = (actual_len - rest_len) / rest_len
            push!(stretches, relative_stretch)
        end
    end

    if isempty(stretches)
        return (0.0, 0.0, 0)
    end

    max_stretch = maximum(stretches)
    mean_stretch = mean(stretches)
    max_idx = argmax(stretches)

    return (max_stretch, mean_stretch, max_idx)
end
