# Copyright (c) 2025 Bart van de Lint
# SPDX-License-Identifier: MPL-2.0

"""
Simulation utility functions ported from SymbolicAWEModels.jl.

These were removed from SymbolicAWEModels when the ram-air kite models were
extracted to dedicated packages (see SymbolicAWEModels changelog).
"""

using SymbolicAWEModels: calc_spring_props

"""
    sim_turn!(sam; dt, total_time, steering_time, steering_magnitude, vsm_interval, prn, lin_model, torque_damp)

Run a turning maneuver simulation.

Applies a one-sided steering torque for `steering_time` seconds, then releases.

# Arguments
- `sam::SymbolicAWEModel`: Initialized AWE model.

# Keywords
- `dt`: Time step [s]. Default `1/sam.set.sample_freq`.
- `total_time`: Simulation duration [s]. Default 10.0.
- `steering_time`: Duration of the steering impulse [s]. Default 2.0.
- `steering_magnitude`: Torque magnitude [N·m]. Default 10.0.
- `vsm_interval`: Steps between VSM updates. Default 3.
- `prn`: Print performance summary. Default false.
- `lin_model`: Optional `StateSpace` for linear comparison.
- `torque_damp`: Torque damping coefficient. Default 0.9.

# Returns
- `(SysLog, Nothing)` or `(SysLog, SysLog)` when `lin_model` is provided.
"""
function sim_turn!(
    sam::SymbolicAWEModel;
    dt=1/sam.set.sample_freq,
    total_time=10.0,
    steering_time=2.0,
    steering_magnitude=10.0,
    vsm_interval=3,
    prn=false,
    lin_model=nothing,
    torque_damp=0.9
)
    steps = Int(round(total_time / dt))
    steering_steps = Int(round(steering_time / dt))
    num_winches = length(sam.sys_struct.winches)
    @assert num_winches == 3 "sim_turn! requires exactly 3 winches, got $num_winches"
    set_values = zeros(Float64, steps, num_winches)

    for step in 1:steps
        if step <= steering_steps
            set_values[step, :] = [0.0, steering_magnitude, -steering_magnitude]
        end
    end

    return sim!(sam, set_values; dt, total_time, vsm_interval, prn, lin_model, torque_damp)
end

"""
    copy_to_simple!(sys::SystemStructure, ssys::SystemStructure)

Copy the dynamic state from a detailed `SystemStructure` to a simplified one.

Maps the state of a complex model (e.g., "ram" with 4 groups and bridle pulleys)
to a simpler model (e.g., "simple_ram" with 2 groups and direct connections).

# Arguments
- `sys::SystemStructure`: The source `ram` model structure.
- `ssys::SystemStructure`: The destination `simple_ram` model structure.
"""
function copy_to_simple!(sys::SystemStructure, ssys::SystemStructure)
    (sys.name != "ram") && @warn "provide a ram sys as the first argument"
    (ssys.name != "simple_ram") && @warn "provide a simple ram sys as the second argument"

    # copy point pos and vel from tether ground attachment points
    for (tether, stether) in zip(sys.tethers, ssys.tethers)
        (length(stether.segment_idxs) != 1) &&
            error("Provide a simple system structure with 1-segment tethers.")
        # copy ground point of the tether
        point_idx = sys.segments[tether.segment_idxs[end]].point_idxs[2]
        spoint_idx = ssys.segments[stether.segment_idxs[1]].point_idxs[2]
        ssys.points[spoint_idx].pos_w .= sys.points[point_idx].pos_w
        ssys.points[spoint_idx].vel_w .= sys.points[point_idx].vel_w
        ssys.points[spoint_idx].disturb .= sys.points[point_idx].disturb
    end

    # copy wing state
    swing = ssys.wings[1]
    wing = sys.wings[1]
    swing.pos_w .= wing.pos_w
    swing.vel_w .= wing.vel_w
    swing.ω_b .= wing.ω_b
    swing.Q_b_to_w .= wing.Q_b_to_w
    # update non-group pos
    ssys.points[1].pos_w .= wing.pos_w + wing.R_b_to_w * ssys.points[1].pos_b
    ssys.points[2].pos_w .= wing.pos_w + wing.R_b_to_w * ssys.points[2].pos_b

    # copy twist (average the two groups on each side)
    (length(sys.groups) != 4) && error("Sys should have 4 groups.")
    (length(ssys.groups) != 2) && error("Simple sys should have 2 groups.")
    ssys.groups[1].twist = (sys.groups[1].twist + sys.groups[2].twist) / 2
    ssys.groups[2].twist = (sys.groups[3].twist + sys.groups[4].twist) / 2
    ssys.groups[1].twist_ω = (sys.groups[1].twist_ω + sys.groups[2].twist_ω) / 2
    ssys.groups[2].twist_ω = (sys.groups[3].twist_ω + sys.groups[4].twist_ω) / 2

    # match moment by changing moment frac
    moment = [group.tether_moment for group in sys.groups]
    moment_frac = sys.groups[1].moment_frac
    moment = [mean(moment[1:2]), mean(moment[3:4])]
    steering_force = [norm(sys.winches[2].force), norm(sys.winches[3].force)]
    for sgroup in ssys.groups
        x_airf = normalize(sgroup.chord)
        init_z_airf = x_airf × sgroup.y_airf
        z_airf = x_airf * sin(sgroup.twist) + init_z_airf * cos(sgroup.twist)
        force = steering_force[sgroup.idx] * normalize(swing.pos_w) ⋅ (swing.R_b_to_w * z_airf)
        r = moment[sgroup.idx] / force
        spoint = ssys.points[sgroup.point_idxs[1]]
        spoint.pos_b .= sgroup.le_pos + sgroup.chord * (r / norm(sgroup.chord) + moment_frac)

        # update pos_w for correct tether len
        chord_b = spoint.pos_b .- sgroup.le_pos
        normal = chord_b × sgroup.y_airf
        pos_b = sgroup.le_pos + cos(sgroup.twist) * chord_b -
                sin(sgroup.twist) * normal
        spoint.pos_w .= swing.pos_w + swing.R_b_to_w * pos_b
    end

    # match winch force by updating tether unstretched length
    for (swinch, winch) in zip(ssys.winches, sys.winches)
        for tether_idx in winch.tether_idxs
            stether = ssys.tethers[tether_idx]
            ssegment = ssys.segments[stether.segment_idxs[1]]
            spoint_idxs = ssegment.point_idxs
            slen = norm(ssys.points[spoint_idxs[1]].pos_w .-
                        ssys.points[spoint_idxs[2]].pos_w)
            stiffness = ssegment.unit_stiffness / slen
            nt = length(winch.tether_idxs)
            stether.len = slen - norm(winch.force) / stiffness / nt
        end
        swinch.vel = winch.vel
    end
end

"""
    copy_to_simple!(sam, tether_sam, simple_sam; prn=true)

Simplify a detailed AWE model into a 1-segment tether model.

1. Calculates equivalent spring properties from the detailed tether model.
2. Assigns these to the single-segment tethers of the simple model.
3. Copies dynamic state (wing position, orientation, attachment points) to the simple model.
4. Reinitializes the simple model from the updated state.

# Arguments
- `sam::SymbolicAWEModel`: The detailed source model.
- `tether_sam::SymbolicAWEModel`: Copy of the detailed model for step response test.
- `simple_sam::SymbolicAWEModel`: The destination simple model to update.

# Keywords
- `prn::Bool=true`: Print progress.

# Returns
- `SymbolicAWEModel`: The updated `simple_sam`.
"""
function copy_to_simple!(sam::SymbolicAWEModel, tether_sam::SymbolicAWEModel,
                         simple_sam::SymbolicAWEModel; prn=true)
    unit_stiffness, unit_damping, _, _ = calc_spring_props(sam, tether_sam; prn)

    for tether in simple_sam.sys_struct.tethers
        segment = simple_sam.sys_struct.segments[tether.segment_idxs[1]]
        segment.unit_stiffness = unit_stiffness[segment.idx]
        segment.unit_damping = unit_damping[segment.idx]
    end
    copy_to_simple!(sam.sys_struct, simple_sam.sys_struct)
    init!(simple_sam; remake=false, reinit_sys=false)
    return simple_sam
end

"""
    sim_oscillate!(sam; dt, total_time, vsm_interval, steering_freq, steering_magnitude, bias, prn, lin_model, torque_damp)

Run a simulation with sinusoidal steering oscillation.

Applies alternating left/right steering torque following a sine wave at
`steering_freq` Hz, with an optional steady `bias` torque.

# Arguments
- `sam::SymbolicAWEModel`: Initialized AWE model.

# Keywords
- `dt`: Time step [s]. Default `1/sam.set.sample_freq`.
- `total_time`: Simulation duration [s]. Default 10.0.
- `vsm_interval`: Steps between VSM updates. Default 3.
- `steering_freq`: Steering oscillation frequency [Hz]. Default 0.5.
- `steering_magnitude`: Peak steering torque magnitude [N·m]. Default 1.0.
- `bias`: Constant torque bias added to the steering [N·m]. Default 0.0.
- `prn`: Print performance summary. Default false.
- `lin_model`: Optional `StateSpace` for linear comparison.
- `torque_damp`: Torque damping coefficient. Default 0.9.

# Returns
- `(SysLog, Nothing)` or `(SysLog, SysLog)` when `lin_model` is provided.
"""
function sim_oscillate!(
    sam::SymbolicAWEModel;
    dt=1/sam.set.sample_freq,
    total_time=10.0,
    vsm_interval=3,
    steering_freq=0.5,
    steering_magnitude=1.0,
    bias=0.0,
    prn=false,
    lin_model=nothing,
    torque_damp=0.9
)
    steps = Int(round(total_time / dt))
    num_winches = length(sam.sys_struct.winches)
    @assert num_winches == 3 "sim_oscillate! requires exactly 3 winches, got $num_winches"
    set_values = zeros(Float64, steps, num_winches)

    for step in 1:steps
        t = step * dt
        steering = steering_magnitude * sin(2π * steering_freq * t) + bias
        set_values[step, :] = [0.0, steering, -steering]
    end

    return sim!(sam, set_values; dt, total_time, vsm_interval, prn, lin_model, torque_damp)
end
