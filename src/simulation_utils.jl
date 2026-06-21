# Copyright (c) 2025 Bart van de Lint
# SPDX-License-Identifier: MPL-2.0

"""
Simulation utility functions ported from SymbolicAWEModels.jl.

These were removed from SymbolicAWEModels when the ram-air kite models were
extracted to dedicated packages (see SymbolicAWEModels changelog).
"""

import SymbolicAWEModels: update_sys_struct!
using SymbolicAWEModels: OrdinaryDiffEqCore

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

Maps the state of a complex model (e.g., "ram" with 4 twist surfaces and bridle pulleys)
to a simpler model (e.g., "simple_ram" with 2 twist surfaces and direct connections).

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
    # update non-twist-surface pos
    ssys.points[1].pos_w .= wing.pos_w + wing.R_b_to_w * ssys.points[1].pos_b
    ssys.points[2].pos_w .= wing.pos_w + wing.R_b_to_w * ssys.points[2].pos_b

    # copy twist (average the two twist surfaces on each side)
    (length(sys.twist_surfaces) != 4) && error("Sys should have 4 twist surfaces.")
    (length(ssys.twist_surfaces) != 2) && error("Simple sys should have 2 twist surfaces.")
    ssys.twist_surfaces[1].twist = (sys.twist_surfaces[1].twist + sys.twist_surfaces[2].twist) / 2
    ssys.twist_surfaces[2].twist = (sys.twist_surfaces[3].twist + sys.twist_surfaces[4].twist) / 2
    ssys.twist_surfaces[1].twist_ω = (sys.twist_surfaces[1].twist_ω + sys.twist_surfaces[2].twist_ω) / 2
    ssys.twist_surfaces[2].twist_ω = (sys.twist_surfaces[3].twist_ω + sys.twist_surfaces[4].twist_ω) / 2

    # match moment by changing moment frac
    moment = [ts.tether_moment for ts in sys.twist_surfaces]
    moment_frac = sys.twist_surfaces[1].moment_frac
    moment = [mean(moment[1:2]), mean(moment[3:4])]
    steering_force = [norm(sys.winches[2].force), norm(sys.winches[3].force)]

    # Pick the twist surface point that is actually connected to a simple-model tether.
    function tether_attachment_point_idx(ssys::SystemStructure, simple_surface::TwistSurface)
        for point_idx in simple_surface.point_idxs
            for tether in ssys.tethers
                segment = ssys.segments[tether.segment_idxs[1]]
                if point_idx == segment.point_idxs[1]
                    return point_idx
                end
            end
        end
        error("Could not find tether attachment point for simple twist surface $(simple_surface.idx).")
    end

    for simple_surface in ssys.twist_surfaces
        x_airf = normalize(simple_surface.chord)
        init_z_airf = x_airf × simple_surface.y_airf
        z_airf = x_airf * sin(simple_surface.twist) + init_z_airf * cos(simple_surface.twist)
        force = steering_force[simple_surface.idx] * normalize(swing.pos_w) ⋅ (swing.R_b_to_w * z_airf)
        r = moment[simple_surface.idx] / force
        spoint_idx = tether_attachment_point_idx(ssys, simple_surface)
        spoint = ssys.points[spoint_idx]
        spoint.pos_b .= simple_surface.le_pos + simple_surface.chord * (r / norm(simple_surface.chord) + moment_frac)

        # update pos_w for correct tether len
        chord_b = spoint.pos_b .- simple_surface.le_pos
        normal = chord_b × simple_surface.y_airf
        pos_b = simple_surface.le_pos + cos(simple_surface.twist) * chord_b -
                sin(simple_surface.twist) * normal
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

"""
    in_percent_band(x, steady, delta_x, i, p) -> Bool

Helper function to check if a time series has settled within a percentage band.

It checks if all values of the time series `x` from index `i` to the end are
within a tolerance band defined by `p` percent of the total change `delta_x`.
"""
function in_percent_band(x, steady, delta_x, i, p)
    tol = p/100 * abs(delta_x)
    # All subsequent points must be within steady ± p%
    all(abs.(x[i:end] .- steady) .<= tol)
end

"""
    calc_spring_props(sam, tether_sam; prn=false) -> (Vector, Vector, Matrix, Float64)

Calculate the equivalent stiffness and damping, and return the step response data.

This function orchestrates the process by performing a step response test on the
`tether_sam` model and then analyzing the resulting tether length data.

# Arguments
- `sam::SymbolicAWEModel`: The reference model, used for its physical properties.
- `tether_sam::SymbolicAWEModel`: A copy of the model to perform the step test on.

# Keywords
- `prn::Bool=false`: If true, enables printing of intermediate results.

# Returns
- `Tuple{Vector{Float64}, Vector{Float64}, Matrix{Float64}, Float64}`: A tuple containing:
    1.  `unit_stiffness` [N]
    2.  `unit_damping` [Ns]
    3.  `tether_lens` (the step response data)
    4.  `dt` (the simulation time step)
"""
function calc_spring_props(sam::SymbolicAWEModel, tether_sam::SymbolicAWEModel;
                           F_step=-0.1, prn=false)
    find_steady_state!(sam; t=10.0, dt=10.0, vsm_interval=0)
    copy!(sam.sys_struct, tether_sam.sys_struct)
    integrator = tether_sam.integrator
    prob = tether_sam.prob
    isnothing(integrator) && error("tether_sam.integrator is not initialized")
    isnothing(prob) && error("tether_sam.prob is not initialized")
    OrdinaryDiffEqCore.reinit!(integrator; reinit_dae=true)
    update_sys_struct!(prob, integrator, tether_sam.sys_struct)

    F_0 = [-tether_sam.sys_struct.points[i].force for i in 1:4]
    steps = 200
    tether_lens = step_response!(tether_sam, steps, F_step, F_0)
    k_values, c_values = calc_spring_props(sam, tether_lens, F_step; prn)

    dt = 1/sam.set.sample_freq
    return k_values .* tether_lens[:,1], c_values .* tether_lens[:,1], tether_lens, dt
end

"""
    calc_spring_props(sam, tether_lens, F_step; p=5, prn=false) -> (Vector, Vector)

Calculate spring constant `k` and damping coefficient `c` from a step response.

This function analyzes the time series of tether lengths (`tether_lens`) resulting
from a step force (`F_step`) to estimate the parameters of an equivalent second-order
mass-spring-damper system.

# Arguments
- `sam::SymbolicAWEModel`: The model from which to take physical parameters (mass).
- `tether_lens::Matrix{Float64}`: A matrix of tether length time series data.
- `F_step::Float64`: The magnitude of the applied step force.

# Keywords
- `p::Int=5`: The percentage band used to determine the settling time.
- `prn::Bool=false`: If true, enables printing of detailed calculations.

# Returns
- `Tuple{Vector{Float64}, Vector{Float64}}`: A tuple containing two vectors:
    1.  `k_values` (spring constants [N/m])
    2.  `c_values` (damping coefficients [Ns/m])
"""
function calc_spring_props(sam::SymbolicAWEModel, tether_lens, F_step; p=5, prn=false)
    (; tethers, segments) = sam.sys_struct
    set = sam.set
    dt = 1/set.sample_freq

    k_values = zeros(4)
    c_values = zeros(4)

    first_segments = [segments[tether.segment_idxs[1]] for tether in tethers]
    mass_per_meter = [seg.density * π * (seg.diameter / 2)^2
        for seg in first_segments]

    for j in eachindex(tethers)
        tether_len_series = tether_lens[j, :]
        initial_len = tether_len_series[1]
        final_len = tether_len_series[end]
        delta_x_ss = final_len - initial_len

        m = mass_per_meter[j] * 0.5 * tethers[j].stretched_len
        @assert m > 0

        if abs(delta_x_ss) < 1e-6
            @warn "Steady-state change too small for Tether $j; skipping."
            k_values[j] = NaN; c_values[j] = NaN
            continue
        end

        # Spring stiffness
        k = F_step / delta_x_ss
        k_values[j] = k
        ω_n = sqrt(k/m)

        # Find settling time index according to your in_percent_band function:
        T_s_index = -1
        for i in 1:length(tether_len_series)
            if in_percent_band(tether_len_series, final_len, delta_x_ss, i, p)
                T_s_index = i
                break
            end
        end

        if T_s_index == -1
            @warn "Could not find settling time ($p% criterion) for Tether $j; using fallback."
            # Fallback: use time constant method to find tau
            target_len = initial_len + (1 - 1/ℯ) * delta_x_ss
            tau_idx = findfirst(i ->
                (delta_x_ss > 0 && tether_len_series[i] >= target_len) ||
                (delta_x_ss < 0 && tether_len_series[i] <= target_len),
                1:length(tether_len_series))
            if tau_idx === nothing
                @warn "Cannot determine tau for Tether $j"
                c_values[j] = NaN
            else
                tau = (tau_idx-1)*dt
                c = k * tau
                c_values[j] = c
                println("Tether $j fallback c=", c)
            end
            continue
        end

        T_s = (T_s_index - 1) * dt
        # Calculate damping ratio based on variable percentage settling criterion:
        X = -log(p / 100)
        ζ = X / (ω_n * T_s)
        c = 2 * ζ * sqrt(k * m)
        c_values[j] = c
    end

    prn && for j in eachindex(tethers)
        println("Tether $(j): k = $(k_values[j]) N/m, c = $(c_values[j]) Ns/m")
    end
    return k_values, c_values
end

"""
    step_response!(sam, steps, F_step, F_0; abs_tol, consecutive_steps_needed, prn) -> Matrix

Apply a step force to a model and simulate its dynamic response.

This function records the length of each tether over a specified number of simulation
steps. It includes an early exit condition if the system's state settles.

# Arguments
- `sam::SymbolicAWEModel`: The model to be simulated.
- `steps::Int`: The total number of simulation steps.
- `F_step::Float64`: The magnitude of the step force to apply.
- `F_0::Vector{KVec3}`: The initial force vector for each tether attachment point.

# Keywords
- `abs_tol::Float64=1e-6`: Absolute tolerance for the settling check.
- `consecutive_steps_needed::Int=10`: Number of consecutive steps required to be
  within tolerance to be considered settled.
- `prn::Bool=false`: If true, enables printing of status messages.

# Returns
- `Matrix{Float64}`: A matrix where each row corresponds to a tether and each
  column to a time step, containing the tether lengths.
"""
function step_response!(sam::SymbolicAWEModel, steps, F_step, F_0;
              abs_tol=1e-6,
              consecutive_steps_needed=10,
              prn=false)

    (; points, tethers) = sam.sys_struct

    initial_tether_lens = [norm(points[i].pos_w) for i in eachindex(tethers)]
    [points[i].disturb .= F_0[i] .+ F_step * normalize(points[i].pos_w) for i in eachindex(tethers)]

    tether_lens = zeros(length(tethers), steps+1)
    tether_lens[:, 1] .= initial_tether_lens # Store the initial lengths
    settled_steps = 0
    for step in 1:steps
        next_step!(sam; vsm_interval=0)
        for j in eachindex(tethers)
            tether_lens[j, step+1] = norm(points[j].pos_w)
        end
        # Check absolute delta for all tethers
        step_deltas = abs.(tether_lens[:, step+1] .- tether_lens[:, step])
        max_delta = maximum(step_deltas)
        if max_delta < abs_tol
            settled_steps += 1
        else
            settled_steps = 0
        end
        if settled_steps >= consecutive_steps_needed
            prn && println("Stopped at step $step: all tethers within $abs_tol for $consecutive_steps_needed steps.")
            tether_lens[:, step+2:end] .= tether_lens[:, step+1]
            break
        end
    end
    if settled_steps < consecutive_steps_needed
        @warn "Stepping simulation did not settle within the given steps."
    end
    return tether_lens
end
