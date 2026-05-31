# Copyright (c) 2025 Bart van de Lint, Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

"""
Factory functions for building SystemStructure objects for the ram-air kite models.

These functions were previously part of SymbolicAWEModels.jl but were moved here
when the ram-air kite models were extracted to this package.
"""

# Internal helpers from SymbolicAWEModels that are not exported
using SymbolicAWEModels: create_vsm_wing, calc_pos, cad_to_body_frame

# ==================== TETHER CREATION HELPERS ==================== #

"""
    find_axis_point(P, l, v=[0,0,1])

Calculate the coordinates of a point `Q` that lies on a line defined by vector `v`
and is at a distance `l` from a given point `P`.
"""
function find_axis_point(P, l, v=[0,0,1])
    D = (v ⋅ P)^2 - norm(v)^2 * (norm(P)^2 - l^2)
    D < 0 && error("No real solution: l is too small or parameters invalid")
    t = (v ⋅ P - √D) / norm(v)^2
    return [t * v[1], t * v[2], t * v[3]]
end

"""
    create_tether(tether_idx, set, points, segments, tethers, attach_point, type, dynamics_type; z, unit_stiffness, unit_damping, d_pos)

Procedurally create a multi-segment tether connecting `attach_point` on the kite
to a new anchor point on the ground.
"""
function create_tether(tether_idx, set, points, segments, tethers, attach_point,
                       dynamics_type; z=[0,0,1], unit_stiffness=NaN,
                       unit_damping=NaN, d_pos=zeros(3), diameter_mm=NaN)
    winch_pos = find_axis_point(attach_point.pos_cad, set.l_tether, z) .+ d_pos
    dir = winch_pos - attach_point.pos_cad
    segment_idxs = Int64[]
    winch_point_idx = 0
    for i in 1:set.segments
        frac = i / set.segments
        pos = attach_point.pos_cad + frac * dir
        point_idx = length(points) + 1
        segment_idx = length(segments) + 1
        if i == 1
            last_idx = attach_point.name
        else
            last_idx = point_idx - 1
        end
        if i == set.segments
            points = [points; Point(point_idx, pos, STATIC)]
            winch_point_idx = point_idx
        else
            points = [points; Point(point_idx, pos, dynamics_type)]
        end
        segments = [segments; Segment(segment_idx, set, last_idx, point_idx;
                                      unit_stiffness, unit_damping, diameter_mm)]
        push!(segment_idxs, segment_idx)
    end
    tethers = [tethers; Tether(tether_idx, segment_idxs, set.l_tether)]
    return points, segments, tethers, tether_idx, winch_point_idx
end

"""
    bridle_kwargs(set) -> NamedTuple

Return keyword arguments for creating bridle `Segment` objects.

Bridle lines use `set.bridle_tether_diameter` with a stiffness factor of 0.01
relative to the standard tether stiffness, modelling their flexibility.
"""
function bridle_kwargs(set)
    diameter_m = 0.001 * set.bridle_tether_diameter
    unit_stiffness = set.e_tether * (diameter_m / 2)^2 * π * 0.01
    return (diameter_mm=set.bridle_tether_diameter, unit_stiffness=unit_stiffness)
end

# ==================== MODEL FACTORY FUNCTIONS ==================== #

"""
    create_ram_sys_struct(set::Settings; d_winch_pos)

Create a `SystemStructure` for the primary "ram" model with a stability-enhancing bridle.

The model features 4 deformable groups (3 deforming points + 1 static point each),
a complex pulley bridle system, 4 main tethers, and 3 winches.

# Arguments
- `set::Settings`: Configuration parameters from `KiteUtils.jl`.
"""
function create_ram_sys_struct(set::Settings; d_winch_pos=[zeros(3), zeros(3)])
    vsm_set_path = joinpath(get_data_path(), "vsm_settings.yaml")
    vsm_set = VortexStepMethod.VSMSettings(vsm_set_path; data_prefix=false)
    vsm_wing = create_vsm_wing(set, vsm_set; prn=false)
    points = Point[]
    groups = Group[]
    segments = Segment[]
    pulleys = Pulley[]
    tethers = Tether[]
    winches = Winch[]
    wings = VSMWing[]

    attach_points = Point[]

    bridle_top_left = [cad_to_body_frame(vsm_wing, set.top_bridle_points[i]) for i in eachindex(set.top_bridle_points)]
    bridle_top_right = [bridle_top_left[i] .* [1, -1, 1] for i in eachindex(set.top_bridle_points)]

    dynamics_type = set.quasi_static ? QUASI_STATIC : DYNAMIC
    z = vsm_wing.R_cad_body[:, 3]

    function create_bridle(bridle_top, gammas)
        i_pnt = length(points)
        i_seg = length(segments)
        i_pul = length(pulleys)
        i_grp = length(groups)

        points_new = [
            Point(1+i_pnt, calc_pos(vsm_wing, gammas[1], set.bridle_fracs[1]), WING)
            Point(2+i_pnt, calc_pos(vsm_wing, gammas[1], set.bridle_fracs[3]), WING)
            Point(3+i_pnt, calc_pos(vsm_wing, gammas[1], set.bridle_fracs[4]), WING)
            Point(4+i_pnt, calc_pos(vsm_wing, gammas[2], set.bridle_fracs[1]), WING)
            Point(5+i_pnt, calc_pos(vsm_wing, gammas[2], set.bridle_fracs[3]), WING)
            Point(6+i_pnt, calc_pos(vsm_wing, gammas[2], set.bridle_fracs[4]), WING)
        ]
        groups_new = [
            Group(1+i_grp, [1+i_pnt, 2+i_pnt, 3+i_pnt], DYNAMIC, 0.25)
            Group(2+i_grp, [4+i_pnt, 5+i_pnt, 6+i_pnt], DYNAMIC, 0.25)
        ]

        body_frame_damping = 1.0
        points_new = [
            points_new
            Point(7+i_pnt, bridle_top[1], dynamics_type; body_frame_damping, world_frame_damping=0.0)
            Point(8+i_pnt, bridle_top[2], WING)
            Point(9+i_pnt, bridle_top[3], dynamics_type; body_frame_damping, world_frame_damping=0.0)
            Point(10+i_pnt, bridle_top[4], dynamics_type; body_frame_damping, world_frame_damping=0.0)
            Point(11+i_pnt, bridle_top[2] - 1z, dynamics_type; body_frame_damping, world_frame_damping=0.0)
            Point(12+i_pnt, bridle_top[1] - 2z, dynamics_type; body_frame_damping, world_frame_damping=0.0)
            Point(13+i_pnt, bridle_top[3] - 2z, dynamics_type; body_frame_damping, world_frame_damping=0.0)
            Point(14+i_pnt, bridle_top[1] - 4z, dynamics_type; body_frame_damping, world_frame_damping=0.0)
            Point(15+i_pnt, bridle_top[3] - 4z, dynamics_type; body_frame_damping, world_frame_damping=0.0)
        ]
        bk = bridle_kwargs(set)
        segments_new = [
            Segment(1+i_seg, set, 1+i_pnt, 7+i_pnt; bk...)
            Segment(2+i_seg, set, 2+i_pnt, 9+i_pnt; bk...)
            Segment(3+i_seg, set, 3+i_pnt, 10+i_pnt; bk...)
            Segment(4+i_seg, set, 4+i_pnt, 7+i_pnt; bk...)
            Segment(5+i_seg, set, 5+i_pnt, 9+i_pnt; bk...)
            Segment(6+i_seg, set, 6+i_pnt, 10+i_pnt; bk...)
            Segment(7+i_seg, set, 7+i_pnt, 12+i_pnt; bk..., l0=2)
            Segment(8+i_seg, set, 8+i_pnt, 11+i_pnt; bk..., l0=1)
            Segment(9+i_seg, set, 9+i_pnt, 13+i_pnt; bk..., l0=2)
            Segment(10+i_seg, set, 10+i_pnt, 15+i_pnt; bk..., l0=4)
            Segment(11+i_seg, set, 11+i_pnt, 12+i_pnt; bk..., l0=1)
            Segment(12+i_seg, set, 11+i_pnt, 13+i_pnt; bk..., l0=1)
            Segment(13+i_seg, set, 12+i_pnt, 14+i_pnt; bk..., l0=2)
            Segment(14+i_seg, set, 13+i_pnt, 14+i_pnt; bk..., l0=2)
            Segment(15+i_seg, set, 13+i_pnt, 15+i_pnt; bk..., l0=2)
        ]
        pulleys_new = [
            Pulley(1+i_pul, 11+i_seg, 12+i_seg, dynamics_type)
            Pulley(2+i_pul, 14+i_seg, 15+i_seg, dynamics_type)
        ]
        append!(points, points_new)
        append!(groups, groups_new)
        append!(segments, segments_new)
        append!(pulleys, pulleys_new)
        push!(attach_points, points[end-1])
        push!(attach_points, points[end])
        return nothing
    end

    gammas = [-3/4, -1/4, 1/4, 3/4] * vsm_wing.gamma_tip
    create_bridle(bridle_top_left, gammas[[1, 2]])
    create_bridle(bridle_top_right, gammas[[3, 4]])

    points, segments, tethers, left_power_idx, left_power_wp =
        create_tether(1, set, points, segments, tethers, attach_points[1],
                      dynamics_type; z, diameter_mm=set.power_tether_diameter)
    points, segments, tethers, right_power_idx, _ =
        create_tether(2, set, points, segments, tethers, attach_points[3],
                      dynamics_type; z, diameter_mm=set.power_tether_diameter)
    points, segments, tethers, left_steering_idx, left_steering_wp =
        create_tether(3, set, points, segments, tethers, attach_points[2],
                      dynamics_type; z, d_pos=d_winch_pos[1], diameter_mm=set.steering_tether_diameter)
    points, segments, tethers, right_steering_idx, right_steering_wp =
        create_tether(4, set, points, segments, tethers, attach_points[4],
                      dynamics_type; z, d_pos=d_winch_pos[2], diameter_mm=set.steering_tether_diameter)

    winches = [
        Winch(1, set, [left_power_idx, right_power_idx]; winch_point=left_power_wp)
        Winch(2, set, [left_steering_idx]; winch_point=left_steering_wp)
        Winch(3, set, [right_steering_idx]; winch_point=right_steering_wp)
    ]

    vsm_aero = BodyAerodynamics([vsm_wing])
    vsm_solver = Solver(vsm_aero; solver_type=NONLIN, atol=2e-8, rtol=2e-8)
    wings = [VSMWing(1, vsm_aero, vsm_wing, vsm_solver, [1, 2, 3, 4], I(3), zeros(3))]
    transforms = [Transform(1, deg2rad(set.elevation), deg2rad(set.azimuth), deg2rad(set.heading);
                             base_pos=zeros(3), base_point=points[end].name, wing=1)]

    return SystemStructure("ram", set; points, groups, segments, pulleys, tethers, winches, wings, transforms)
end

"""
    create_4_attach_ram_sys_struct(set::Settings)

Create a `SystemStructure` for a ram-air kite with a 4-point attachment bridle.

Similar to `create_ram_sys_struct` but with all four bridle attachment points
deforming with the wing group twist dynamics.

# Arguments
- `set::Settings`: Configuration parameters from `KiteUtils.jl`.
"""
function create_4_attach_ram_sys_struct(set::Settings)
    vsm_set_path = joinpath(get_data_path(), "vsm_settings.yaml")
    vsm_set = VortexStepMethod.VSMSettings(vsm_set_path; data_prefix=false)
    vsm_wing = create_vsm_wing(set, vsm_set; prn=false)
    points = Point[]
    groups = Group[]
    segments = Segment[]
    pulleys = Pulley[]
    tethers = Tether[]
    winches = Winch[]
    wings = VSMWing[]

    attach_points = Point[]

    bridle_top_left = [cad_to_body_frame(vsm_wing, set.top_bridle_points[i]) for i in eachindex(set.top_bridle_points)]
    bridle_top_right = [bridle_top_left[i] .* [1, -1, 1] for i in eachindex(set.top_bridle_points)]

    dynamics_type = set.quasi_static ? QUASI_STATIC : DYNAMIC
    z = vsm_wing.R_cad_body[:, 3]

    function create_bridle(bridle_top, gammas)
        i_pnt = length(points)
        i_seg = length(segments)
        i_pul = length(pulleys)
        i_grp = length(groups)

        points_new = [
            Point(1+i_pnt, calc_pos(vsm_wing, gammas[1], set.bridle_fracs[1]), WING)
            Point(2+i_pnt, calc_pos(vsm_wing, gammas[1], set.bridle_fracs[2]), WING)
            Point(3+i_pnt, calc_pos(vsm_wing, gammas[1], set.bridle_fracs[3]), WING)
            Point(4+i_pnt, calc_pos(vsm_wing, gammas[1], set.bridle_fracs[4]), WING)
            Point(5+i_pnt, calc_pos(vsm_wing, gammas[2], set.bridle_fracs[1]), WING)
            Point(6+i_pnt, calc_pos(vsm_wing, gammas[2], set.bridle_fracs[2]), WING)
            Point(7+i_pnt, calc_pos(vsm_wing, gammas[2], set.bridle_fracs[3]), WING)
            Point(8+i_pnt, calc_pos(vsm_wing, gammas[2], set.bridle_fracs[4]), WING)
        ]
        groups_new = [
            Group(1+i_grp, [1+i_pnt, 2+i_pnt, 3+i_pnt, 4+i_pnt], DYNAMIC, set.bridle_fracs[2])
            Group(2+i_grp, [5+i_pnt, 6+i_pnt, 7+i_pnt, 8+i_pnt], DYNAMIC, set.bridle_fracs[2])
        ]

        body_frame_damping = 1.0
        points_new = [
            points_new
            Point(9+i_pnt, bridle_top[1], dynamics_type; body_frame_damping, world_frame_damping=0.0)
            Point(10+i_pnt, bridle_top[2], dynamics_type; body_frame_damping, world_frame_damping=0.0)
            Point(11+i_pnt, bridle_top[3], dynamics_type; body_frame_damping, world_frame_damping=0.0)
            Point(12+i_pnt, bridle_top[4], dynamics_type; body_frame_damping, world_frame_damping=0.0)
            Point(13+i_pnt, bridle_top[2] - 1z, dynamics_type; body_frame_damping, world_frame_damping=0.0)
            Point(14+i_pnt, bridle_top[1] - 2z, dynamics_type; body_frame_damping, world_frame_damping=0.0)
            Point(15+i_pnt, bridle_top[3] - 2z, dynamics_type; body_frame_damping, world_frame_damping=0.0)
            Point(16+i_pnt, bridle_top[1] - 4z, dynamics_type; body_frame_damping, world_frame_damping=0.0)
            Point(17+i_pnt, bridle_top[3] - 4z, dynamics_type; body_frame_damping, world_frame_damping=0.0)
        ]
        bk = bridle_kwargs(set)
        segments_new = [
            Segment(1+i_seg, set, 1+i_pnt, 9+i_pnt; bk...)
            Segment(2+i_seg, set, 2+i_pnt, 10+i_pnt; bk...)
            Segment(3+i_seg, set, 3+i_pnt, 11+i_pnt; bk...)
            Segment(4+i_seg, set, 4+i_pnt, 12+i_pnt; bk...)
            Segment(5+i_seg, set, 5+i_pnt, 9+i_pnt; bk...)
            Segment(6+i_seg, set, 6+i_pnt, 10+i_pnt; bk...)
            Segment(7+i_seg, set, 7+i_pnt, 11+i_pnt; bk...)
            Segment(8+i_seg, set, 8+i_pnt, 12+i_pnt; bk...)
            Segment(9+i_seg, set, 9+i_pnt, 14+i_pnt; bk..., l0=2)
            Segment(10+i_seg, set, 10+i_pnt, 13+i_pnt; bk..., l0=1)
            Segment(11+i_seg, set, 11+i_pnt, 15+i_pnt; bk..., l0=2)
            Segment(12+i_seg, set, 12+i_pnt, 17+i_pnt; bk..., l0=4)
            Segment(13+i_seg, set, 13+i_pnt, 14+i_pnt; bk..., l0=1)
            Segment(14+i_seg, set, 13+i_pnt, 15+i_pnt; bk..., l0=1)
            Segment(15+i_seg, set, 14+i_pnt, 16+i_pnt; bk..., l0=2)
            Segment(16+i_seg, set, 15+i_pnt, 16+i_pnt; bk..., l0=2)
            Segment(17+i_seg, set, 15+i_pnt, 17+i_pnt; bk..., l0=2)
        ]
        pulleys_new = [
            Pulley(1+i_pul, 13+i_seg, 14+i_seg, dynamics_type)
            Pulley(2+i_pul, 16+i_seg, 17+i_seg, dynamics_type)
        ]
        append!(points, points_new)
        append!(groups, groups_new)
        append!(segments, segments_new)
        append!(pulleys, pulleys_new)
        push!(attach_points, points[end-1])
        push!(attach_points, points[end])
        return nothing
    end

    gammas = [-3/4, -1/4, 1/4, 3/4] * vsm_wing.gamma_tip
    create_bridle(bridle_top_left, gammas[[1, 2]])
    create_bridle(bridle_top_right, gammas[[3, 4]])

    points, segments, tethers, left_power_idx, left_power_wp =
        create_tether(1, set, points, segments, tethers, attach_points[1],
                      dynamics_type; z, diameter_mm=set.power_tether_diameter)
    points, segments, tethers, right_power_idx, _ =
        create_tether(2, set, points, segments, tethers, attach_points[3],
                      dynamics_type; z, diameter_mm=set.power_tether_diameter)
    points, segments, tethers, left_steering_idx, left_steering_wp =
        create_tether(3, set, points, segments, tethers, attach_points[2],
                      dynamics_type; z, diameter_mm=set.steering_tether_diameter)
    points, segments, tethers, right_steering_idx, right_steering_wp =
        create_tether(4, set, points, segments, tethers, attach_points[4],
                      dynamics_type; z, diameter_mm=set.steering_tether_diameter)

    winches = [
        Winch(1, set, [left_power_idx, right_power_idx]; winch_point=left_power_wp)
        Winch(2, set, [left_steering_idx]; winch_point=left_steering_wp)
        Winch(3, set, [right_steering_idx]; winch_point=right_steering_wp)
    ]

    vsm_aero = BodyAerodynamics([vsm_wing])
    vsm_solver = Solver(vsm_aero; solver_type=NONLIN, atol=2e-8, rtol=2e-8)
    wings = [VSMWing(1, vsm_aero, vsm_wing, vsm_solver, [1, 2, 3, 4], I(3), zeros(3))]
    transforms = [Transform(1, deg2rad(set.elevation), deg2rad(set.azimuth), deg2rad(set.heading);
                             base_pos=zeros(3), base_point=points[end].name, wing=1)]

    return SystemStructure("4_attach_ram", set; points, groups, segments, pulleys, tethers, winches, wings, transforms)
end

"""
    create_simple_ram_sys_struct(set::Settings; unit_stiffness, unit_damping)

Create a simplified `SystemStructure` for a ram-air kite with direct tether connections.

Simplified bridle without pulley system. Each tether is a single segment.

# Arguments
- `set::Settings`: Configuration parameters.
"""
function create_simple_ram_sys_struct(set::Settings;
                                      unit_stiffness=fill(NaN, 4),
                                      unit_damping=fill(NaN, 4))
    set.segments = 1
    vsm_set_path = joinpath(get_data_path(), "vsm_settings.yaml")
    vsm_set = VortexStepMethod.VSMSettings(vsm_set_path; data_prefix=false)
    vsm_wing = create_vsm_wing(set, vsm_set; prn=false)
    gammas = [-1/2, 1/2] * vsm_wing.gamma_tip

    bridle_top_left = [vsm_wing.R_cad_body * (set.top_bridle_points[i] + vsm_wing.T_cad_body) for i in eachindex(set.top_bridle_points)]
    bridle_top_right = [bridle_top_left[i] .* [1, -1, 1] for i in eachindex(set.top_bridle_points)]

    points = [
        Point(1, bridle_top_left[2], WING)
        Point(2, bridle_top_right[2], WING)
        Point(3, calc_pos(vsm_wing, gammas[1], set.bridle_fracs[4]), WING)
        Point(4, calc_pos(vsm_wing, gammas[2], set.bridle_fracs[4]), WING)
        Point(5, [0, 0, -set.l_tether], STATIC)
        Point(6, [0, 0, -set.l_tether], STATIC)
        Point(7, [0, 0, -set.l_tether], STATIC)
        Point(8, [0, 0, -set.l_tether], STATIC)
    ]
    groups = [
        Group(1, [3], DYNAMIC, 0.25)
        Group(2, [4], DYNAMIC, 0.25)
    ]
    segments = [
        Segment(1, set, 1, 5; unit_stiffness=unit_stiffness[1], unit_damping=unit_damping[1], diameter_mm=set.power_tether_diameter)
        Segment(2, set, 2, 6; unit_stiffness=unit_stiffness[2], unit_damping=unit_damping[2], diameter_mm=set.power_tether_diameter)
        Segment(3, set, 3, 7; unit_stiffness=unit_stiffness[3], unit_damping=unit_damping[3], diameter_mm=set.steering_tether_diameter)
        Segment(4, set, 4, 8; unit_stiffness=unit_stiffness[4], unit_damping=unit_damping[4], diameter_mm=set.steering_tether_diameter)
    ]
    tethers = [
        Tether(1, [1], set.l_tether)
        Tether(2, [2], set.l_tether)
        Tether(3, [3], set.l_tether)
        Tether(4, [4], set.l_tether)
    ]
    winches = [
        Winch(1, set, [1, 2]; winch_point=5)
        Winch(2, set, [3]; winch_point=7)
        Winch(3, set, [4]; winch_point=8)
    ]
    vsm_aero = BodyAerodynamics([vsm_wing])
    vsm_solver = Solver(vsm_aero; solver_type=NONLIN, atol=2e-8, rtol=2e-8)
    wings = [VSMWing(1, vsm_aero, vsm_wing, vsm_solver, [1, 2], I(3), zeros(3))]
    transforms = [
        Transform(1, deg2rad(set.elevation), deg2rad(set.azimuth), deg2rad(set.heading);
                  base_pos=zeros(3), base_point=5, wing=1)
    ]

    return SystemStructure("simple_ram", set;
        points, groups, segments, tethers, winches, wings, transforms)
end

"""
    create_tether_sys_struct(set::Settings; unit_stiffness, unit_damping)

Create a simplified `SystemStructure` for testing tether dynamics only.

4 independent tethers, each a dynamic point connected to a fixed anchor.
No wing or bridle system.

# Arguments
- `set::Settings`: Configuration parameters.
"""
function create_tether_sys_struct(set::Settings;
                                  unit_stiffness=fill(NaN, 4),
                                  unit_damping=fill(NaN, 4))
    points = [
        Point(1, zeros(3), DYNAMIC; fix_sphere=true)
        Point(2, zeros(3), DYNAMIC; fix_sphere=true)
        Point(3, zeros(3), DYNAMIC; fix_sphere=true)
        Point(4, zeros(3), DYNAMIC; fix_sphere=true)
    ]
    segments = Segment[]
    tethers = Tether[]

    points, segments, tethers, left_power_idx, left_power_wp =
        create_tether(1, set, points, segments, tethers, points[1], DYNAMIC;
                      unit_stiffness=unit_stiffness[1], unit_damping=unit_damping[1],
                      diameter_mm=set.power_tether_diameter)
    points, segments, tethers, right_power_idx, _ =
        create_tether(2, set, points, segments, tethers, points[2], DYNAMIC;
                      unit_stiffness=unit_stiffness[2], unit_damping=unit_damping[2],
                      diameter_mm=set.power_tether_diameter)
    points, segments, tethers, left_steering_idx, left_steering_wp =
        create_tether(3, set, points, segments, tethers, points[3], DYNAMIC;
                      unit_stiffness=unit_stiffness[3], unit_damping=unit_damping[3],
                      diameter_mm=set.steering_tether_diameter)
    points, segments, tethers, right_steering_idx, right_steering_wp =
        create_tether(4, set, points, segments, tethers, points[4], DYNAMIC;
                      unit_stiffness=unit_stiffness[4], unit_damping=unit_damping[4],
                      diameter_mm=set.steering_tether_diameter)

    winches = [
        Winch(1, set, [left_power_idx, right_power_idx]; brake=true, winch_point=left_power_wp)
        Winch(2, set, [left_steering_idx]; brake=true, winch_point=left_steering_wp)
        Winch(3, set, [right_steering_idx]; brake=true, winch_point=right_steering_wp)
    ]
    transforms = [Transform(1, deg2rad(set.elevation), deg2rad(set.azimuth), deg2rad(set.heading);
                             base_pos=zeros(3), base_point=points[end].name, rot_point=1)]

    return SystemStructure("tether", set; points, segments, tethers, winches, transforms)
end

"""
    create_sys_struct(set::Settings; kwargs...)

Dispatcher that calls the appropriate factory function based on `set.physical_model`.
"""
function create_sys_struct(set::Settings; kwargs...)
    model = set.physical_model
    if model == "ram"
        return create_ram_sys_struct(set; kwargs...)
    elseif model == "simple_ram"
        return create_simple_ram_sys_struct(set; kwargs...)
    elseif model == "4_attach_ram"
        return create_4_attach_ram_sys_struct(set; kwargs...)
    elseif model == "tether"
        return create_tether_sys_struct(set; kwargs...)
    else
        error("Unknown physical_model: \"$model\". Supported: \"ram\", \"simple_ram\", \"4_attach_ram\", \"tether\"")
    end
end
