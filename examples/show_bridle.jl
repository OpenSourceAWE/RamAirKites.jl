# Copyright (c) 2026 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0
#
# Interactive 3D visualization of the ram-air kite bridle (hover + click).
# Adapted from HybridWings.jl/examples/show_bridle.jl, reduced to the parts
# needed for the ram_air_kite wing.

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using DataFrames, GeometryBasics, SymbolicAWEModels, YAML
using GLMakie
using LinearAlgebra
using GeometryBasics: Point2f, Point3f
using VortexStepMethod: VSMSettings
using KiteUtils: set_data_path, get_data_path, Settings

data_dir  = "ram_air_kite"
data_path = joinpath(@__DIR__, "..", "data", data_dir)
yaml_path = joinpath(data_path, "ram_air_kite_export.yaml")
obj_path  = joinpath(data_path, "ram_air_kite_body.obj")
SHOW_3D = true # show 3D visualization of the kite and bridle
SHOW_BODY_FRAME = true # show the body reference frame as three labeled arrows (X, Y, Z)
PULLEY_OFFSET = 0.07 # offset for pulley segments in X direction, also pulley radius [m]
REMOVE_GS = true # remove ground station tether segments (Frontline, Backline) and their bottom endpoints

"""
    body_frame(sys::SystemStructure; verbose=true)

Return the body reference frame of each wing, expressed in CAD coordinates,
as a vector of named tuples `(name, origin, x, y, z)`. Uses `wing.pos_cad`
for the origin and the columns of `wing.R_b_to_c` for the unit vectors.
With `verbose=true` (default) the frames are also printed.
"""
function body_frame(sys::SystemStructure; verbose=true)
    frames = [(name = wing.name,
               origin = Vector(wing.pos_cad),
               x = Vector(wing.R_b_to_c[:, 1]),
               y = Vector(wing.R_b_to_c[:, 2]),
               z = Vector(wing.R_b_to_c[:, 3])) for wing in sys.wings]
    if verbose
        for frame in frames
            println("Body frame of wing $(frame.name) in CAD coordinates:")
            println("  origin: $(round.(frame.origin; digits=4))")
            println("  x-axis: $(round.(frame.x; digits=4))")
            println("  y-axis: $(round.(frame.y; digits=4))")
            println("  z-axis: $(round.(frame.z; digits=4))")
        end
    end
    return frames
end

# ============================================================
# Parse points, segments, and pulleys from the YAML export file
# ============================================================
println("\n--- Parsing YAML export: $(split(yaml_path, '/')[end]) ---")

yaml_dict = YAML.load_file(yaml_path)

# Helper: transpose YAML row-major data into column-major DataFrame
function yaml_section_to_df(section)
    headers = Symbol.(section["headers"])
    data = section["data"]
    cols = [[row[i] for row in data] for i in eachindex(headers)]
    return DataFrame(cols, headers)
end

# --- Points ---
df_points = yaml_section_to_df(yaml_dict["points"])
df_points.name = string.(df_points.name)
select!(df_points, :name, :pos_cad => :pos, :type)
insertcols!(df_points, 1, :idx => 1:nrow(df_points))

println("YAML points: $(nrow(df_points)) entries")

# --- Segments (as string names first) ---
df_segments = yaml_section_to_df(yaml_dict["segments"])
df_segments.name = string.(df_segments.name)
df_segments.point_i_str = string.(df_segments.point_i)
df_segments.point_j_str = string.(df_segments.point_j)
insertcols!(df_segments, 1, :orig_idx => 1:nrow(df_segments))

# --- Remove tether segments (defined in YAML tethers section) ---
if REMOVE_GS && haskey(yaml_dict, "tethers")
    df_tethers = yaml_section_to_df(yaml_dict["tethers"])
    tether_prefixes = [lowercase(string(row.name)) for row in eachrow(df_tethers)]
    tether_mask = falses(nrow(df_segments))
    for (i, seg_name) in enumerate(df_segments.name)
        lname = lowercase(seg_name)
        for prefix in tether_prefixes
            if startswith(lname, prefix)
                tether_mask[i] = true
                break
            end
        end
    end
    df_segments = df_segments[.!tether_mask, :]
    println("Removed $(sum(tether_mask)) tether segments based on YAML tethers section")
end

# --- Twist surfaces (must be read before point filtering) ---
# Point references are stable NAMES (not positional indices), so they survive
# point removal/reordering upstream in import_bridle.jl / csv_to_yaml.jl.
df_twist = nothing
twist_surface_point_names = Set{String}()
if haskey(yaml_dict, "twist_surfaces")
    df_twist = yaml_section_to_df(yaml_dict["twist_surfaces"])
    for row in eachrow(df_twist)
        for ref in row.point_idxs
            push!(twist_surface_point_names, string(ref))
        end
    end
    println("YAML twist_surfaces: $(nrow(df_twist)) entries")
else
    println("No twist_surfaces section in YAML")
end

# Keep only points referenced by remaining segments or twist surfaces.
used_point_names = Set(vcat(df_segments.point_i_str, df_segments.point_j_str))
keep_mask = [n in used_point_names || n in twist_surface_point_names for n in df_points.name]
df_points = df_points[keep_mask, :]
df_points.idx = 1:nrow(df_points)
println("After filtering: $(nrow(df_points)) points, $(nrow(df_segments)) segments")

# Build mapping: original segment index → new segment index (DataFrame row position)
orig_to_new = Dict{Int, Int}(row.orig_idx => i for (i, row) in enumerate(eachrow(df_segments)))

# --- Build name-to-index lookup ---
name_to_idx = Dict(string(row.name) => row.idx for row in eachrow(df_points))

# --- Convert point name references to indices ---
df_segments.p1 = [name_to_idx[n] for n in df_segments.point_i_str]
df_segments.p2 = [name_to_idx[n] for n in df_segments.point_j_str]
select!(df_segments, :name, :p1, :p2, :diameter_mm)
insertcols!(df_segments, 1, :idx => 1:nrow(df_segments))

println("YAML segments: $(nrow(df_segments)) entries")

# --- Pulleys ---
df_points.pulley = falses(nrow(df_points))
df_segments.pulley_id = zeros(Int, nrow(df_segments))

if haskey(yaml_dict, "pulleys")
    df_pulleys = yaml_section_to_df(yaml_dict["pulleys"])
    for row in eachrow(df_pulleys)
        pid = row.name
        si = orig_to_new[row.segment_i]
        sj = orig_to_new[row.segment_j]
        df_segments.pulley_id[si] = pid
        df_segments.pulley_id[sj] = pid
        # Find the shared point index between the two segments
        p1_set = Set([df_segments.p1[si], df_segments.p2[si]])
        p2_set = Set([df_segments.p1[sj], df_segments.p2[sj]])
        shared = intersect(p1_set, p2_set)
        if !isempty(shared)
            for pt in shared
                df_points.pulley[pt] = true
            end
        end
    end
    println("YAML pulleys: $(nrow(df_pulleys)) entries")
else
    println("No pulleys section in YAML")
end

println("\nFirst 5 points:")
display(first(df_points, 5))
println("\nFirst 5 segments:")
display(first(df_segments, 5))

# Helper: minimum distance from point to line segment in 2D
function point_line_segment_distance(p::Point2f, a::Point2f, b::Point2f)
    ab = b - a
    ap = p - a
    t = clamp(dot(ap, ab) / dot(ab, ab), 0, 1)
    return norm(ap - t * ab)
end

# Load a triangulated OBJ file (vertices only, no normals/textures) and return (vertices, faces)
# offset is a 3-element vector applied to every vertex
function load_kite_mesh(obj_path; offset=[0.0, 0.0, 0.0])
    verts = Point3f[]
    faces = GeometryBasics.TriangleFace{Int}[]
    for line in eachline(obj_path)
        line = strip(line)
        if startswith(line, "v ")
            parts = split(line[2:end])
            x, y, z = parse.(Float64, parts)
            push!(verts, Point3f(x + offset[1], y + offset[2], z + offset[3]))
        elseif startswith(line, "f ")
            idxs = [parse(Int, split(group, '/')[1]) for group in split(line[3:end]) if !isempty(strip(group))]
            for i in 3:length(idxs)
                push!(faces, GeometryBasics.TriangleFace(idxs[1], idxs[i-1], idxs[i]))
            end
        end
    end
    return verts, faces
end

function show_bridle(df_points, df_segments; kite_obj_path=nothing, kite_obj_offset=[0.0, 0.0, 0.0], twist_surfaces=nothing, body_frames=nothing)
    # Extract point coordinates into a matrix
    points = reduce(hcat, df_points.pos)'

    fig = Figure()
    gl = GridLayout(fig[1, 1])
    ax = Axis3(gl[1, 1], title="Bridle Visualization (hover + click)", xlabel="X", ylabel="Y", zlabel="Z", aspect=(1, 1, 1), viewmode=:free)

    # Force equal X, Y and Z axis ranges so spheres appear as spheres
    function equalize_xyz_limits!(ax, points)
        DELTA = 0.5
        x_range = extrema(points[:, 1])
        y_range = extrema(points[:, 2])
        z_range = extrema(points[:, 3])
        x_span = x_range[2] - x_range[1]
        y_span = y_range[2] - y_range[1]
        z_span = z_range[2] - z_range[1]
        max_span = max(x_span, y_span, z_span)
        x_center = (x_range[1] + x_range[2]) / 2
        y_center = (y_range[1] + y_range[2]) / 2
        z_center = (z_range[1] + z_range[2]) / 2
        half = max_span / 2
        limits!(ax,
            x_center - half - DELTA, x_center + half + DELTA,
            y_center - half - DELTA, y_center + half + DELTA,
            z_center - half - DELTA, z_center + half + DELTA)
    end

    # View control buttons
    btn_home = Button(gl[2, 1], label="HOME", tellwidth=false,
                       width=80, height=30)
    btn_front = Button(gl[2, 1], label="FRONT", tellwidth=false,
                       width=80, height=30)
    btn_side  = Button(gl[2, 1], label="SIDE", tellwidth=false,
                       width=80, height=30)
    btn_layout = GridLayout(gl[2, 1])
    btn_layout[1, 1] = btn_home
    btn_layout[1, 2] = btn_front
    btn_layout[1, 3] = btn_side

    on(btn_home.clicks) do _
        ax.azimuth[] = init_azimuth
        ax.elevation[] = init_elevation
        ax.zoom_mult[] = init_zoom_mult
        ax.axis_offset[] = init_axis_offset
        limits!(ax, xmin, xmax, ymin, ymax, zmin, zmax)
        ax.xticklabelsvisible[] = true
        ax.xlabelvisible[] = true
        ax.yticklabelsvisible[] = true
        ax.ylabelvisible[] = true
        ax.zticklabelsvisible[] = true
        ax.zlabelvisible[] = true
    end

    on(btn_front.clicks) do _
        ax.azimuth[] = pi
        ax.elevation[] = 0.0
        ax.zoom_mult[] = init_zoom_mult / 1.2
        ax.axis_offset[] = Vec2d(0)
        ax.yticklabelsvisible[] = false
        ax.ylabelvisible[] = false
        ax.xticklabelsvisible[] = true
        ax.xlabelvisible[] = true
    end

    on(btn_side.clicks) do _
        ax.azimuth[] = -pi / 2
        ax.elevation[] = 0.0
        ax.zoom_mult[] = init_zoom_mult / 1.2
        ax.axis_offset[] = Vec2d(0)
        ax.xticklabelsvisible[] = false
        ax.xlabelvisible[] = false
        ax.yticklabelsvisible[] = true
        ax.ylabelvisible[] = true
    end

    # Load and plot kite mesh if path is given
    if kite_obj_path !== nothing
        kite_verts, kite_faces = load_kite_mesh(kite_obj_path; offset=kite_obj_offset)
        mesh!(ax, kite_verts, kite_faces, color=(:lightgray, 0.25), shading=true, transparency=true)
    end
    # Color points by type: WING green, DYNAMIC blue
    scatter_colors = [t == "WING" ? RGBAf(0, 0.8, 0, 1) : RGBAf(0, 0, 1, 1) for t in df_points.type]
    # scatter_sizes = [p ? 12 : 12 for p in df_points.pulley]
    scatter!(ax, points[:, 1], points[:, 2], points[:, 3], color=scatter_colors, markersize=8)

    # Draw circles around pulley points in the X-Z plane with radius PULLEY_OFFSET
    n_theta = 32
    for row in eachrow(df_points)
        row.pulley || continue
        cx, cy, cz = row.pos
        theta = range(0, 2π, length=n_theta+1)
        circle_pts = [Point3f(cx + PULLEY_OFFSET * cos(t), cy, cz + PULLEY_OFFSET * sin(t)) for t in theta]
        lines!(ax, circle_pts, color=:blue, linewidth=1)
    end

    # Build segment list, offsetting pulley endpoints so the two pulley segments separate visually.
    point_pos = Dict{Int, Vector{Float64}}(row.idx => copy(row.pos) for row in eachrow(df_points))

    # Group segments by pulley_id (non-zero entries form pulley pairs)
    pulley_groups = Dict{Int, Vector{Int}}()
    for (i, pid) in enumerate(df_segments.pulley_id)
        if pid > 0
            push!(get!(pulley_groups, pid, Int[]), i)
        end
    end

    # For each pulley pair, find the shared point
    pulley_shared_pt = Dict{Int, Int}()
    for (pid, idxs) in pulley_groups
        if length(idxs) == 2
            i, j = idxs
            shared = intersect(
                Set([df_segments.p1[i], df_segments.p2[i]]),
                Set([df_segments.p1[j], df_segments.p2[j]])
            )
            if !isempty(shared)
                pulley_shared_pt[pid] = first(shared)
            end
        end
    end

    # Determine offset direction for each pulley segment based on the other endpoint
    pulley_offset_dir = Dict{Int, Float64}()
    for (pid, idxs) in pulley_groups
        if length(idxs) == 2 && haskey(pulley_shared_pt, pid)
            shared = pulley_shared_pt[pid]
            for idx in idxs
                row = df_segments[idx, :]
                other = row.p1 == shared ? row.p2 : row.p1
                dir = point_pos[other][1] < point_pos[shared][1] ? -PULLEY_OFFSET : PULLEY_OFFSET
                pulley_offset_dir[idx] = dir
            end
        end
    end

    seg_list = []
    for row in eachrow(df_segments)
        name = row.name
        if row.pulley_id > 0 && haskey(pulley_offset_dir, row.idx)
            dir = pulley_offset_dir[row.idx]
            shared = pulley_shared_pt[row.pulley_id]
            p_shared = point_pos[shared]
            p_off = [p_shared[1] + dir, p_shared[2], p_shared[3]]
            if row.p1 == shared
                push!(seg_list, (name, p_off, point_pos[row.p2]))
            else
                push!(seg_list, (name, point_pos[row.p1], p_off))
            end
        else
            push!(seg_list, (name, point_pos[row.p1], point_pos[row.p2]))
        end
    end

    # Default colors: A-Main, B-Main, C-Main, Br-main in green, rest in red
    special_names = Set(["A-Main", "B-Main", "C-Main", "Br-main",
                          "A-Main_right", "B-Main_right", "C-Main_right", "Br-main_right"])
    default_colors = [name in special_names ? RGBAf(0, 0.8, 0, 1) : RGBAf(1, 0, 0, 1)
                      for (name, _, _) in seg_list]
    seg_colors = Observable(copy(default_colors))
    seg_line_points = Point3f[]
    for (_, p1, p2) in seg_list
        push!(seg_line_points, Point3f(p1), Point3f(p2))
    end
    linesegments!(ax, seg_line_points, color=seg_colors, linewidth=2)

    # --- Draw twist surfaces as black polylines ---
    if twist_surfaces !== nothing
        for (i, pts) in enumerate(twist_surfaces)
            if length(pts) == 2
                # LE → TE: single chord line
                lines!(ax, pts, color=:black, linewidth=3, linestyle=:solid)
            else
                # LE → TE baseline, plus spider lines from LE and TE to intermediate points
                lines!(ax, [pts[1], pts[2]], color=:black, linewidth=3, linestyle=:solid)
                for j in 3:length(pts)
                    lines!(ax, [pts[1], pts[j]], color=:black, linewidth=1, linestyle=:dash)
                    lines!(ax, [pts[2], pts[j]], color=:black, linewidth=1, linestyle=:dash)
                end
            end
        end
    end

    # --- Draw the body reference frame as three labeled arrows (X, Y, Z) ---
    frame_tips = Point3f[]
    if body_frames !== nothing
        spans = [r[2] - r[1] for r in (extrema(points[:, 1]), extrema(points[:, 2]), extrema(points[:, 3]))]
        axis_len = 0.2 * maximum(spans)
        axis_colors = (:red, :green, :blue)
        for frame in body_frames
            origin = Point3f(frame.origin)
            for (axis, label, color) in zip((frame.x, frame.y, frame.z), ("X", "Y", "Z"), axis_colors)
                dir = Vec3f(axis_len .* axis)
                arrows3d!(ax, [origin], [dir], color=color, shaftradius=0.0125, tipradius=0.0375, tiplength=0.1)
                tip = origin + 1.15f0 * dir
                text!(ax, tip, text=label, fontsize=20, color=color,
                      align=(:center, :center), overdraw=true)
                push!(frame_tips, tip)
            end
        end
    end

    # Now that all plots are added, set limits and capture initial state
    limit_points = isempty(frame_tips) ? points : vcat(points, reduce(hcat, frame_tips)')
    equalize_xyz_limits!(ax, limit_points)
    ax.zoom_mult[] = 1.0 / 1.2
    ax.axis_offset[] = Vec2d(0)
    rect = ax.finallimits[]
    init_azimuth = ax.azimuth[]
    init_elevation = ax.elevation[]
    init_zoom_mult = ax.zoom_mult[]
    init_axis_offset = ax.axis_offset[]
    xmin, ymin, zmin = rect.origin
    xmax, ymax, zmax = rect.origin .+ rect.widths
    # Hover tracking
    last_hovered = Observable(-1)
    last_hovered_point = Observable(-1)

    # Labels: segment name and endpoint names at midpoint
    seg_text = Observable("")
    seg_text_pos = Observable(Point2f(0, 0))
    seg_text_visible = Observable(false)
    text!(ax, seg_text, position=seg_text_pos, space=:pixel,
          fontsize=18, color=:black, overdraw=true,
          align=(:center, :bottom), visible=seg_text_visible, transparency=true)

    scene = ax.scene

    # Hover handler: find closest point or segment (points have priority)
    on(events(scene).mouseposition, priority=2) do mp
        min_dist_pt = Inf
        closest_pt = -1
        min_dist_seg = Inf
        closest_seg = -1
        mouse_pos_2d = Point2f(mouseposition_px(scene))

        # Check points first — use a generous 25px radius
        for (i, p) in enumerate(df_points.pos)
            p_2d = Point2f(Makie.project(scene, Point3f(p)))
            dist = norm(mouse_pos_2d - p_2d)
            if dist < min_dist_pt
                min_dist_pt = dist
                closest_pt = i
            end
        end

        hover_pt = (min_dist_pt < 45.0) ? closest_pt : -1

        # Only check segments when no point is hovered
        if hover_pt == -1
            for (i, (_, p1, p2)) in enumerate(seg_list)
                p1_2d = Point2f(Makie.project(scene, Point3f(p1)))
                p2_2d = Point2f(Makie.project(scene, Point3f(p2)))
                dist = point_line_segment_distance(mouse_pos_2d, p1_2d, p2_2d)
                if dist < min_dist_seg
                    min_dist_seg = dist
                    closest_seg = i
                end
            end
        end

        hover_seg = (hover_pt == -1 && min_dist_seg < 30.0) ? closest_seg : -1

        if hover_seg != last_hovered[] || hover_pt != last_hovered_point[]
            new_colors = copy(default_colors)
            if hover_seg != -1
                new_colors[hover_seg] = RGBAf(1, 1, 0, 1)  # highlight yellow
            end
            seg_colors[] = new_colors
            last_hovered[] = hover_seg
            last_hovered_point[] = hover_pt
        end
    end

    # Click handler: show segment name or point index
    on(events(scene).mousebutton, priority=1) do event
        if event.button == Mouse.left && event.action == Mouse.press
            pt = last_hovered_point[]
            if pt != -1
                p = df_points.pos[pt]
                seg_text[] = string(df_points.idx[pt])
                p_2d = Point2f(Makie.project(scene, Point3f(p)))
                seg_text_pos[] = p_2d + Point2f(20, 0)
                seg_text_visible[] = true
            else
                i = last_hovered[]
                if i != -1
                    name, p1, p2 = seg_list[i]
                    mid_2d = Point2f(Makie.project(scene, Point3f((p1 .+ p2) / 2)))
                    seg_text[] = string(name)
                    seg_text_pos[] = mid_2d + Point2f(20, 0)
                    seg_text_visible[] = true
                else
                    seg_text_visible[] = false
                end
            end
        end
    end

    # Camera movement: update label position
    on(scene.camera.view, priority=1) do _
        pt = last_hovered_point[]
        if pt != -1
            p = df_points.pos[pt]
            p_2d = Point2f(Makie.project(scene, Point3f(p)))
            seg_text_pos[] = p_2d + Point2f(20, 0)
        else
            i = last_hovered[]
            if i != -1
                _, p1, p2 = seg_list[i]
                mid_2d = Point2f(Makie.project(scene, Point3f((p1 .+ p2) / 2)))
                seg_text_pos[] = mid_2d + Point2f(20, 0)
            end
        end
    end

    display(fig)
end

if SHOW_3D
    # OBJ offset: the ram_air_kite OBJ uses a different coordinate system
    obj_offset = [-0.58, 0.0, 0.539]

    # Resolve twist surfaces into point coordinate polylines
    twist_surface_lines = Vector{Point3f}[]
    if df_twist !== nothing
        for row in eachrow(df_twist)
            pt_refs = row.point_idxs
            pts = Point3f[]
            valid = true
            for ref in pt_refs
                # Twist surface references are point NAMES (stable across edits)
                pt_name = string(ref)
                if haskey(name_to_idx, pt_name)
                    idx = name_to_idx[pt_name]
                else
                    valid = false
                    break
                end
                pos = df_points.pos[idx]
                push!(pts, Point3f(pos))
            end
            if valid && length(pts) >= 2
                push!(twist_surface_lines, pts)
            end
        end
    end

    # Resolve the body reference frame (origin + x/y/z axes in CAD coordinates)
    body_frames = nothing
    if SHOW_BODY_FRAME
        set_data_path(data_path)
        set = Settings("system.yaml")
        vsm_set = VSMSettings(joinpath(get_data_path(), "vsm_settings.yaml"); data_prefix=false)
        sys_struct = load_sys_struct_from_yaml(
            yaml_path; system_name=data_dir, set=set, vsm_set=vsm_set)
        body_frames = body_frame(sys_struct; verbose=false)
    end

    # Show bridle with kite mesh overlay
    show_bridle(df_points, df_segments; kite_obj_path=obj_path, kite_obj_offset=obj_offset,
                twist_surfaces=twist_surface_lines, body_frames)
end
