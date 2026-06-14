# Copyright (c) 2025 Bart van de Lint, Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

# Compute bounding box dimensions and projected area of an OBJ file.

using VortexStepMethod: read_faces
using LinearAlgebra: cross

choice = "ram_air_kite"
file = joinpath(@__DIR__, "..", "data", lowercase(choice), choice * "_body.obj")
file = normpath(file)  # Normalize path for better error messages

@assert isfile(file) "File not found: $file"

println("File: $file")

vertices, faces = read_faces(file)

xs = [v[1] for v in vertices]
ys = [v[2] for v in vertices]
zs = [v[3] for v in vertices]

xmin, xmax = minimum(xs), maximum(xs)
ymin, ymax = minimum(ys), maximum(ys)
zmin, zmax = minimum(zs), maximum(zs)

println("  x range: $xmin  to  $xmax")
println("  y range: $ymin  to  $ymax")
println("  z range: $zmin  to  $zmax")
println()
println("  Width  (x): $(xmax - xmin)")
println("  Height (y): $(ymax - ymin)")
println("  Depth  (z): $(zmax - zmin)")

println()
println("="^60)
println("Planform area (view from above, projected onto xy-plane)")
println("="^60)

let
    area = 0.0
    for face in faces
        v1, v2, v3 = vertices[face[1]], vertices[face[2]], vertices[face[3]]
        n = cross(v2 - v1, v3 - v1)
        # Only count faces pointing upward (positive z)
        if n[3] > 0
            area += n[3] / 2
        end
    end
    println("  Planform area (xy-projection): $area")
end