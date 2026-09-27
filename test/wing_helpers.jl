# Copyright (c) 2025 Bart van de Lint, Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

using LinearAlgebra: norm

"""
    section_area(sections)

Canopy area of the strips between neighbouring sections: the mean chord of each pair times
the leading-edge distance in the y-z plane, so the sweep does not add to it.
"""
function section_area(sections)
    sections = sort(sections; by=section -> section.LE_point[2])
    chord(section) = norm(section.TE_point - section.LE_point)
    return sum((chord(sections[i]) + chord(sections[i+1])) / 2 *
               norm(sections[i+1].LE_point[2:3] - sections[i].LE_point[2:3])
               for i in 1:length(sections)-1)
end

"""
    inertia_tensor(wing)

The wing's own inertia tensor [kg m²] about its own COM in the CAD frame, without the
points it carries.
"""
inertia_tensor(wing) = wing.R_b_to_c * wing.extra_inertia_b * wing.R_b_to_c'

"""
    placed_mass(sys_struct)

The mass [kg] placed on the structure: its wings' and points' `extra_mass`, without the
tether segments.
"""
placed_mass(sys_struct) = sum(wing -> wing.extra_mass, sys_struct.wings) +
                          sum(point -> point.extra_mass, sys_struct.points)
