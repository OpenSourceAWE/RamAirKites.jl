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
