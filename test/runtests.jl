# Copyright (c) 2025 Bart van de Lint
# SPDX-License-Identifier: MPL-2.0

using Test

@testset "RamAirKite.jl" begin
    include("test-data-path.jl")
    include("test-simulation-disabled.jl")
end
