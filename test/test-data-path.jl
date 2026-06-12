# Copyright (c) 2025 Bart van de Lint, Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

using Test
using RamAirKite
using Pkg

if !isnothing(Pkg.project().name)
    @info "Activating test environment"
    using TestEnv
    TestEnv.activate()
end

@testset "Data Path" begin
    path = ram_air_data_path()
    @test isdir(path)
    @test isfile(joinpath(path, "system.yaml"))
    @test isfile(joinpath(path, "settings.yaml"))
    @test isfile(joinpath(path, "vsm_settings.yaml"))
end
nothing
