using Test
using RamAirKite

@testset "Data Path" begin
    path = ram_air_data_path()
    @test isdir(path)
    @test isfile(joinpath(path, "system.yaml"))
    @test isfile(joinpath(path, "settings.yaml"))
    @test isfile(joinpath(path, "vsm_settings.yaml"))
end
