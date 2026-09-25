# Copyright (c) 2025 Bart van de Lint, Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end
Pkg.instantiate()

using Test
using LinearAlgebra
using RamAirKite
using SymbolicAWEModels
using SymbolicAWEModels: Settings
import VortexStepMethod: VSMSettings

include("wing_helpers.jl")

let
    set_data_path(ram_air_data_path())
    set = Settings("system.yaml")
    set.physical_model = "ram"
    vsm_set = VSMSettings(joinpath(get_data_path(), "vsm_settings.yaml"); data_prefix=false)

    @testset "Wing geometry" begin
        sys_struct = load_sys_struct_from_yaml(
            joinpath(get_data_path(), "ram_air_kite_export.yaml");
            system_name="ram", set=set, vsm_set=vsm_set)
        vsm_wing = sys_struct.wings[1].vsm_wing

        # the 41 sections obj_to_yaml writes for the 40 panels
        @test length(vsm_wing.unrefined_sections) == 41
        @test section_area(vsm_wing.unrefined_sections) ≈ 4.75 atol=0.01
        @test vsm_wing.span ≈ 3.29 atol=0.01
    end

    @testset "YAML and factory wing mass" begin
        yaml_wing = load_sys_struct_from_yaml(
            joinpath(get_data_path(), "ram_air_kite_export.yaml");
            system_name="ram", set=set, vsm_set=vsm_set).wings[1]
        factory_wing = create_sys_struct(set).wings[1]

        @test yaml_wing.extra_mass ≈ set.mass
        @test factory_wing.extra_mass ≈ yaml_wing.extra_mass
        @test norm(yaml_wing.com_offset_b) > 0
        # the factory places the frame's reference points on the VSM sections, up to 0.11 m
        # from the exported ones, which moves the origin and axes by millimetres
        @test factory_wing.com_offset_b ≈ yaml_wing.com_offset_b atol=0.01
        @test inertia_tensor(factory_wing) ≈ inertia_tensor(yaml_wing) atol=1e-6
    end
end
nothing
