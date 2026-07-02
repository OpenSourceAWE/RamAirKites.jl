# Copyright (c) 2025 Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end
Pkg.instantiate()

using Timers
tic()
@info "Loading packages..."
using Test
using RamAirKite
using SymbolicAWEModels
using SymbolicAWEModels: Settings
import VortexStepMethod: VSMSettings
toc()

@info "Setting up YAML smoke test..."
let
    set_data_path(ram_air_data_path())
    set = Settings("system.yaml")
    set.physical_model = "ram"
    set.v_wind = 15.51
    set.upwind_dir = -90.0
    set.profile_law = 3
    set.l_tether = 50.0

    @testset "YAML Load Smoke Test" begin
        # Load VSM settings
        vsm_set_path = joinpath(get_data_path(), "vsm_settings.yaml")
        vsm_set = VSMSettings(vsm_set_path; data_prefix=false)

        # Load system structure from exported YAML
        sys_struct = load_sys_struct_from_yaml(
            joinpath(get_data_path(), "ram_air_kite_export.yaml");
            system_name="ram", set=set, vsm_set=vsm_set)

        toc("YAML loaded after: ")
        @test sys_struct isa SystemStructure
        @test sys_struct.name == "ram"

        # Basic structural invariants
        @test length(sys_struct.points) > 0
        @test length(sys_struct.segments) > 0
        @test length(sys_struct.tethers) > 0
        @test length(sys_struct.wings) > 0
        @test length(sys_struct.twist_surfaces) > 0

        # Expected quantities from the exported model
        @test length(sys_struct.points) == 46
        @test length(sys_struct.segments) == 46
        @test length(sys_struct.tethers) == 4
        @test length(sys_struct.winches) == 3
        @test length(sys_struct.wings) == 1
        @test length(sys_struct.twist_surfaces) == 4
        @test length(sys_struct.pulleys) == 4
        @test length(sys_struct.transforms) == 1

        # Verify all point references in segments resolve correctly
        for seg in sys_struct.segments
            @test length(seg.point_idxs) == 2
            @test seg.point_idxs[1] !== nothing
            @test seg.point_idxs[2] !== nothing
        end

        # Verify tether references
        for tether in sys_struct.tethers
            @test tether.start_point_idx !== nothing
            @test tether.end_point_idx !== nothing
        end

        # === Symbol-keyed lookup assertions ===
        # NamedCollection on NamedTuple lookup (Symbol key)
        @test sys_struct.winches[:power_winch] !== nothing
        @test sys_struct.winches[:steering_left_winch] !== nothing
        @test sys_struct.winches[:steering_right_winch] !== nothing

        @test sys_struct.tethers[:power_left] !== nothing
        @test sys_struct.tethers[:power_right] !== nothing
        @test sys_struct.tethers[:steering_left] !== nothing
        @test sys_struct.tethers[:steering_right] !== nothing

        # Integer-keyed lookups
        @test sys_struct.points[1] !== nothing
        @test sys_struct.points[34] !== nothing
        @test sys_struct.segments[1] !== nothing
        @test sys_struct.segments[46] !== nothing
        @test sys_struct.pulleys[1] !== nothing
        @test sys_struct.pulleys[4] !== nothing
        @test sys_struct.twist_surfaces[1] !== nothing
        @test sys_struct.twist_surfaces[4] !== nothing
        @test sys_struct.wings[1] !== nothing
        @test sys_struct.transforms[1] !== nothing

        # Symbol-keyed lookups for named points
        @test sys_struct.points[:power_left_anchor] !== nothing
        @test sys_struct.points[:steering_right_point_2] !== nothing

        # Verify that Symbol keys from wings.point_idxs resolve correctly
        for wing in sys_struct.wings
            for ref in wing.twist_surface_refs
                @test ref !== nothing
                @test sys_struct.twist_surfaces[ref] !== nothing
            end
            for ref in wing.transform_ref
                @test ref !== nothing
                @test sys_struct.transforms[ref] !== nothing
            end
        end

        @info "YAML export load smoke test passed."
    end
end
nothing
