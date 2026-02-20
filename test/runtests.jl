# Copyright (c) 2025 Bart van de Lint
# SPDX-License-Identifier: MPL-2.0

using Test
using RamAirKite
using Statistics, LinearAlgebra, Printf, Serialization
using ControlSystemsBase
using ModelingToolkit: @variables, t_nounits

# Import SymbolicAWEModels types and functions we need
using SymbolicAWEModels: Point, Segment, Transform, SystemStructure
using SymbolicAWEModels: DYNAMIC, STATIC, BRIDLE
using SymbolicAWEModels: create_simple_ram_sys_struct, update_from_sysstate!
using SymbolicAWEModels: Settings, SysState, linearize!, simple_linearize!
using SymbolicAWEModels: sim_turn!, sim_reposition!, copy_to_simple!
using SymbolicAWEModels: calc_spring_props, calc_winch_force, get_model_name

# Set up data path using RamAirKite's bundled data
tmpdir = mktempdir()
data_path = joinpath(tmpdir, "ram_air_kite")
cp(ram_air_data_path(), data_path; force=true)
set_data_path(data_path)

# Create models
set = Settings("system.yaml")
sam = SymbolicAWEModel(set, "ram")

tether_set = Settings("system.yaml")
tether_sam = SymbolicAWEModel(tether_set, "tether")
init!(tether_sam)

simple_set = Settings("system.yaml")
simple_sam = SymbolicAWEModel(simple_set, "simple_ram")
init!(simple_sam)

original_set = Settings("system.yaml")

function reset!(set::Settings)
    for field in fieldnames(Settings)
        setfield!(set, field, getfield(original_set, field))
    end
    return set
end

@testset "RamAirKite.jl" begin

    @testset "Data Path" begin
        path = ram_air_data_path()
        @test isdir(path)
        @test isfile(joinpath(path, "system.yaml"))
        @test isfile(joinpath(path, "settings.yaml"))
        @test isfile(joinpath(path, "vsm_settings.yaml"))
    end

    @testset "RamAirSimConfig Defaults" begin
        config = RamAirSimConfig()
        @test config.physical_model == "ram"
        @test config.sim_time == 10.0
        @test config.dt == 0.05
        @test config.v_wind == 15.51
        @test config.tether_length == 50.0
        @test config.vsm_interval == 3
        @test config.steering_freq == 0.5
    end

    @testset "Physical Model Variants" begin
        for model in ["ram", "simple_ram", "4_attach_ram"]
            config = RamAirSimConfig(physical_model=model)
            @test config.physical_model == model
        end
    end

    @testset verbose=true "Initialization" begin
        function init_test(elevation, azimuth, heading)
            transform = sam.sys_struct.transforms[1]
            transform.elevation = deg2rad(elevation)
            transform.azimuth = deg2rad(azimuth)
            transform.heading = deg2rad(heading)
            init!(sam)
            ss = SysState(sam)
            @test sam.sys_struct.wings[1].elevation ≈ transform.elevation atol=1e-2
            @test sam.sys_struct.wings[1].azimuth ≈ transform.azimuth atol=1e-2
            @test sam.sys_struct.wings[1].heading ≈ transform.heading atol=1e-2
            @test ss.elevation ≈ transform.elevation atol=1e-2
            @test ss.azimuth ≈ transform.azimuth atol=1e-2
            @test ss.heading ≈ transform.heading atol=1e-2
        end

        @testset "Model types" begin
            @test sam isa SymbolicAWEModel
            @test simple_sam isa SymbolicAWEModel
            @test tether_sam isa SymbolicAWEModel
        end

        @testset "Initialization timing" begin
            init_time = @elapsed init!(sam; prn=true)
            @show init_time
            @test init_time < 700
            init!(sam; prn=true)
            init_time = @elapsed init!(sam; prn=true)
            @test init_time < 0.3
        end

        @testset "Initialization with different angles" begin
            init_test(ones(3)...)
            init_test(zeros(3)...)
        end
    end

    @testset verbose=true "Simulation" begin
        @testset "Oscillating simulation" begin
            function test_for_peak_at_steering_freq(sam, steering_freq)
                dt = 0.01
                sl, _ = sim_oscillate!(sam; total_time=5.0, steering_freq, dt)
                @test sl.syslog.elevation[begin] ≈ deg2rad(set.elevation) atol=1e-2
                @test sl.syslog.azimuth[begin] ≈ deg2rad(set.azimuth) atol=1e-2
                @test sl.syslog.heading[begin] ≈ deg2rad(set.heading) atol=1e-2
                @test isapprox(sl.syslog.time, collect(dt:dt:5.0))

                heading_signal = sl.syslog.heading
                t = sl.syslog.time
                trend = range(heading_signal[1], heading_signal[end], length=length(t))
                signal_detrended = heading_signal .- trend
                ref_sin = sin.(2 * π * steering_freq .* t)
                ref_cos = cos.(2 * π * steering_freq .* t)
                corr_sin = dot(signal_detrended, ref_sin)
                corr_cos = dot(signal_detrended, ref_cos)
                magnitude_at_freq = sqrt(corr_sin^2 + corr_cos^2)
                freq_lower = steering_freq * 0.5
                ref_sin_lower = sin.(2 * π * freq_lower .* t)
                ref_cos_lower = cos.(2 * π * freq_lower .* t)
                mag_lower = sqrt(dot(signal_detrended, ref_sin_lower)^2 + dot(signal_detrended, ref_cos_lower)^2)
                freq_higher = steering_freq * 1.5
                ref_sin_higher = sin.(2 * π * freq_higher .* t)
                ref_cos_higher = cos.(2 * π * freq_higher .* t)
                mag_higher = sqrt(dot(signal_detrended, ref_sin_higher)^2 + dot(signal_detrended, ref_cos_higher)^2)
                @show magnitude_at_freq, mag_lower, mag_higher
                @test magnitude_at_freq > mag_lower && magnitude_at_freq > mag_higher
            end

            reset!(set)
            init!(sam)
            find_steady_state!(sam)
            test_for_peak_at_steering_freq(sam, 0.5)

            init!(sam)
            find_steady_state!(sam)
            copy_to_simple!(sam, tether_sam, simple_sam)
            test_for_peak_at_steering_freq(simple_sam, 0.5)
        end

        @testset "Turning simulation" begin
            function unwrap!(v::AbstractVector, period::Real=2π)
                offset = 0.0
                for i in 2:length(v)
                    diff = v[i] - v[i-1]
                    if diff > period / 2
                        offset -= period
                    elseif diff < -period / 2
                        offset += period
                    end
                    v[i] += offset
                end
                return v
            end
            function calc_heading(steering_time, steering_magnitude)
                reset!(set)
                init!(sam)
                find_steady_state!(sam)
                dt = 0.05
                sl, _ = sim_turn!(sam; total_time=10.0, steering_time, steering_magnitude, dt)
                unwrap!(sl.syslog.heading)
                @test sl.syslog.heading[begin] ≈ 0.0 atol=1e-1
                return sl.syslog.heading[end]
            end
            default_heading = calc_heading(1.0, 10.0)
            @test default_heading ≈ 850 atol=50.0
            short_steer_heading = calc_heading(0.5, 10.0)
            soft_steer_heading = calc_heading(1.0, 5.0)
            @test default_heading - short_steer_heading > 100
            @test default_heading - soft_steer_heading > 100
            @show default_heading, short_steer_heading, soft_steer_heading
        end

        @testset "Reposition simulation" begin
            reset!(set)
            init!(sam)
            find_steady_state!(sam)
            target_elevation = deg2rad(50)
            target_azimuth = deg2rad(10)
            target_heading = deg2rad(10)
            lg = sim_reposition!(
                sam;
                total_time=5.0,
                reposition_interval_s=1/set.sample_freq,
                target_elevation,
                target_azimuth,
                target_heading,
                prn=false
            )
            @test lg.syslog.heading[end] ≈ target_heading atol=0.05
            @test lg.syslog.elevation[end] ≈ target_elevation atol=0.05
            @test lg.syslog.azimuth[end] ≈ target_azimuth atol=0.05
        end
    end

    @testset verbose=true "Tether properties" begin
        @testset "Tether spring properties" begin
            reset!(set)
            set.sample_freq = 600
            set.abs_tol = 1e-6
            set.rel_tol = 1e-6
            set.segments = 1
            one_seg_sam = SymbolicAWEModel(set, "ram")
            init!(one_seg_sam)
            one_seg_tether_sam = SymbolicAWEModel(set, "tether")
            init!(one_seg_tether_sam)

            axial_stiffness, axial_damping =
                calc_spring_props(one_seg_sam, one_seg_tether_sam)
            next_step!(one_seg_sam; dt=1.0)
            axial_stiffness, axial_damping =
                calc_spring_props(one_seg_sam, one_seg_tether_sam)
            segments = one_seg_sam.sys_struct.segments
            tethers = one_seg_sam.sys_struct.tethers
            segments = [segments[tether.segment_idxs[1]] for tether in tethers]
            real_axial_stiffness = [segment.axial_stiffness for segment in segments]
            real_axial_damping = [segment.axial_damping for segment in segments]
            @test isapprox(real_axial_stiffness, axial_stiffness; rtol=0.02)
            @test isapprox(real_axial_damping, axial_damping; rtol=0.2)

            println("\n--- Tether Spring Properties ---")
            @printf "%-8s | %-15s %-15s %-10s | %-15s %-15s %-10s\n" "Tether" "Calc. Stiffness" "Real Stiffness" "Error (%)" "Calc. Damping" "Real Damping" "Error (%)"
            println(repeat("-", 100))
            for i in 1:4
                stiffness_err = 100 * abs(axial_stiffness[i] - real_axial_stiffness[i]) / real_axial_stiffness[i]
                damping_err   = 100 * abs(axial_damping[i] - real_axial_damping[i]) / real_axial_damping[i]
                @printf "%-8d | %-15.2f %-15.2f %-10.2f | %-15.2f %-15.2f %-10.2f\n" i axial_stiffness[i] real_axial_stiffness[i] stiffness_err axial_damping[i] real_axial_damping[i] damping_err
            end
            println()
        end

        @testset "Test calc winch force" begin
            reset!(set)
            init!(sam)
            tether_vel = [winch.tether_vel for winch in sam.sys_struct.winches]
            tether_acc = [winch.tether_acc for winch in sam.sys_struct.winches]
            set_values = [winch.set_value for winch in sam.sys_struct.winches]
            winch_force = calc_winch_force(sam.sys_struct, tether_vel, tether_acc, set_values)
            ss = SysState(sam)
            @test all(isapprox(ss.winch_force[1:3], winch_force))
        end

        @testset "Just a tether, without winch or kite" begin
            set.segments = 20
            dynamics_type = DYNAMIC

            points = Point[]
            segments = Segment[]

            points = push!(points, Point(1, zeros(3), STATIC; wing_idx=0))

            segment_idxs = Int[]
            for i in 1:set.segments
                point_idx = i+1
                pos = [0.0, 0.0, i * set.l_tether / set.segments]
                push!(points, Point(point_idx, pos, dynamics_type; wing_idx=0))
                segment_idx = i
                push!(segments, Segment(segment_idx, set, (point_idx-1, point_idx), BRIDLE))
                push!(segment_idxs, segment_idx)
            end

            transforms = [Transform(1, deg2rad(-80), 0.0, 0.0;
                base_pos=[0.0, 0.0, 50.0], base_point_idx=points[1].idx, rot_point_idx=points[end].idx)]
            sys_struct = SystemStructure("tether", set; points, segments, transforms)

            local_sam = SymbolicAWEModel(set, sys_struct)
            init!(local_sam; remake=false)
            sys = local_sam.prob.sys
            @test isapprox(local_sam.integrator[sys.pos[:, end]], [8.682408883346524, 0.0, 0.7596123493895988], atol=1e-2)
            for i in 1:100
                next_step!(local_sam)
            end
            @test local_sam.integrator[sys.pos[1, end]] > 0.8set.l_tether
            @test isapprox(local_sam.integrator[sys.pos[2, end]], 0.0, atol=1.0)
        end
    end

    @testset verbose=true "Linearization" begin
        @testset "Linearize" begin
            old_abs = set.abs_tol
            old_rel = set.rel_tol
            set.abs_tol = 1e-4
            set.rel_tol = 1e-4
            init!(sam)
            init!(simple_sam)

            (; A, B, C, D) = linearize!(simple_sam)
            sys = ss(A,B,C,D)
            norm_A = norm(A)
            res = lsim(sys, repeat([-1.0 0.0 -1.0], 2)', [0.0, 0.5])
            println(res.y[:,2])
            @test isapprox(res.y[:,2],
                [-0.0008037289321365251, 0.0004562826732837309, -0.020711457720341487,
                            -0.0017333135190197818], rtol=0.1)

            find_steady_state!(sam)
            (; A, B, C, D) = simple_linearize!(sam; tstab=1.0)
            sys = ss(A,B,C,D)
            res = lsim(sys, repeat([-1.0 0.0 -1.0], 2)', [0.0, 0.5])
            println(res.y[:,2])
            @test isapprox(res.y[:,2],
                [0.014234402954620558, -0.0005674058560722778, -0.0186760660540293,
                    5.933033873737758], rtol=0.1)

            next_step!(simple_sam; dt=1.0)
            (; A, B, C, D) = linearize!(simple_sam)
            @test !isapprox(norm(A), norm_A; atol=1e-3)

            set.abs_tol = old_abs
            set.rel_tol = old_rel
        end
    end

    @testset verbose=true "Serialization" begin
        @testset "Serialization and Deserialization" begin
            points = [
                Point(1, zeros(3), DYNAMIC; wing_idx=0, transform_idx=1)
                Point(2, ones(3), DYNAMIC; wing_idx=0, transform_idx=1)
            ]
            segments = [
                Segment(1, set, (1,2), BRIDLE)
            ]
            transforms = [Transform(1, zeros(3)...;
                base_pos=zeros(3), base_point_idx=1, rot_point_idx=1)]
            sys_struct = SystemStructure("one_point", set; points, segments, transforms)
            local_sam = SymbolicAWEModel(set, sys_struct)
            model_path = joinpath(get_data_path(), get_model_name(set))

            function test_init_with_reset(create_prob, create_lin_prob, create_control_func)
                println("Create prob: $create_prob \t"*
                        "lin_prob: $create_lin_prob \t"*
                        "control_func: $create_control_func")
                rm(model_path; force=true)
                local_sam = SymbolicAWEModel(set, sys_struct)
                init!(local_sam; create_prob, create_lin_prob, create_control_func, prn=false)
                @test isnothing(local_sam.prob) == !create_prob
                @test isnothing(local_sam.lin_prob) == !create_lin_prob
                @test isnothing(local_sam.control_funcs) == !create_control_func
            end
            test_init_with_reset(false, false, false)
            test_init_with_reset(true, false, false)
            test_init_with_reset(false, true, false)
            test_init_with_reset(false, false, true)

            init!(local_sam; create_prob=true, create_lin_prob=true, create_control_func=true, prn=false)
            init!(local_sam; create_prob=false, create_lin_prob=false, create_control_func=false, prn=false)
            @test !isnothing(local_sam.prob)
            @test !isnothing(local_sam.lin_prob)
            @test !isnothing(local_sam.control_funcs)

            push!(points, Point(3, [0, 0, 2], DYNAMIC; wing_idx=0, transform_idx=1))
            sys_struct2 = SystemStructure("one_point", set; points, segments, transforms)
            local_sam.sys_struct = sys_struct2
            model_path2 = joinpath(get_data_path(), get_model_name(set))
            @test model_path == model_path2
            init!(local_sam; create_prob=false, create_lin_prob=false, create_control_func=false, prn=false)
            @test isnothing(local_sam.prob)
            @test isnothing(local_sam.lin_prob)
            @test isnothing(local_sam.control_funcs)
            init!(local_sam; create_prob=true, create_lin_prob=true, create_control_func=true, prn=false)
            @test !isnothing(local_sam.prob)
            @test !isnothing(local_sam.lin_prob)
            @test !isnothing(local_sam.control_funcs)

            old_ny = length(local_sam.outputs)
            outputs = [local_sam.prob.sys.pos[1,1]]
            init!(local_sam; outputs)
            @test old_ny == 0
            @test length(local_sam.outputs) == 1
            lin_model = linearize!(local_sam)
            @test size(lin_model.C)[1] == 1

            old_ny = length(local_sam.outputs)
            outputs = [local_sam.prob.sys.pos[1,1], local_sam.prob.sys.pos[2,1]]
            init!(local_sam; outputs)
            @test old_ny == 1
            @test length(local_sam.outputs) == 2
            lin_model = linearize!(local_sam)
            @test size(lin_model.C)[1] == 2
        end
    end

    @testset verbose=true "SysState ↔ SystemStructure conversion" begin
        @testset "Basic point position updates" begin
            reset!(set)
            sys = create_simple_ram_sys_struct(set)
            P = length(sys.points)

            ss = SysState{P}()
            for i in 1:P
                ss.X[i] = 10.0 * i
                ss.Y[i] = 5.0 * i
                ss.Z[i] = 20.0 + i
            end

            update_from_sysstate!(sys, ss)

            for point in sys.points
                @test point.pos_w[1] ≈ ss.X[point.idx]
                @test point.pos_w[2] ≈ ss.Y[point.idx]
                @test point.pos_w[3] ≈ ss.Z[point.idx]
                @test all(point.vel_w .≈ 0.0)
                @test all(isnan.(point.force))
            end
        end

        @testset "Wing state updates" begin
            reset!(set)
            sys = create_simple_ram_sys_struct(set)
            P = length(sys.points)

            ss = SysState{P}()
            ss.orient .= [0.9239, 0.3827, 0.0, 0.0]
            ss.elevation = 0.5
            ss.azimuth = 0.2
            ss.heading = 0.1
            ss.vel_kite .= [1.0, 2.0, 3.0]
            ss.turn_rates .= [0.1, 0.2, 0.3]
            ss.AoA = 0.15
            ss.course = 0.25
            ss.v_wind_kite .= [10.0, 0.0, 0.0]

            for i in 1:P
                ss.X[i] = Float32(i)
                ss.Y[i] = Float32(i)
                ss.Z[i] = Float32(10 + i)
            end

            update_from_sysstate!(sys, ss)

            @test length(sys.wings) > 0
            wing = sys.wings[1]
            @test wing.Q_b_w ≈ ss.orient
            @test wing.elevation ≈ ss.elevation
            @test wing.azimuth ≈ ss.azimuth
            @test wing.heading ≈ ss.heading
            @test wing.vel_w ≈ ss.vel_kite
            @test wing.ω_b ≈ ss.turn_rates
            @test wing.aoa ≈ ss.AoA
            @test wing.course ≈ ss.course
            @test wing.v_wind ≈ ss.v_wind_kite

            @test all(isnan.(wing.aero_force_b))
            @test all(isnan.(wing.aero_moment_b))
            @test all(isnan.(wing.tether_force))
            @test all(isnan.(wing.tether_moment))
            @test all(isnan.(wing.va_b))
        end

        @testset "Winch state updates" begin
            reset!(set)
            sys = create_simple_ram_sys_struct(set)
            P = length(sys.points)

            ss = SysState{P}()
            ss.l_tether .= [50.0, 51.0, 52.0, 53.0]
            ss.v_reelout .= [0.5, 0.6, 0.7, 0.8]
            ss.set_torque .= [100.0, 101.0, 102.0, 103.0]

            update_from_sysstate!(sys, ss)

            n_winches = min(length(sys.winches), 4)
            for i in 1:n_winches
                @test sys.winches[i].tether_len ≈ ss.l_tether[i]
                @test sys.winches[i].tether_vel ≈ ss.v_reelout[i]
                @test sys.winches[i].set_value ≈ ss.set_torque[i]
                @test all(isnan.(sys.winches[i].force))
                @test isnan(sys.winches[i].friction)
            end
        end

        @testset "Group twist updates" begin
            reset!(set)
            sys = create_simple_ram_sys_struct(set)
            P = length(sys.points)

            ss = SysState{P}()
            ss.twist_angles .= [0.1, 0.2, 0.3, 0.4]

            update_from_sysstate!(sys, ss)

            n_groups = min(length(sys.groups), 4)
            for i in 1:n_groups
                @test sys.groups[i].twist ≈ ss.twist_angles[i]
                @test sys.groups[i].twist_ω ≈ 0.0
                @test isnan(sys.groups[i].tether_force)
                @test isnan(sys.groups[i].tether_moment)
                @test isnan(sys.groups[i].aero_moment)
            end
        end

        @testset "Round-trip consistency" begin
            reset!(set)
            local_sam = SymbolicAWEModel(set, "simple_ram")
            init!(local_sam)

            ss1 = SysState(local_sam)

            sys2 = create_simple_ram_sys_struct(set)
            update_from_sysstate!(sys2, ss1)

            for point in sys2.points
                @test point.pos_w[1] ≈ ss1.X[point.idx] atol=1e-4
                @test point.pos_w[2] ≈ ss1.Y[point.idx] atol=1e-4
                @test point.pos_w[3] ≈ ss1.Z[point.idx] atol=1e-4
            end

            if length(sys2.wings) > 0 && length(local_sam.sys_struct.wings) > 0
                wing_orig = local_sam.sys_struct.wings[1]
                wing_new = sys2.wings[1]
                @test wing_new.Q_b_w ≈ wing_orig.Q_b_w atol=1e-4
                @test wing_new.elevation ≈ wing_orig.elevation atol=1e-4
                @test wing_new.azimuth ≈ wing_orig.azimuth atol=1e-4
            end
        end

        @testset "Integration with simulation" begin
            reset!(set)
            local_sam = SymbolicAWEModel(set, "simple_ram")
            init!(local_sam)
            find_steady_state!(local_sam)

            for _ in 1:10
                next_step!(local_sam)
            end

            ss = SysState(local_sam)

            sys_fresh = create_simple_ram_sys_struct(set)

            update_from_sysstate!(sys_fresh, ss)

            for i in 1:length(sys_fresh.points)
                @test sys_fresh.points[i].pos_w[1] ≈ local_sam.sys_struct.points[i].pos_w[1] atol=1e-3
                @test sys_fresh.points[i].pos_w[2] ≈ local_sam.sys_struct.points[i].pos_w[2] atol=1e-3
                @test sys_fresh.points[i].pos_w[3] ≈ local_sam.sys_struct.points[i].pos_w[3] atol=1e-3
            end
        end
    end

end
