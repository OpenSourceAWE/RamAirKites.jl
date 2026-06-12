# Copyright (c) 2025 Bart van de Lint, Uwe Fechner
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
using SymbolicAWEModels: update_sys_struct!
using VortexStepMethod
using LinearAlgebra
toc()

PHYSICAL_MODEL = "ram"      # Options: "ram", "simple_ram", "4_attach_ram"
SIM_TIME = 10.0             # Total simulation time [s]
DT = 0.05                   # Time step [s]
V_WIND = 15.51              # Wind speed [m/s]
UPWIND_DIR = -85.0          # Upwind direction [deg]
TETHER_LENGTH = 50.0        # Tether length [m]
ELEVATION = 80.0            # Initial elevation angle [deg]
PROFILE_LAW = 3             # Wind profile law (3 = EXPLOG)
REMAKE_CACHE = false         # If true, force rebuild of compiled model cache
MAX_STEERING = 2.0          # Steering torque limit [Nm]

@info "Creating ram air kite model..."
set_data_path(ram_air_data_path())
set::Settings = Settings("system.yaml")
set.physical_model = PHYSICAL_MODEL
set.v_wind = V_WIND
set.upwind_dir = UPWIND_DIR
set.profile_law = PROFILE_LAW
set.l_tether = TETHER_LENGTH

@testset "Initial equilibrium" begin
    # 1. system structure
    sys_struct = create_sys_struct(set)
    toc("System structure created after: ")
    @test sys_struct isa SystemStructure
    @test sys_struct.wings[1].aero isa AbstractVSMAero
    @test length(sys_struct.points) == 42
    @test length(sys_struct.segments) == 42
    @test length(sys_struct.tethers) == 4
    @test all(t -> t.len ≈ TETHER_LENGTH, sys_struct.tethers)
    @test length(sys_struct.winches) == 3
    @test length(sys_struct.wings) == 1
    @test sys_struct.total_mass ≈ set.mass
    vsm_wing = sys_struct.wings[1].vsm_wing
    wing_area = 2 * vsm_wing.area_interp(vsm_wing.span)
    @test wing_area ≈ 13.01 atol=0.01
    @test vsm_wing.span ≈ 3.27 atol=0.01
    tf = sys_struct.transforms[1]
    @test rad2deg(tf.elevation) ≈ set.elevation
    @test TETHER_LENGTH * sind(set.elevation) ≈ 43.30 atol=0.01

    # 2. model
    sam = SymbolicAWEModel(set, sys_struct)

    # edit sys_struct before init!
    sys_struct.transforms[1].elevation = deg2rad(ELEVATION)
    sys_struct.winches[:power_winch].brake = true
    for point in sam.sys_struct.points
        point.body_frame_damping .= 0.0
    end
    for segment in sam.sys_struct.segments
        segment.compression_frac = 0.01 # relative compression stiffness
    end
    # set init_stretched_frac differently for the front and rear tethers
    depower = 0.009
    sys_struct.tethers[:steering_left].init_stretch_frac = 1.0 - depower
    sys_struct.tethers[:steering_right].init_stretch_frac = 1.0 - depower

    # Setting moment_frac = 0.0 means the moment pivot is at the leading edge. 
    # This effectively zeros out the twist moments from tether forces 
    # (since the moment arm about the LE is zero), which simplifies the initial 
    # equilibrium search by removing twist dynamics as a degree of freedom.
    for twist_surface in sam.sys_struct.twist_surfaces
        twist_surface.moment_frac = 0.0
    end
    toc("Model created after: ")
    # 3. init
    init!(sam; remake=REMAKE_CACHE)
    toc("Model initialized after: ")

    # After init!, find the aerodynamic steady state
    find_steady_state!(sam; dt=0.05, vsm_interval=5)
    toc("Steady state found after: ")

    # Extra stabilization: free steps to dissipate DAE constraint forces
    @info "Stabilizing for 2 seconds..."
    for _ in 1:40
        next_step!(sam; dt=0.05)
    end
    toc("Stabilization done after: ")

    # Sync integrator state → sys_struct fields
    @assert sam.prob !== nothing "Expected sam.prob to be initialized"
    @assert sam.integrator !== nothing "Expected sam.integrator to be initialized"
    update_sys_struct!(sam.prob, sam.integrator, sam.sys_struct)
    forces = [segment.force for segment in sam.sys_struct.segments]
    @test all(f -> 0.05 < f < 300.0, forces)

    # Angle of attack — using the geometric formula from update_sys_state!
    # (atan of apparent wind in body frame), without the twist correction
    # that wing.aoa includes. This matches the SysLog AoA used in plotting.
    wing = sam.sys_struct.wings[1]
    aoa_rad = atan(wing.va_b[3], wing.va_b[1])
    aoa_deg = rad2deg(aoa_rad)
    @debug "Angle of attack (geometric): $(round(aoa_deg; digits=2))°"
    @test 2 < aoa_deg < 15

    # Acceleration
    acc = sam.sys_struct.wings[1].acc_w
    acc_norm = norm(acc)
    @debug "Acceleration magnitude: $(round(acc_norm, digits=2)) m/s²"
    @test acc_norm < 10.0
end
nothing