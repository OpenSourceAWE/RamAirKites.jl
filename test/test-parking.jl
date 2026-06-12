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
using DiscretePIDs
toc()

PHYSICAL_MODEL = "ram"       # Options: "ram", "simple_ram", "4_attach_ram"
SIM_TIME = 30.0              # Total simulation time [s]
DT = 0.01                    # Time step [s]
V_WIND = 12.51               # Wind speed [m/s]
UPWIND_DIR = -90.0           # Upwind direction [deg]
TETHER_LENGTH = 50.0         # Tether length [m]
ELEVATION = 74.0             # Initial elevation angle [deg]
PROFILE_LAW = 3              # Wind profile law (3 = EXPLOG)
REMAKE_CACHE = false         # Force rebuild of compiled model cache
VSM_INTERVAL = 7             # VSM update interval
MAX_STEERING = 1.5           # Steering limit [m]

@info "Creating ram air kite model..."
set_data_path(ram_air_data_path())
set = Settings("system.yaml")
set.physical_model = PHYSICAL_MODEL
set.v_wind = V_WIND
set.upwind_dir = UPWIND_DIR
set.profile_law = PROFILE_LAW
set.l_tether = TETHER_LENGTH

@testset "Parking     " begin
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
    sys_struct.tethers[:steering_left].init_stretch_frac = 1.0
    sys_struct.tethers[:steering_right].init_stretch_frac = 1.0

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
    find_steady_state!(sam; dt=0.05, vsm_interval=0)
    toc("Steady state found after: ")

    # Run heading-tracking simulation (parking procedure)
    @info "Starting parking procedure..."
    dt = DT
    steps = Int(round(SIM_TIME / dt))
    torque_damp = 0.9

    logger = Logger(sam, steps + 1)
    sys_state = SysState(sam)
    sys_state.time = 0.0

    steady_torque = Ref(calc_steady_torque(sam))

    for twist_surface in sam.sys_struct.twist_surfaces
        twist_surface.damping = 200.0
    end

    heading_pid = DiscretePID(; K=0.7, Ti=0, Td=0.43, Ts=dt,
                              umin=-MAX_STEERING, umax=MAX_STEERING)
    pos_pid = DiscretePID(; K=4.0, Ti=0.2, Td=0.0005, Ts=dt,
                           umin=-1.2, umax=1.2)
    speed_pid = DiscretePID(; K=6.0, Ti=0.1, Td=0.0, Ts=dt,
                             umin=-40.0, umax=40.0)

    l_diff_prev = Ref(sys_state.l_tether[3] - sys_state.l_tether[4])
    l_diff_speed_filt = Ref(0.0)
    alpha = dt / (dt + 0.14)  # low-pass filter coefficient (SPEED_TAU=0.14)

    azimuth_at_10s = Ref(0.0)
    elevation_at_10s = Ref(0.0)
    heading_at_10s = Ref(0.0)
    azimuth_at_30s = Ref(0.0)
    elevation_at_30s = Ref(0.0)
    heading_at_30s = Ref(0.0)

    for step in 1:steps
        t = step * dt

        current_heading = sam.sys_struct.wings[1].heading
        steering = heading_pid(0, current_heading, 0.0)

        # Cascaded position → speed → torque control
        local l_diff = sys_state.l_tether[3] - sys_state.l_tether[4]
        l_diff_speed_raw = (l_diff - l_diff_prev[]) / dt
        l_diff_prev[] = l_diff
        l_diff_speed_filt[] = alpha * l_diff_speed_raw + (1 - alpha) * l_diff_speed_filt[]
        speed_setpoint = pos_pid(steering, l_diff, 0.0)
        torque = speed_pid(speed_setpoint, l_diff_speed_filt[], 0.0)
        set_values = [0.0, torque, -torque]

        steady_torque[] = torque_damp * steady_torque[] +
                          (1 - torque_damp) * calc_steady_torque(sam)
        set_torques = steady_torque[] .+ set_values

        next_step!(sam; set_values=set_torques, dt, vsm_interval=VSM_INTERVAL)

        update_sys_state!(sys_state, sam)
        sys_state.time = t
        log!(logger, sys_state)

        # Capture state at checkpoints
        if step == Int(round(10.0 / dt))
            azimuth_at_10s[] = rad2deg(sam.sys_struct.wings[1].azimuth)
            elevation_at_10s[] = rad2deg(sam.sys_struct.wings[1].elevation)
            heading_at_10s[] = rad2deg(sam.sys_struct.wings[1].heading)
        elseif step == Int(round(30.0 / dt))
            azimuth_at_30s[] = rad2deg(sam.sys_struct.wings[1].azimuth)
            elevation_at_30s[] = rad2deg(sam.sys_struct.wings[1].elevation)
            heading_at_30s[] = rad2deg(sam.sys_struct.wings[1].heading)
        end
    end

    # Check azimuth and heading at 10s: should already be converging
    @info "At 10s — azimuth: $(round(azimuth_at_10s[], digits=2))°, heading: $(round(heading_at_10s[], digits=2))°"
    @test abs(azimuth_at_10s[]) < 5.0
    @test abs(heading_at_10s[]) < 10.0

    # Check elevation at 10s
    @info "At 10s — elevation: $(round(elevation_at_10s[], digits=2))° (target: $(ELEVATION)° ± 8°)"
    @test abs(elevation_at_10s[] - ELEVATION) < 8.0

    # Check azimuth and heading at 30s: should be well converged
    @info "At 30s — azimuth: $(round(azimuth_at_30s[], digits=2))°, heading: $(round(heading_at_30s[], digits=2))°"
    @test abs(azimuth_at_30s[]) < 5.0
    @test abs(heading_at_30s[]) < 10.0

    # Check elevation at 30s
    @info "At 30s — elevation: $(round(elevation_at_30s[], digits=2))° (target: $(ELEVATION)° ± 8°)"
    @test abs(elevation_at_30s[] - ELEVATION) < 8.0
end
nothing