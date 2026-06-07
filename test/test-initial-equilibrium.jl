using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Timers
tic()
@info "Loading packages..."
using Test
using RamAirKite
using SymbolicAWEModels
using VortexStepMethod
using LinearAlgebra
toc()

PHYSICAL_MODEL = "ram"      # Options: "ram", "simple_ram", "4_attach_ram"
SIM_TIME = 10.0             # Total simulation time [s]
DT = 0.05                   # Time step [s]
V_WIND = 15.51              # Wind speed [m/s]
UPWIND_DIR = -90.0          # Upwind direction [deg]
TETHER_LENGTH = 50.0        # Tether length [m]
PROFILE_LAW = 3             # Wind profile law (3 = EXPLOG)
REMAKE_CACHE = false        # Force rebuild of compiled model cache
MAX_STEERING = 2.0          # Steering torque limit [Nm]

@info "Creating ram air kite model..."
set_data_path(ram_air_data_path())
set = Settings("system.yaml")
set.physical_model = PHYSICAL_MODEL
set.v_wind = V_WIND
set.upwind_dir = UPWIND_DIR
set.profile_law = PROFILE_LAW
set.l_tether = TETHER_LENGTH

@testset "Initial equilibrium" begin
    # 1. system structure
    global sys_struct
    sys_struct = create_sys_struct(set)
    @test typeof(sys_struct) == SystemStructure{VSMWing{VortexStepMethod.BodyAerodynamics{56, Wing{56, Float64}, Float64}, Wing{56, Float64}, VortexStepMethod.Solver{56, 4, Float64}}}
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

end
toc()
nothing