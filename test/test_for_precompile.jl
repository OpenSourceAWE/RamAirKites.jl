# SPDX-FileCopyrightText: 2026 Bart van de Lint, Uwe Fechner
# SPDX-License-Identifier: MPL-2.0

using GLMakie
using RamAirKite
using RamAirKite: ram_air_data_path
using KiteUtils: Settings, set_data_path
using DiscretePIDs
using SymbolicAWEModels

set_data_path(ram_air_data_path())
set = Settings("system.yaml")
set.physical_model = "ram"
set.v_wind = 12.51
set.upwind_dir = -90.0
set.l_tether = 25.0

sys_struct = create_sys_struct(set)

# Edit sys_struct before init
sys_struct.transforms[1].elevation = deg2rad(74.0)
sys_struct.winches[:power_winch].brake = true

for point in sys_struct.points
    point.body_frame_damping .= 0.0
end
for segment in sys_struct.segments
    segment.compression_frac = 0.01
end
for twist_surface in sys_struct.twist_surfaces
    twist_surface.moment_frac = 0.0
end

sam = SymbolicAWEModel(set, sys_struct)

init!(sam; remake=false)
find_steady_state!(sam; dt=0.05, vsm_interval=0)

# Warm up plotting
fig = Makie.plot(sam.sys_struct)

# Run a short simulation to precompile the hot loop
dt = 0.05
SIM_TIME = 2.0
steps = Int(round(SIM_TIME / dt))
VSM_INTERVAL = 7
torque_damp = 0.9

logger = Logger(sam, steps + 1)
sys_state = SysState(sam)
sys_state.time = 0.0

steady_torque = calc_steady_torque(sam)

for twist_surface in sam.sys_struct.twist_surfaces
    twist_surface.damping = 200.0
end

heading_pid = DiscretePID(; K=0.7, Ti=1.5, Td=0.43, Ts=dt,
                          umin=-1.5, umax=1.5)
pos_pid = DiscretePID(; K=10.0, Ti=2.0, Td=0.0005, Ts=dt,
                       umin=-1.2, umax=1.2)
speed_pid = DiscretePID(; K=14.0, Ti=0.08, Td=0.0, Ts=dt,
                         umin=-40.0, umax=40.0)

l_diff_prev = Ref(sys_state.l_tether[3] - sys_state.l_tether[4])
l_diff_speed_filt = Ref(0.0)
alpha = dt / (dt + 0.16)  # low-pass filter coefficient

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

    global steady_torque = torque_damp * steady_torque +
                           (1 - torque_damp) * calc_steady_torque(sam)
    set_torques = steady_torque .+ set_values

    next_step!(sam; set_values=set_torques, dt, vsm_interval=VSM_INTERVAL)

    update_sys_state!(sys_state, sam)
    sys_state.time = t
    log!(logger, sys_state)
end

mkpath(get_data_path())
save_log(logger, "tmp_run")
syslog = load_log("tmp_run")
sl = syslog.syslog