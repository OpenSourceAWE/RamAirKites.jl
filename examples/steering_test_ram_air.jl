# Copyright (c) 2025 Uwe Fechner, Bart van de Lint
# SPDX-License-Identifier: MPL-2.0
#
# Tests the steering response of the ram-air kite model by applying
# different torque-based steering values and measuring the resulting
# turn rate. Fits a turn-rate law model and plots the results.
#
# Adapted from the older KPS4 steering test to work with RamAirKite.jl's
# generic 3-winch torque control architecture.

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Timers
tic()
@info "Loading packages..."
using DiscretePIDs
using RamAirKite
using SymbolicAWEModels
using KiteUtils
using LinearAlgebra
using Statistics
using Printf
toc()

# ==================== USER PARAMETERS ==================== #

PHYSICAL_MODEL = "ram"       # Options: "ram", "simple_ram", "4_attach_ram"
SIM_TIME = 20.0              # Total simulation time [s]
DT = 0.02                    # Time step [s]
INITIAL_STEERING = -0.012    # Initial steering line length difference [m]
V_WIND = 12.51               # Wind speed [m/s]
UPWIND_DIR = -90.0           # Upwind direction [deg]
TETHER_LENGTH = 50.0        # Tether length [m]
ELEVATION = 74               # Initial elevation angle [deg]
VSM_INTERVAL = 3             # VSM update interval (steps)
OFFSET_DEG = 4.0             # Heading offset for direction reversal [deg]
STEERING_SEQ = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, -0.8, 0.9, -0.9, 1.0, -1.0] .* 0.3  # Steering setpoint sequence [m]

# Cascaded position → speed → torque PID parameters
POSITION_P = 10.0            # Position PID proportional gain
POSITION_I = 2.0             # Position PID integral time [s]
POSITION_D = 0.0005          # Position PID derivative time (0 = off)
POSITION_UMIN = -1.2*0.2         # Minimum speed setpoint [m/s]
POSITION_UMAX = 1.2*0.2          # Maximum speed setpoint [m/s]
SPEED_P = 14                 # Speed PID proportional gain
SPEED_I = 0.08               # Speed PID integral time [s]
SPEED_D = 0.0                # Speed PID derivative time (0 = off)
SPEED_TAU = 0.16             # Low-pass filter time constant for speed [s]
TORQUE_UMIN = -40.0          # Minimum torque output [Nm]
TORQUE_UMAX = 40.0           # Maximum torque output [Nm]

PLOT = true                   # Show plots
# =========================================================== #

dt = DT
steps = Int(round(SIM_TIME / dt))

@info "Creating ram air kite model..."
set_data_path(ram_air_data_path())
set = Settings("system.yaml")
set.physical_model = PHYSICAL_MODEL
set.v_wind = V_WIND
set.upwind_dir = UPWIND_DIR
set.profile_law = 3
set.l_tether = TETHER_LENGTH
set.sample_freq = Int(round(1 / dt))

# 1. system structure
sys_struct = create_sys_struct(set)

# 2. model
sam = SymbolicAWEModel(set, sys_struct)

# edit sys_struct before init!
sys_struct.transforms[1].elevation = deg2rad(ELEVATION)
sys_struct.winches[:power_winch].brake = true

for point in sam.sys_struct.points
    point.body_frame_damping .= 0.0
end
for segment in sam.sys_struct.segments
    segment.compression_frac = 0.01
end
for group in sam.sys_struct.groups
    group.moment_frac = 0.0
end

depower = 0.0
sys_struct.tethers[:steering_left].init_stretch_frac = 1.0 - depower
sys_struct.tethers[:steering_right].init_stretch_frac = 1.0 - depower

# 3. init
@info "Initializing model..."
init!(sam; remake=false)

find_steady_state!(sam; dt=0.05, vsm_interval=0)

# Logger setup
logger = Logger(sam, steps + 1)
sys_state = SysState(sam)
sys_state.time = 0.0
steady_torque = calc_steady_torque(sam)
torque_damp = 0.9

for group in sam.sys_struct.groups
    group.damping = 200.0
end

# Setup cascaded position→speed→torque PIDs
pos_pid = DiscretePID(; K=POSITION_P, Ti=POSITION_I, Td=POSITION_D, Ts=dt,
                       umin=POSITION_UMIN, umax=POSITION_UMAX)
speed_pid = DiscretePID(; K=SPEED_P, Ti=SPEED_I, Td=SPEED_D, Ts=dt,
                         umin=TORQUE_UMIN, umax=TORQUE_UMAX)

alpha = dt / (dt + SPEED_TAU)  # low-pass filter coefficient

# ==================== SIMULATION ==================== #

function simulate(sam, logger, steps; plot=false)
    OFFSET = OFFSET_DEG
    v_reelout_diff_filt = Ref(0.0)  # low-pass filtered reelout speed diff

    heading = 0.0
    # Previous sys_state heading for rate calculation
    prev_sys_heading = 0.0
    seq_idx = 1
    # Start with zero steering; sequence activates at t >= 10
    steering_setpoint = INITIAL_STEERING
    steering_active = false

    for i in 1:steps
        t = i * dt - dt

        # After 10 seconds, start steering
        if !steering_active && t >= 2.0
            steering_active = true
            steering_setpoint = abs(STEERING_SEQ[seq_idx])
        end

        if steering_active
            # heading is in [-π, π], 0 = pointing toward ground station
            last_heading = heading
            heading = sam.sys_struct.wings[1].heading
            if heading > π
                heading -= 2π
            end

            if seq_idx <= length(STEERING_SEQ) && rad2deg(heading) < -OFFSET
                steering_setpoint = abs(STEERING_SEQ[seq_idx])  # steer right
            elseif seq_idx <= length(STEERING_SEQ) && rad2deg(heading) > OFFSET
                steering_setpoint = -abs(STEERING_SEQ[seq_idx]) # steer left
                if rad2deg(last_heading) <= OFFSET   # just crossed 0°
                    seq_idx += 1
                    if seq_idx <= length(STEERING_SEQ)
                        @info "Advanced to seq_idx=$seq_idx, setpoint=$(STEERING_SEQ[seq_idx]) m at t=$t s"
                    else
                        @info "Finished all sequence values at t=$t s"
                    end
                end
            end
            # else: heading within OFFSET band — hold current steering_setpoint
        end

        # --- Cascaded position→speed→torque ---
        local l_diff = sys_state.l_tether[3] - sys_state.l_tether[4]
        v_reelout_diff = sys_state.v_reelout[2] - sys_state.v_reelout[3]
        v_reelout_diff_filt[] = alpha * v_reelout_diff + (1 - alpha) * v_reelout_diff_filt[]
        speed_setpoint = pos_pid(steering_setpoint, l_diff, 0.0)
        torque = speed_pid(speed_setpoint, v_reelout_diff_filt[], 0.0)

        set_values = [0.0, torque, -torque]

        # --- Steady-state torque compensation ---
        global steady_torque = torque_damp * steady_torque +
                               (1 - torque_damp) * calc_steady_torque(sam)
        set_torques = steady_torque .+ set_values

        next_step!(sam; set_values=set_torques, dt, vsm_interval=VSM_INTERVAL)

        update_sys_state!(sys_state, sam)
        sys_state.time = t

        # Store heading rate [deg/s] in var_15 for analysis
        sys_state.var_15 = rad2deg(sys_state.heading - prev_sys_heading) / dt
        prev_sys_heading = sys_state.heading

        # Store actual line length difference (not setpoint) in var_01 for analysis
        sys_state.var_01 = l_diff
        # Store steering setpoint in var_02 for analysis
        sys_state.var_02 = steering_setpoint

        log!(logger, sys_state)

        # if plot && mod(i, 5) == 1
        #     @printf "t=%.1f  heading=%.1f°  dl_setpoint=%.3f m  torque=%.1f N·m\n" t rad2deg(sam.sys_struct.wings[1].heading) steering_setpoint applied_torque
        # end

        if mod(i, 20) == 0
            @info "step $i / $steps, steering_setpoint=$steering_setpoint m"
        end
    end
end

@info "Running simulation..."
simulate(sam, logger, steps; plot=true)
@info "Simulation finished"

mkpath(get_data_path())
save_log(logger, "tmp_run")

# ==================== ANALYSIS ==================== #

# wrap2pi: wrap angle to [-π, π]
function wrap2pi(theta)
    theta = theta % (2π)
    if theta > π
        theta -= 2π
    elseif theta < -π
        theta += 2π
    end
    return theta
end

function crosscor_simple(x, y, max_lag)
    n = length(x)
    x = x .- mean(x)
    y = y .- mean(y)
    result = zeros(2 * max_lag + 1)
    for lag in -max_lag:max_lag
        s = 0.0
        c = 0
        for i in 1:n
            j = i + lag
            if 1 <= j <= n
                s += x[i] * y[j]
                c += 1
            end
        end
        result[lag + max_lag + 1] = c > 0 ? s / c : 0.0
    end
    return result
end

function delay(x, y, t_max=10)
    @assert length(x) == length(y)
    overlap = round(Int, t_max / dt)
    z = crosscor_simple(x, y, overlap - 1)
    delay_ = argmax(z)
    delay_ -= overlap
    return delay_ - 1
end

# Shift vector by `shift` positions to the right
function shift_vector(vec, shift)
    shift *= -1
    if shift > 0
        return [vec[1+shift:end]; zeros(shift)]
    elseif shift < 0
        return [zeros(-shift); vec[1:end+shift]]
    else
        return vec
    end
end

function plot_steering_vs_turn_rate()
    lg = load_log("tmp_run")
    sl = lg.syslog
    psi = rad2deg.(wrap2pi.(sl.heading))
    psi_dot = sl.var_15  # deg/s
    var_01 = sl.var_01  # actual line length difference [m] (not setpoint)

    delta = delay(var_01, psi_dot ./ sl.v_app)
    println("Delay of turnrate: $(round(delta * dt, digits=3)) s")
    delayed_steering = shift_vector(var_01, delta)

    G = psi_dot ./ sl.v_app ./ delayed_steering  # °/s / (m/s) / m = °/m
    for (i, _) in enumerate(G)
        if abs(delayed_steering[i]) < 0.05
            G[i] = NaN
        end
    end

    G_mean = mean(filter(!isnan, G))
    G_std = std(filter(!isnan, G))
    println("Mean turnrate-law factor: $(round(G_mean, digits=3)) °/m ± $(round(G_std / G_mean * 100, digits=2)) %")
    println("Mean turnrate-law factor: $(round(deg2rad(G_mean), digits=4)) rad/m ± $(round(G_std / G_mean * 100, digits=2)) %")

    if PLOT
        p1 = plot(sl.time, delayed_steering, sl.var_15 ./ sl.v_app;
                  ylabels=["delayed_steering", "turnrate/v_app [°/m]"],
                  ylims=[(-0.6, 0.6), (-G_mean * 0.6, G_mean * 0.6)],
                  fig="steering vs turnrate")
        p2 = plot(sl.time, G ./ G_mean; ylabel="G/G_mean [-]", fig="turnrate_law")
        display(p1)
        display(p2)
    end

    return sl.time, sl.v_app, deg2rad.(psi), sl.elevation, deg2rad.(psi_dot), delayed_steering
end

function calc_c1_c2(v_app, psi, beta, psi_dot, steering)
    # c1 * v_app * u_s + c2/v_app * sin(psi) * cos(beta) - psi_dot = 0
    # Ac = b where c = [c1, c2]
    col1 = v_app .* steering
    col2 = sin.(psi) .* cos.(beta) ./ v_app
    A = [col1 col2]
    c = A \ psi_dot
    return c[1], c[2]
end

function plot_turnrate_law(c1, c2, time, v_app, psi, beta, psi_dot, steering)
    est_steering = psi_dot ./ (v_app * c1) .- c2 ./ (c1 .* v_app .^ 2) .* sin.(psi) .* cos.(beta)
    if PLOT
        p1 = plot(time, steering, est_steering;
                  ylabels=["delayed_steering", "est_steering"],
                  ylims=[(-0.6, 0.6), (-0.6, 0.6)],
                  fig="steering vs est_steering")
        display(p1)
    end
end

# ==================== RUN ANALYSIS ==================== #

@info "Analyzing results..."
if PLOT
    using MakieControlPlots
end

time, v_app, psi, beta, psi_dot, steering = plot_steering_vs_turn_rate()

c1, c2 = calc_c1_c2(v_app, psi, beta, psi_dot, steering)
println("Turn-rate law coefficients:")
println("  c1 (steering gain): $(round(c1, digits=6))")
println("  c2 (pendulum stability): $(round(c2, digits=6))")

plot_turnrate_law(c1, c2, time, v_app, psi, beta, psi_dot, steering)
lg = load_log("tmp_run")
sl = lg.syslog
steering = sl.var_01
steering_setpoint_logged = sl.var_02
p=plotx(sl.time, rad2deg.(sl.elevation), rad2deg.(sl.azimuth), rad2deg.(sl.heading), steering, steering_setpoint_logged; ylabels=["elevation [°]", "azimuth [°]", "heading [°]", "steering [m]", "setpoint [m]"], fig="elevation and azimuth")
display(p)

@info "Done!"

# 100 m tether
# Delay of turnrate: 0.12 s
# Mean turnrate-law factor: 12.581 °/m ± 97.21 %
# Mean turnrate-law factor: 0.2196 rad/m ± 97.21 %
# Turn-rate law coefficients:
#   c1 (steering gain): 0.25957
#   c2 (pendulum stability): 38.044747

# 50 m tether
# Delay of turnrate: 0.48 s
# Mean turnrate-law factor: 45.013 °/m ± 27.92 %
# Mean turnrate-law factor: 0.7856 rad/m ± 27.92 %
# Turn-rate law coefficients:
#   c1 (steering gain): 0.7841
#   c2 (pendulum stability): 8.551764