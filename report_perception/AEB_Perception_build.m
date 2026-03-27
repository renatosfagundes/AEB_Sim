%% AEB_Perception_build.m  — v3  (Physics-correct sensor models)
%
% Generates AEB_Perception.slx programmatically (Simulink R2025b).
%
% ── Sensor physics ────────────────────────────────────────────────────────
%
%  RADAR (FMCW):
%    • Measures RANGE via beat frequency (time-of-flight of modulated wave)
%    • Measures RADIAL VELOCITY via Doppler frequency shift — DIRECTLY
%    • Does NOT measure ego speed
%    • Noise: σ_d ≈ 0.30 m,  σ_vrel ≈ 0.10 m/s
%    • Update rate: 20 ms,  latency: 15 ms
%    Ref: Bosch LRR4, Continental ARS540
%
%  LIDAR (ToF — pulsed laser):
%    • Measures RANGE only via time-of-flight — NO Doppler capability
%    • Relative velocity is DERIVED from consecutive distance readings:
%        v_lidar(k) ≈ [d(k) - d(k-1)] / Δt  →  much noisier than radar
%    • Noise: σ_d ≈ 0.05 m (3× more precise than radar for range)
%    • Update rate: 50 ms,  latency: 10 ms
%    Ref: Velodyne HDL-32E, Hesai Pandar40
%
%  WHEEL/IMU:
%    • Ego vehicle speed from wheel-speed sensors + IMU integration
%    • NOT provided by radar or LiDAR
%    • Noise: σ_ego ≈ 0.05 m/s
%    • Update rate: 10 ms (high-frequency loop)
%    Ref: SAE J2490, Continental MK C1 ESC
%
% ── Kalman Filter ────────────────────────────────────────────────────────
%
%  State:  x = [distance; rel_speed]   (2 states, sufficient for TTC)
%  Process:  F = [1, -DT; 0, 1]        (constant-relative-velocity model)
%  Sequential measurement updates:
%    1) LiDAR update  H = [1, 0]         → corrects distance only
%    2) Radar update  H = eye(2)         → corrects distance + rel_speed
%  Ego speed: pass-through from Wheel/IMU (not in KF state)
%
% ── Degraded-mode fallback ───────────────────────────────────────────────
%  Radar OK + LiDAR OK  → both sequential updates,  confidence = 1.0
%  Radar only           → radar update only,         confidence = 0.7
%  LiDAR only           → range-only update,         confidence = 0.5
%  Both failed          → predict-only,              confidence = 0.0
%
% ── Ref ──────────────────────────────────────────────────────────────────
%  MathWorks AEB with Sensor Fusion example (Automated Driving Toolbox)
%  MDPI Sensors 2023 — Multi-Sensor Fusion Survey
%  FMCW Radar: wirelesspi.com
%
% Usage:
%   AEB_Perception_build
%   AEB_Perception_scenarios('fault_radar');
%   out = sim('AEB_Perception');
%   AEB_Perception_plot(out);

clear; clc;

%% =========================================================================
%% 1. Parameters
%% =========================================================================
SIM_DT        = 0.001;
SIM_STOP_TIME = 15;

% --- Radar (FMCW) ---
RADAR_RANGE_NOISE  = 0.30;   % [m]   Range noise  σ — beat frequency uncertainty
RADAR_VEL_NOISE    = 0.10;   % [m/s] Radial-velocity noise σ — Doppler resolution
RADAR_UPDATE_RATE  = 0.020;  % [s]   20 ms
RADAR_LATENCY      = 0.015;  % [s]   15 ms (DSP + CAN stack)
RADAR_RANGE_MIN    = 0.5;    % [m]
RADAR_RANGE_MAX    = 200.0;  % [m]
RADAR_VREL_MAX     = 50.0;   % [m/s]

% --- LiDAR (ToF pulsed) ---
LIDAR_RANGE_NOISE  = 0.05;   % [m]   Range noise σ — more precise than radar
LIDAR_UPDATE_RATE  = 0.050;  % [s]   50 ms (20 Hz scan)
LIDAR_LATENCY      = 0.010;  % [s]   10 ms (point-cloud processing)
LIDAR_RANGE_MIN    = 1.0;    % [m]
LIDAR_RANGE_MAX    = 100.0;  % [m]

% --- Wheel/IMU ego-speed sensor ---
WHEEL_SPEED_NOISE  = 0.05;   % [m/s] Wheel sensor + IMU noise σ
WHEEL_UPDATE_RATE  = 0.010;  % [s]   10 ms (100 Hz)
WHEEL_LATENCY      = 0.005;  % [s]    5 ms

% --- Fault detection (mirrors aeb_perception.c) ---
SENSOR_FAULT_CYCLES = 3;
DISTANCE_ROC_MAX    = 10.0;  % [m]   per cycle
VREL_ROC_MAX        = 2.0;   % [m/s] per cycle

% --- Default scenario ---
SCENARIO_TRUE_DISTANCE   = 80.0;
SCENARIO_TRUE_REL_SPEED  = 13.89;
SCENARIO_TRUE_EGO_SPEED  = 13.89;
SCENARIO_FAULT_INJECTION = 0;
SCENARIO_FAULT_START     = 5.0;
SCENARIO_FAULT_DURATION  = 1.5;

fprintf('=== AEB Perception Build v3 (Physics-Correct Sensor Models) ===\n');
fprintf('  Radar : range + Doppler velocity  (direct measurement)\n');
fprintf('  LiDAR : range only                (velocity derived by diff)\n');
fprintf('  Wheel/IMU: ego speed              (wheel sensor + IMU)\n');
fprintf('  Fusion: 2-state Kalman Filter  x = [distance; rel_speed]\n\n');

%% =========================================================================
%% 2. Create model
%% =========================================================================
mdl = 'AEB_Perception';
script_dir = fileparts(mfilename('fullpath'));

if bdIsLoaded(mdl), close_system(mdl, 0); end
slx_path = fullfile(script_dir, [mdl '.slx']);
if exist(slx_path, 'file'), delete(slx_path); end

new_system(mdl);
open_system(mdl);

set_param(mdl, ...
    'SolverType',        'Fixed-step', ...
    'Solver',            'ode4', ...
    'FixedStep',         num2str(SIM_DT), ...
    'StopTime',          num2str(SIM_STOP_TIME), ...
    'SignalLogging',     'on', ...
    'SignalLoggingName', 'logsout');

%% =========================================================================
%% 3. Input sources  (ground truth from plant / test harness)
%% =========================================================================
add_block('simulink/Sources/From Workspace',[mdl '/TrueDistance_src'], ...
    'VariableName','ts_true_distance',   'Position',[60  60 200  80]);
add_block('simulink/Sources/From Workspace',[mdl '/TrueRelSpeed_src'], ...
    'VariableName','ts_true_rel_speed',  'Position',[60 130 200 150]);
add_block('simulink/Sources/From Workspace',[mdl '/TrueEgoSpeed_src'], ...
    'VariableName','ts_true_ego_speed',  'Position',[60 200 200 220]);
add_block('simulink/Sources/From Workspace',[mdl '/FaultInj_src'], ...
    'VariableName','ts_fault_injection', 'Position',[60 270 200 290]);

%% =========================================================================
%% 4. RADAR subsystem
%%    Inputs:  d_true [m],  vrel_true [m/s]
%%    Outputs: d_r [m],     vr_r [m/s]   (range + Doppler velocity)
%%    Chain:   UnitDelay(15ms) → Add(noise) → ZOH(20ms) → Saturation
%% =========================================================================
build_radar_sub(mdl, ...
    RADAR_RANGE_NOISE, RADAR_VEL_NOISE, ...
    RADAR_UPDATE_RATE, RADAR_LATENCY, ...
    RADAR_RANGE_MIN, RADAR_RANGE_MAX, RADAR_VREL_MAX);

%% =========================================================================
%% 5. LIDAR subsystem
%%    Input:   d_true [m]
%%    Output:  d_l [m]   (range only — NO velocity output)
%%    Note: velocity is derived inside KalmanFusion via finite difference
%%    Chain:   UnitDelay(10ms) → Add(noise) → ZOH(50ms) → Saturation
%% =========================================================================
build_lidar_sub(mdl, ...
    LIDAR_RANGE_NOISE, LIDAR_UPDATE_RATE, LIDAR_LATENCY, ...
    LIDAR_RANGE_MIN, LIDAR_RANGE_MAX);

%% =========================================================================
%% 6. WHEEL/IMU subsystem
%%    Input:   ve_true [m/s]
%%    Output:  ve_ego [m/s]   (wheel speed + IMU, small noise)
%%    Chain:   UnitDelay(5ms) → Add(noise) → ZOH(10ms) → Saturation
%% =========================================================================
build_wheelimu_sub(mdl, WHEEL_SPEED_NOISE, WHEEL_UPDATE_RATE, WHEEL_LATENCY);

%% =========================================================================
%% 7. Per-sensor fault detection (MATLAB Function blocks)
%%
%% RADAR fault checks: range  [0.5, 200] m  AND
%%                     radial-velocity |vr| ≤ 50 m/s  AND
%%                     rate-of-change: |Δd| ≤ 10 m/cycle, |Δvr| ≤ 2 m/s/cycle
%%
%% LIDAR fault checks: range  [1.0, 100] m  AND
%%                     rate-of-change: |Δd| ≤ 10 m/cycle
%%                     (no velocity to check — range-only sensor)
%% =========================================================================

% ---- Radar fault (2-input: d_r, vr_r) ----
radar_fault_code = strjoin({
  'function radar_fault = radar_fault_detect(d_r, vr_r)'
  '% Fault detection for FMCW radar (range + radial velocity).'
  '% Range plausibility, velocity plausibility, and rate-of-change checks.'
  '% Latches after 3 consecutive bad cycles (SENSOR_FAULT_CYCLES = 3).'
  'persistent prev_d prev_vr ctr is_first;'
  'FAULT_CYCLES = 3; DIST_ROC = 10.0; VEL_ROC = 2.0;'
  'if isempty(is_first)'
  '    is_first = true; prev_d = d_r; prev_vr = vr_r; ctr = 0;'
  'end'
  'bad = (d_r  < 0.5)  || (d_r  > 200.0) || (abs(vr_r) > 50.0);'
  'if ~is_first'
  '    bad = bad || (abs(d_r - prev_d) > DIST_ROC) || (abs(vr_r - prev_vr) > VEL_ROC);'
  'end'
  'if bad, ctr = ctr+1; else, ctr = 0; end'
  'radar_fault = double(ctr >= FAULT_CYCLES);'
  'prev_d = d_r; prev_vr = vr_r; is_first = false;'
  'end'
}, newline);

% ---- LiDAR fault (1-input: d_l) ----
lidar_fault_code = strjoin({
  'function lidar_fault = lidar_fault_detect(d_l)'
  '% Fault detection for ToF LiDAR (range only).'
  '% Range plausibility and rate-of-change check (no velocity to validate).'
  '% Latches after 3 consecutive bad cycles.'
  'persistent prev_d ctr is_first;'
  'FAULT_CYCLES = 3; DIST_ROC = 10.0;'
  'if isempty(is_first)'
  '    is_first = true; prev_d = d_l; ctr = 0;'
  'end'
  'bad = (d_l < 1.0) || (d_l > 100.0);'
  'if ~is_first'
  '    bad = bad || (abs(d_l - prev_d) > DIST_ROC);'
  'end'
  'if bad, ctr = ctr+1; else, ctr = 0; end'
  'lidar_fault = double(ctr >= FAULT_CYCLES);'
  'prev_d = d_l; is_first = false;'
  'end'
}, newline);

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/RadarFault'], 'Position',[620  60 800 130]);
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/LidarFault'], 'Position',[620 200 800 270]);

sf_root = sfroot();
sf_root.find('-isa','Stateflow.EMChart','Path',[mdl '/RadarFault']).Script = radar_fault_code;
sf_root.find('-isa','Stateflow.EMChart','Path',[mdl '/LidarFault']).Script = lidar_fault_code;

set_param([mdl '/RadarFault'],'BackgroundColor','[1.0 0.88 0.75]','ForegroundColor','[0.6 0.1 0]');
set_param([mdl '/LidarFault'],'BackgroundColor','[0.88 1.0 0.88]','ForegroundColor','[0 0.4 0]');

%% =========================================================================
%% 8. KALMAN FILTER FUSION (MATLAB Function block)
%%
%% State x = [distance; rel_speed]  — 2 states, sufficient for TTC
%%
%% Process model (constant-relative-velocity):
%%   x(k+1) = F*x(k) + w
%%   F = [1, -DT;    distance decreases at rel_speed
%%        0,  1  ]   rel_speed assumed constant between frames
%%
%% Measurement models:
%%   Radar  : H_r = eye(2)   →  z_r = [d_r; vr_r]  (range + Doppler)
%%   LiDAR  : H_l = [1, 0]  →  z_l = [d_l]         (range only)
%%
%% Sequential update (LiDAR first → Radar second) when both available.
%% Velocity from LiDAR is NOT used (derived, too noisy for KF update).
%% Ego speed is a PASS-THROUGH from Wheel/IMU (not in KF state).
%%
%% Inputs  (6): d_r, vr_r, d_l, ve_ego, r_fault, l_fault, fi
%% Outputs (5): d_fused, vr_fused, ve_fused, confidence, sensor_fault
%% =========================================================================
kf_code = strjoin({
  'function [d_out, vr_out, ve_out, conf_out, fault_out] = kalman_fusion(d_r, vr_r, d_l, ve_ego, r_fault, l_fault, fi)'
  '% Discrete Kalman Filter for AEB 1D sensor fusion.'
  '%'
  '% Sensor physics:'
  '%   Radar  — measures range AND radial velocity (Doppler). Both states.'
  '%   LiDAR  — measures range ONLY (ToF). Velocity NOT measured directly.'
  '%   WheelIMU — measures ego speed. Passed through, not in KF state.'
  '%'
  '% State x = [distance; rel_speed]'
  '% Process F = [1, -DT; 0, 1]  (constant-relative-velocity model)'
  '%'
  '% Sequential updates when both sensors active:'
  '%   1) LiDAR: H=[1,0], R=sigma_l^2  — corrects range estimate'
  '%   2) Radar: H=eye(2), R=diag([sd^2, sv^2])  — corrects range+velocity'
  'persistent x P;'
  'DT = 0.001;'
  'if isempty(x)'
  '    x = [d_r; vr_r];'
  '    P = diag([1.0, 0.25]);'
  'end'
  'F = [1, -DT; 0, 1];'
  'Q = diag([1e-4, 1e-5]);'
  '% ---- Predict ----'
  'xp = F * x;'
  'Pp = F * P * F'' + Q;'
  '% ---- Sensor availability ----'
  'r_ok = (r_fault < 0.5) && (fi < 0.5);'
  'l_ok = (l_fault < 0.5) && (fi < 0.5);'
  'fault_out = double(~r_ok && ~l_ok);'
  'if ~r_ok && ~l_ok'
  '    % Both failed: predict-only, hold last valid state'
  '    x = xp; P = Pp;'
  '    conf_out = 0.0;'
  'else'
  '    x_u = xp; P_u = Pp;'
  '    % ---- Sequential update 1: LiDAR (range only) ----'
  '    % LiDAR measures only range. Velocity is NOT provided by ToF LiDAR.'
  '    if l_ok'
  '        H_l = [1, 0];'
  '        R_l = 0.0025;'
  '        S_l = H_l * P_u * H_l'' + R_l;'
  '        K_l = P_u * H_l'' / S_l;'
  '        x_u = x_u + K_l * (d_l - H_l * x_u);'
  '        P_u = (eye(2) - K_l * H_l) * P_u;'
  '    end'
  '    % ---- Sequential update 2: Radar (range + Doppler velocity) ----'
  '    % Radar measures BOTH range and radial velocity via Doppler shift.'
  '    if r_ok'
  '        H_r = eye(2);'
  '        R_r = diag([0.09, 0.01]);'
  '        S_r = H_r * P_u * H_r'' + R_r;'
  '        K_r = P_u * H_r'' / S_r;'
  '        x_u = x_u + K_r * ([d_r; vr_r] - H_r * x_u);'
  '        P_u = (eye(2) - K_r * H_r) * P_u;'
  '    end'
  '    x = x_u; P = P_u;'
  '    if r_ok && l_ok'
  '        conf_out = 1.0;'
  '    elseif r_ok'
  '        conf_out = 0.7;'
  '    else'
  '        conf_out = 0.5;'
  '    end'
  '    conf_out = conf_out * max(0.0, 1.0 - trace(P) / 2.0);'
  'end'
  '% Physical limits'
  'x(1) = max(0.5,  min(300.0, x(1)));'
  'x(2) = max(-50.0, min(50.0, x(2)));'
  'd_out  = x(1);'
  'vr_out = x(2);'
  've_out = ve_ego;'
  'end'
}, newline);

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/KalmanFusion'], 'Position',[880  60 1080 360]);
sf_root.find('-isa','Stateflow.EMChart','Path',[mdl '/KalmanFusion']).Script = kf_code;
set_param([mdl '/KalmanFusion'],'BackgroundColor','[1.0 0.97 0.75]','ForegroundColor','[0.4 0.3 0]');

%% =========================================================================
%% 9. Output logging (To Workspace + Scopes)
%% =========================================================================
out_x = 1160;
tw = {'d_meas_log',  out_x  80; 'vr_meas_log', out_x 120; 've_meas_log', out_x 160;
      'conf_log',    out_x 200; 'fault_log',   out_x 240; 'd_true_log',  out_x 280};
for k = 1:size(tw,1)
    add_block('simulink/Sinks/To Workspace',[mdl '/' tw{k,1}], ...
        'VariableName',tw{k,1},'SaveFormat','Timeseries', ...
        'Position',[tw{k,2} tw{k,3} tw{k,2}+150 tw{k,3}+20]);
end

add_block('simulink/Sinks/Scope',[mdl '/Scope_Distance'], ...
    'NumInputPorts','3','Position',[out_x 360 out_x+80 410]);
add_block('simulink/Sinks/Scope',[mdl '/Scope_Fault'], ...
    'NumInputPorts','2','Position',[out_x 430 out_x+80 480]);

%% =========================================================================
%% 10. Top-level connections
%% =========================================================================

% Sources → Radar (d_true, vrel_true)
add_line(mdl,'TrueDistance_src/1','Radar/1','autorouting','on');
add_line(mdl,'TrueRelSpeed_src/1','Radar/2','autorouting','on');

% Sources → LiDAR (d_true only — range-only sensor)
add_line(mdl,'TrueDistance_src/1','Lidar/1','autorouting','on');

% Sources → WheelIMU (ego speed — NOT from radar/lidar)
add_line(mdl,'TrueEgoSpeed_src/1','WheelIMU/1','autorouting','on');

% Radar → RadarFault (d_r, vr_r)
add_line(mdl,'Radar/1','RadarFault/1','autorouting','on');
add_line(mdl,'Radar/2','RadarFault/2','autorouting','on');

% LiDAR → LidarFault (d_l only)
add_line(mdl,'Lidar/1','LidarFault/1','autorouting','on');

% → KalmanFusion ports:
%   1=d_r  2=vr_r  3=d_l  4=ve_ego  5=r_fault  6=l_fault  7=fi
add_line(mdl,'Radar/1',       'KalmanFusion/1','autorouting','on');
add_line(mdl,'Radar/2',       'KalmanFusion/2','autorouting','on');
add_line(mdl,'Lidar/1',       'KalmanFusion/3','autorouting','on');
add_line(mdl,'WheelIMU/1',    'KalmanFusion/4','autorouting','on');
add_line(mdl,'RadarFault/1',  'KalmanFusion/5','autorouting','on');
add_line(mdl,'LidarFault/1',  'KalmanFusion/6','autorouting','on');
add_line(mdl,'FaultInj_src/1','KalmanFusion/7','autorouting','on');

% KalmanFusion → outputs (1=d 2=vr 3=ve 4=conf 5=fault)
add_line(mdl,'KalmanFusion/1','d_meas_log/1', 'autorouting','on');
add_line(mdl,'KalmanFusion/2','vr_meas_log/1','autorouting','on');
add_line(mdl,'KalmanFusion/3','ve_meas_log/1','autorouting','on');
add_line(mdl,'KalmanFusion/4','conf_log/1',   'autorouting','on');
add_line(mdl,'KalmanFusion/5','fault_log/1',  'autorouting','on');

add_line(mdl,'KalmanFusion/1',     'Scope_Distance/1','autorouting','on');
add_line(mdl,'Radar/1',            'Scope_Distance/2','autorouting','on');
add_line(mdl,'TrueDistance_src/1', 'Scope_Distance/3','autorouting','on');
add_line(mdl,'KalmanFusion/5',     'Scope_Fault/1',   'autorouting','on');
add_line(mdl,'KalmanFusion/4',     'Scope_Fault/2',   'autorouting','on');

add_line(mdl,'TrueDistance_src/1','d_true_log/1','autorouting','on');

%% =========================================================================
%% 11. Default scenario timeseries
%% =========================================================================
t = (0:SIM_DT:SIM_STOP_TIME)';
d_true = max(0, SCENARIO_TRUE_DISTANCE - SCENARIO_TRUE_REL_SPEED * t);
fi     = double(t >= SCENARIO_FAULT_START & ...
                t <  SCENARIO_FAULT_START + SCENARIO_FAULT_DURATION) * SCENARIO_FAULT_INJECTION;

assignin('base','ts_true_distance',   timeseries(d_true, t));
assignin('base','ts_true_rel_speed',  timeseries(SCENARIO_TRUE_REL_SPEED * ones(size(t)), t));
assignin('base','ts_true_ego_speed',  timeseries(SCENARIO_TRUE_EGO_SPEED * ones(size(t)), t));
assignin('base','ts_fault_injection', timeseries(fi, t));

%% =========================================================================
%% 12. Auto-layout and line routing
%% =========================================================================
fprintf('Applying auto-layout...\n');
Simulink.BlockDiagram.arrangeSystem([mdl '/Radar'],    'FullLayout','on');
Simulink.BlockDiagram.arrangeSystem([mdl '/Lidar'],    'FullLayout','on');
Simulink.BlockDiagram.arrangeSystem([mdl '/WheelIMU'], 'FullLayout','on');
Simulink.BlockDiagram.arrangeSystem(mdl, 'FullLayout', 'on');
all_lines = find_system(mdl, 'FindAll','on','Type','line');
Simulink.BlockDiagram.routeLine(all_lines);

%% =========================================================================
%% 13. Save
%% =========================================================================
save_system(mdl, slx_path);
fprintf('Model saved: %s\n', slx_path);
fprintf('\nSensor outputs summary:\n');
fprintf('  Radar    → d_r [m] + vr_r [m/s]  (range + Doppler)\n');
fprintf('  LiDAR    → d_l [m]               (range only)\n');
fprintf('  WheelIMU → ve_ego [m/s]           (not from radar/lidar)\n');
fprintf('  KF state → x = [distance; rel_speed]  (2D)\n');

%% =========================================================================
%% Local functions — must appear at end of script
%% =========================================================================

function build_radar_sub(mdl, noise_d, noise_v, rate, latency, rmin, rmax, vrmax)
%BUILD_RADAR_SUB  FMCW Radar: measures range AND radial velocity
%  Inputs:  d_true [m],  vrel_true [m/s]
%  Outputs: d_r [m],     vr_r [m/s]
    p = [mdl '/Radar'];
    add_block('simulink/Ports & Subsystems/Subsystem', p, 'Position',[300 60 480 190]);
    delete_line(p,'In1/1','Out1/1'); delete_block([p '/In1']); delete_block([p '/Out1']);

    add_block('simulink/Ports & Subsystems/In1',[p '/d_in'],  'Port','1','Position',[20 30  50 46]);
    add_block('simulink/Ports & Subsystems/In1',[p '/vr_in'], 'Port','2','Position',[20 110 50 126]);

    add_block('simulink/Discrete/Unit Delay',[p '/Dly_d'], 'SampleTime',num2str(latency),'Position',[90  25 150  55]);
    add_block('simulink/Discrete/Unit Delay',[p '/Dly_vr'],'SampleTime',num2str(latency),'Position',[90 105 150 135]);

    add_block('simulink/Sources/Band-Limited White Noise',[p '/N_d'], ...
        'Cov',num2str(noise_d^2),'Ts',num2str(rate),'seed','12345','Position',[90 200 150 230]);
    add_block('simulink/Sources/Band-Limited White Noise',[p '/N_vr'], ...
        'Cov',num2str(noise_v^2),'Ts',num2str(rate),'seed','12346','Position',[90 245 150 275]);

    add_block('simulink/Math Operations/Add',[p '/Add_d'], 'Inputs','++','Position',[200  25 240  55]);
    add_block('simulink/Math Operations/Add',[p '/Add_vr'],'Inputs','++','Position',[200 105 240 135]);

    add_block('simulink/Discrete/Zero-Order Hold',[p '/ZOH_d'], 'SampleTime',num2str(rate),'Position',[270  25 330  55]);
    add_block('simulink/Discrete/Zero-Order Hold',[p '/ZOH_vr'],'SampleTime',num2str(rate),'Position',[270 105 330 135]);

    add_block('simulink/Discontinuities/Saturation',[p '/Sat_d'], ...
        'UpperLimit',num2str(rmax),'LowerLimit',num2str(rmin),'Position',[360  25 420  55]);
    add_block('simulink/Discontinuities/Saturation',[p '/Sat_vr'], ...
        'UpperLimit',num2str(vrmax),'LowerLimit',num2str(-vrmax),'Position',[360 105 420 135]);

    add_block('simulink/Ports & Subsystems/Out1',[p '/d_r'], 'Port','1','Position',[460  30 490  46]);
    add_block('simulink/Ports & Subsystems/Out1',[p '/vr_r'],'Port','2','Position',[460 110 490 126]);

    add_line(p,'d_in/1','Dly_d/1','autorouting','on');
    add_line(p,'vr_in/1','Dly_vr/1','autorouting','on');
    add_line(p,'Dly_d/1','Add_d/1','autorouting','on');
    add_line(p,'Dly_vr/1','Add_vr/1','autorouting','on');
    add_line(p,'N_d/1','Add_d/2','autorouting','on');
    add_line(p,'N_vr/1','Add_vr/2','autorouting','on');
    add_line(p,'Add_d/1','ZOH_d/1','autorouting','on');
    add_line(p,'Add_vr/1','ZOH_vr/1','autorouting','on');
    add_line(p,'ZOH_d/1','Sat_d/1','autorouting','on');
    add_line(p,'ZOH_vr/1','Sat_vr/1','autorouting','on');
    add_line(p,'Sat_d/1','d_r/1','autorouting','on');
    add_line(p,'Sat_vr/1','vr_r/1','autorouting','on');

    set_param(p,'BackgroundColor','[0.75 0.87 1.0]','ForegroundColor','[0 0 0.5]');
    % Arrange subsystem internal layout
    Simulink.BlockDiagram.arrangeSystem(p, 'FullLayout', 'on');
end

function build_lidar_sub(mdl, noise_d, rate, latency, rmin, rmax)
%BUILD_LIDAR_SUB  ToF LiDAR: measures range ONLY (no velocity output)
%  Input:  d_true [m]
%  Output: d_l [m]
    p = [mdl '/Lidar'];
    add_block('simulink/Ports & Subsystems/Subsystem', p, 'Position',[300 240 480 320]);
    delete_line(p,'In1/1','Out1/1'); delete_block([p '/In1']); delete_block([p '/Out1']);

    add_block('simulink/Ports & Subsystems/In1',[p '/d_in'],'Port','1','Position',[20 60 50 76]);

    add_block('simulink/Discrete/Unit Delay',[p '/Dly_d'],'SampleTime',num2str(latency),'Position',[90 55 150 85]);

    add_block('simulink/Sources/Band-Limited White Noise',[p '/N_d'], ...
        'Cov',num2str(noise_d^2),'Ts',num2str(rate),'seed','23456','Position',[90 130 150 160]);

    add_block('simulink/Math Operations/Add',[p '/Add_d'],'Inputs','++','Position',[200 55 240 85]);

    add_block('simulink/Discrete/Zero-Order Hold',[p '/ZOH_d'],'SampleTime',num2str(rate),'Position',[270 55 330 85]);

    add_block('simulink/Discontinuities/Saturation',[p '/Sat_d'], ...
        'UpperLimit',num2str(rmax),'LowerLimit',num2str(rmin),'Position',[360 55 420 85]);

    add_block('simulink/Ports & Subsystems/Out1',[p '/d_l'],'Port','1','Position',[460 60 490 76]);

    add_line(p,'d_in/1','Dly_d/1','autorouting','on');
    add_line(p,'Dly_d/1','Add_d/1','autorouting','on');
    add_line(p,'N_d/1','Add_d/2','autorouting','on');
    add_line(p,'Add_d/1','ZOH_d/1','autorouting','on');
    add_line(p,'ZOH_d/1','Sat_d/1','autorouting','on');
    add_line(p,'Sat_d/1','d_l/1','autorouting','on');

    set_param(p,'BackgroundColor','[0.75 1.0 0.75]','ForegroundColor','[0 0.4 0]');
    Simulink.BlockDiagram.arrangeSystem(p, 'FullLayout', 'on');
end

function build_wheelimu_sub(mdl, noise_v, rate, latency)
%BUILD_WHEELIMU_SUB  Wheel speed + IMU: ego vehicle speed sensor
%  Ego speed is NOT measured by radar or LiDAR — comes from chassis sensors.
%  Input:  ve_true [m/s]
%  Output: ve_ego [m/s]
    p = [mdl '/WheelIMU'];
    add_block('simulink/Ports & Subsystems/Subsystem', p, 'Position',[300 360 480 430]);
    delete_line(p,'In1/1','Out1/1'); delete_block([p '/In1']); delete_block([p '/Out1']);

    add_block('simulink/Ports & Subsystems/In1',[p '/ve_in'],'Port','1','Position',[20 60 50 76]);

    add_block('simulink/Discrete/Unit Delay',[p '/Dly_ve'],'SampleTime',num2str(latency),'Position',[90 55 150 85]);

    add_block('simulink/Sources/Band-Limited White Noise',[p '/N_ve'], ...
        'Cov',num2str(noise_v^2),'Ts',num2str(rate),'seed','34567','Position',[90 130 150 160]);

    add_block('simulink/Math Operations/Add',[p '/Add_ve'],'Inputs','++','Position',[200 55 240 85]);

    add_block('simulink/Discrete/Zero-Order Hold',[p '/ZOH_ve'],'SampleTime',num2str(rate),'Position',[270 55 330 85]);

    add_block('simulink/Discontinuities/Saturation',[p '/Sat_ve'], ...
        'UpperLimit','50','LowerLimit','0','Position',[360 55 420 85]);

    add_block('simulink/Ports & Subsystems/Out1',[p '/ve_ego'],'Port','1','Position',[460 60 490 76]);

    add_line(p,'ve_in/1','Dly_ve/1','autorouting','on');
    add_line(p,'Dly_ve/1','Add_ve/1','autorouting','on');
    add_line(p,'N_ve/1','Add_ve/2','autorouting','on');
    add_line(p,'Add_ve/1','ZOH_ve/1','autorouting','on');
    add_line(p,'ZOH_ve/1','Sat_ve/1','autorouting','on');
    add_line(p,'Sat_ve/1','ve_ego/1','autorouting','on');

    set_param(p,'BackgroundColor','[0.97 0.90 0.97]','ForegroundColor','[0.4 0 0.4]');
    Simulink.BlockDiagram.arrangeSystem(p, 'FullLayout', 'on');
end
