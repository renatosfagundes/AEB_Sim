%% AEB_Plant_build.m
%
% Builds the AEB Plant (vehicle dynamics) Simulink model programmatically.
% The Plant subsystem models the physical environment: ego vehicle dynamics,
% brake actuator, target vehicle profile, and relative kinematics.
%
% Subsystems:
%   1. Ego Vehicle Dynamics   - longitudinal dynamics with drag and rolling resistance
%   2. Brake Actuator Model   - 1st order + transport delay
%   3. Target Vehicle Profile - programmable speed profile (CCRs, CCRm, CCRb)
%   4. Relative Kinematics    - distance, relative speed, TTC
%
% Inputs (from Controller / Scenario):
%   brake_pressure_cmd [bar]  - brake command from AEB controller (0-10 bar)
%   scenario_select   [enum]  - 1=CCRs, 2=CCRm, 3=CCRb
%
% Outputs (to Perception / CAN):
%   ego_speed     [m/s]   - ego vehicle speed
%   ego_accel     [m/s^2] - ego vehicle acceleration
%   target_speed  [m/s]   - target vehicle speed
%   distance      [m]     - inter-vehicular distance
%   rel_speed     [m/s]   - relative speed (v_ego - v_target)
%   collision     [bool]  - 1 if distance <= 0
%
% Requirements traced:
%   FR-PER-001: Distance acquisition
%   FR-PER-002: Ego speed acquisition
%   FR-PER-003: Relative speed computation
%   FR-DEC-001: TTC computation support
%   FR-DEC-002: Braking distance computation support
%   FR-BRK-008: Brake actuator model (tau=50ms, dead_time=30ms)
%
% Compatible with MATLAB R2025b / Simulink.
%
% Usage:
%   >> AEB_Plant_build        % creates AEB_Plant.slx
%   >> sim('AEB_Plant')       % runs default CCRs 50 km/h scenario

%% ===== Plant Parameters =====
% All parameters are defined in the workspace so they can be tuned without
% modifying the model. See AEB_Plant_Parameters.md for justification.

% --- Simulation ---
SIM_STOP_TIME  = 15;          % Simulation stop time [s]
SIM_DT         = 0.001;       % Fixed-step solver step size [s]

% --- Vehicle parameters ---
VEHICLE_MASS   = 1500;        % Vehicle mass [kg] (Euro NCAP compact sedan)
Cd             = 0.30;        % Aerodynamic drag coefficient [-]
A_FRONTAL      = 2.25;        % Frontal area [m^2]
RHO_AIR        = 1.225;       % Air density [kg/m^3] at sea level, 15 degC
Crr            = 0.012;       % Rolling resistance coefficient [-]
G_ACCEL        = 9.81;        % Gravitational acceleration [m/s^2]
ROAD_GRADE     = 0;           % Road grade [%] (0 = flat, Euro NCAP baseline)

% --- Brake actuator model ---
BRAKE_TAU       = 0.05;       % Pressure build-up time constant [s]
BRAKE_DEAD_TIME = 0.03;       % Hydraulic dead time [s]
BRAKE_MAX_DECEL = 10.0;       % Maximum deceleration [m/s^2]
BRAKE_BAR_MAX   = 10.0;       % Maximum brake pressure [bar]

% --- Scenario definitions ---
% CCRs: ego at 50 km/h (13.89 m/s), target stationary, gap = 100 m
% CCRm: ego at 50 km/h, target at 20 km/h (5.56 m/s), gap = 100 m
% CCRb: ego at 50 km/h, target at 50 km/h initially, decelerates at -2 m/s^2 after 3s, gap = 40 m
SCENARIO_EGO_SPEED     = 13.89;  % Default ego speed [m/s] = 50 km/h
SCENARIO_TARGET_SPEED  = 0.00;   % Default target speed [m/s] = 0 (CCRs)
SCENARIO_INITIAL_GAP   = 100.0;  % Default initial gap [m]
SCENARIO_TARGET_DECEL  = 0.0;    % Default target deceleration [m/s^2]
SCENARIO_BRAKE_TIME    = 99.0;   % Time when target starts braking [s] (99 = never)

%% ===== Model Creation =====
mdl = 'AEB_Plant';

% Close if already open
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

new_system(mdl);
open_system(mdl);

% Set model parameters
set_param(mdl, ...
    'StopTime',        num2str(SIM_STOP_TIME), ...
    'Solver',          'ode4', ...
    'SolverType',      'Fixed-step', ...
    'FixedStep',       num2str(SIM_DT), ...
    'SaveOutput',      'on', ...
    'SaveTime',        'on', ...
    'ReturnWorkspaceOutputs', 'on');

%% ===== Layout constants =====
% Vertical spacing between rows of blocks
X0 = 50;   % left margin
Y0 = 50;   % top margin
BW = 60;   % block width
BH = 30;   % block height
DX = 120;  % horizontal spacing between blocks
DY = 70;   % vertical spacing between blocks

% Helper: position vector
pos = @(col, row) [X0 + (col-1)*DX, Y0 + (row-1)*DY, ...
                   X0 + (col-1)*DX + BW, Y0 + (row-1)*DY + BH];

%% ===== INPUT PORTS =====
add_block('simulink/Sources/In1', [mdl '/brake_cmd_bar']);
set_param([mdl '/brake_cmd_bar'], 'Position', mat2str(pos(1,3)), 'Port', '1');

%% ===== BRAKE ACTUATOR MODEL =====
% brake_cmd_bar -> Transport Delay -> 1st Order TF -> Saturate -> actual_decel
%
% Maps brake pressure (0-10 bar) to deceleration (0-10 m/s^2) via:
%   decel = (brake_pressure / BRAKE_BAR_MAX) * BRAKE_MAX_DECEL
% Then applies actuator dynamics: dead time + 1st order lag.

% Gain: bar -> normalized (0-1)
add_block('simulink/Math Operations/Gain', [mdl '/bar_to_norm']);
set_param([mdl '/bar_to_norm'], ...
    'Position', mat2str(pos(2,3)), ...
    'Gain', num2str(1/BRAKE_BAR_MAX));

% Transport Delay (hydraulic dead time)
add_block('simulink/Continuous/Transport Delay', [mdl '/BrakeDeadTime']);
set_param([mdl '/BrakeDeadTime'], ...
    'Position', mat2str(pos(3,3)), ...
    'DelayTime', num2str(BRAKE_DEAD_TIME));

% Transfer Function: 1/(tau*s + 1)
add_block('simulink/Continuous/Transfer Fcn', [mdl '/BrakeTF']);
set_param([mdl '/BrakeTF'], ...
    'Position', mat2str(pos(4,3)), ...
    'Numerator', '[1]', ...
    'Denominator', ['[' num2str(BRAKE_TAU) ' 1]']);

% Gain: normalized -> deceleration (m/s^2)
add_block('simulink/Math Operations/Gain', [mdl '/norm_to_decel']);
set_param([mdl '/norm_to_decel'], ...
    'Position', mat2str(pos(5,3)), ...
    'Gain', num2str(BRAKE_MAX_DECEL));

% Saturation: 0 to BRAKE_MAX_DECEL
add_block('simulink/Discontinuities/Saturation', [mdl '/BrakeSat']);
set_param([mdl '/BrakeSat'], ...
    'Position', mat2str(pos(6,3)), ...
    'UpperLimit', num2str(BRAKE_MAX_DECEL), ...
    'LowerLimit', '0');

% Connect brake actuator chain
add_line(mdl, 'brake_cmd_bar/1', 'bar_to_norm/1', 'autorouting', 'smart');
add_line(mdl, 'bar_to_norm/1', 'BrakeDeadTime/1', 'autorouting', 'smart');
add_line(mdl, 'BrakeDeadTime/1', 'BrakeTF/1', 'autorouting', 'smart');
add_line(mdl, 'BrakeTF/1', 'norm_to_decel/1', 'autorouting', 'smart');
add_line(mdl, 'norm_to_decel/1', 'BrakeSat/1', 'autorouting', 'smart');

%% ===== EGO VEHICLE DYNAMICS =====
% Net force = -F_brake - F_drag - F_rolling - F_grade
% a_ego = F_net / m
% v_ego = integral(a_ego), clamped >= 0
% x_ego = integral(v_ego)

% --- Aerodynamic drag: F_drag = 0.5 * Cd * A * rho * v^2 ---
% We compute this inside a MATLAB Function block for clarity
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/EgoDynamics']);
set_param([mdl '/EgoDynamics'], 'Position', mat2str([X0+6*DX, Y0+2*DY, X0+6*DX+140, Y0+2*DY+80]));

% Set the MATLAB Function code
% We need to use the Stateflow API to set the function script
% Instead, use a Fcn block for the acceleration calculation
delete_block([mdl '/EgoDynamics']);

% Use a Subsystem for ego dynamics
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/EgoDynamics']);
set_param([mdl '/EgoDynamics'], 'Position', mat2str([X0+7*DX, Y0+1*DY, X0+7*DX+160, Y0+1*DY+100]));

% Delete default connection inside subsystem
delete_line([mdl '/EgoDynamics'], 'In1/1', 'Out1/1');

% Rename ports
set_param([mdl '/EgoDynamics/In1'], 'Name', 'brake_decel');
set_param([mdl '/EgoDynamics/Out1'], 'Name', 'v_ego');

% Add second output port for position
add_block('simulink/Sinks/Out1', [mdl '/EgoDynamics/x_ego']);
set_param([mdl '/EgoDynamics/x_ego'], 'Port', '2', 'Position', mat2str([700, 200, 720, 220]));

% Add third output port for acceleration
add_block('simulink/Sinks/Out1', [mdl '/EgoDynamics/a_ego']);
set_param([mdl '/EgoDynamics/a_ego'], 'Port', '3', 'Position', mat2str([700, 300, 720, 320]));

% --- Inside EgoDynamics subsystem ---
sub = [mdl '/EgoDynamics'];

% Gain: -1 (brake decel is positive, force is negative)
add_block('simulink/Math Operations/Gain', [sub '/neg_brake']);
set_param([sub '/neg_brake'], 'Gain', '-1', 'Position', mat2str([120, 30, 150, 50]));

% Drag force calculation: F_drag/m = 0.5*Cd*A*rho/m * v^2
% This is a gain * v^2
DRAG_COEFF = 0.5 * Cd * A_FRONTAL * RHO_AIR / VEHICLE_MASS;  % [1/m]
add_block('simulink/Math Operations/Math Function', [sub '/v_squared']);
set_param([sub '/v_squared'], 'Operator', 'square', 'Position', mat2str([300, 120, 340, 150]));

add_block('simulink/Math Operations/Gain', [sub '/drag_coeff']);
set_param([sub '/drag_coeff'], 'Gain', num2str(-DRAG_COEFF, '%.6f'), 'Position', mat2str([370, 120, 410, 150]));

% Rolling resistance: F_roll/m = Crr * g
ROLL_DECEL = Crr * G_ACCEL;  % [m/s^2]
add_block('simulink/Sources/Constant', [sub '/roll_decel']);
set_param([sub '/roll_decel'], 'Value', num2str(-ROLL_DECEL, '%.6f'), 'Position', mat2str([300, 180, 340, 200]));

% Grade force: F_grade/m = g * sin(atan(grade/100)) ~ g * grade/100
GRADE_DECEL = G_ACCEL * sin(atan(ROAD_GRADE/100));  % [m/s^2]
add_block('simulink/Sources/Constant', [sub '/grade_decel']);
set_param([sub '/grade_decel'], 'Value', num2str(-GRADE_DECEL, '%.6f'), 'Position', mat2str([300, 240, 340, 260]));

% Sum all forces: a_total = -brake_decel + drag + rolling + grade
add_block('simulink/Math Operations/Sum', [sub '/SumForces']);
set_param([sub '/SumForces'], 'Inputs', '++++', 'Position', mat2str([460, 60, 490, 260]));

% Integrator: a -> v (with saturation at 0, vehicle cannot go backwards)
add_block('simulink/Continuous/Integrator', [sub '/vel_integrator']);
set_param([sub '/vel_integrator'], ...
    'InitialCondition', num2str(SCENARIO_EGO_SPEED), ...
    'LowerSaturationLimit', '0', ...
    'UpperSaturationLimit', 'inf', ...
    'LimitOutput', 'on', ...
    'Position', mat2str([540, 130, 580, 170]));

% Integrator: v -> x
add_block('simulink/Continuous/Integrator', [sub '/pos_integrator']);
set_param([sub '/pos_integrator'], ...
    'InitialCondition', '0', ...
    'Position', mat2str([620, 130, 660, 170]));

% Connect inside subsystem
add_line(sub, 'brake_decel/1', 'neg_brake/1', 'autorouting', 'smart');
add_line(sub, 'neg_brake/1', 'SumForces/1', 'autorouting', 'smart');
add_line(sub, 'v_squared/1', 'drag_coeff/1', 'autorouting', 'smart');
add_line(sub, 'drag_coeff/1', 'SumForces/2', 'autorouting', 'smart');
add_line(sub, 'roll_decel/1', 'SumForces/3', 'autorouting', 'smart');
add_line(sub, 'grade_decel/1', 'SumForces/4', 'autorouting', 'smart');
add_line(sub, 'SumForces/1', 'vel_integrator/1', 'autorouting', 'smart');
add_line(sub, 'vel_integrator/1', 'pos_integrator/1', 'autorouting', 'smart');
add_line(sub, 'vel_integrator/1', 'v_ego/1', 'autorouting', 'smart');
add_line(sub, 'pos_integrator/1', 'x_ego/1', 'autorouting', 'smart');

% Feedback: v_ego -> v_squared (for drag calculation)
add_line(sub, 'vel_integrator/1', 'v_squared/1', 'autorouting', 'smart');

% Acceleration output: sum output directly
add_line(sub, 'SumForces/1', 'a_ego/1', 'autorouting', 'smart');

% Connect brake actuator output to ego dynamics
add_line(mdl, 'BrakeSat/1', 'EgoDynamics/1', 'autorouting', 'smart');

%% ===== TARGET VEHICLE PROFILE =====
% Subsystem that generates target speed and position based on scenario.
% Uses From Workspace blocks for flexibility.

add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/TargetVehicle']);
set_param([mdl '/TargetVehicle'], 'Position', mat2str([X0+7*DX, Y0+5*DY, X0+7*DX+160, Y0+5*DY+60]));

% Delete default connection
delete_line([mdl '/TargetVehicle'], 'In1/1', 'Out1/1');
delete_block([mdl '/TargetVehicle/In1']);

% Rename output
set_param([mdl '/TargetVehicle/Out1'], 'Name', 'v_target');

% Add second output for position
add_block('simulink/Sinks/Out1', [mdl '/TargetVehicle/x_target']);
set_param([mdl '/TargetVehicle/x_target'], 'Port', '2', 'Position', mat2str([500, 150, 520, 170]));

tgt = [mdl '/TargetVehicle'];

% Constant: initial target speed
add_block('simulink/Sources/Constant', [tgt '/v_target_init']);
set_param([tgt '/v_target_init'], ...
    'Value', 'SCENARIO_TARGET_SPEED', ...
    'Position', mat2str([50, 30, 100, 50]));

% Clock for time comparison
add_block('simulink/Sources/Clock', [tgt '/Clock']);
set_param([tgt '/Clock'], 'Position', mat2str([50, 100, 80, 120]));

% Compare: t >= brake_time?
add_block('simulink/Logic and Bit Operations/Compare To Constant', [tgt '/BrakeTimeCheck']);
set_param([tgt '/BrakeTimeCheck'], ...
    'const', 'SCENARIO_BRAKE_TIME', ...
    'relop', '>=', ...
    'Position', mat2str([120, 100, 170, 120]));

% Time since braking: t - brake_time (clamped at 0)
add_block('simulink/Math Operations/Sum', [tgt '/TimeSinceBrake']);
set_param([tgt '/TimeSinceBrake'], 'Inputs', '+-', 'Position', mat2str([120, 160, 150, 190]));

add_block('simulink/Sources/Constant', [tgt '/brake_time_const']);
set_param([tgt '/brake_time_const'], ...
    'Value', 'SCENARIO_BRAKE_TIME', ...
    'Position', mat2str([50, 180, 100, 200]));

add_block('simulink/Discontinuities/Saturation', [tgt '/ClampTime']);
set_param([tgt '/ClampTime'], ...
    'UpperLimit', 'inf', ...
    'LowerLimit', '0', ...
    'Position', mat2str([180, 160, 220, 190]));

% Deceleration contribution: target_decel * time_since_brake
add_block('simulink/Sources/Constant', [tgt '/target_decel_val']);
set_param([tgt '/target_decel_val'], ...
    'Value', 'SCENARIO_TARGET_DECEL', ...
    'Position', mat2str([180, 220, 230, 240]));

add_block('simulink/Math Operations/Product', [tgt '/DecelProduct']);
set_param([tgt '/DecelProduct'], 'Position', mat2str([270, 170, 300, 230]));

% Speed change: v_target = v_init + decel * t_since_brake (decel is negative for braking)
add_block('simulink/Math Operations/Sum', [tgt '/SpeedSum']);
set_param([tgt '/SpeedSum'], 'Inputs', '++', 'Position', mat2str([330, 50, 360, 100]));

% Switch: if braking started, use computed speed; otherwise initial speed
add_block('simulink/Signal Routing/Switch', [tgt '/BrakeSwitch']);
set_param([tgt '/BrakeSwitch'], ...
    'Criteria', 'u2 ~= 0', ...
    'Position', mat2str([330, 30, 360, 100]));

% Clamp target speed >= 0
add_block('simulink/Discontinuities/Saturation', [tgt '/ClampSpeed']);
set_param([tgt '/ClampSpeed'], ...
    'UpperLimit', 'inf', ...
    'LowerLimit', '0', ...
    'Position', mat2str([390, 50, 430, 80]));

% Integrator: v_target -> x_target
add_block('simulink/Continuous/Integrator', [tgt '/pos_integrator_tgt']);
set_param([tgt '/pos_integrator_tgt'], ...
    'InitialCondition', 'SCENARIO_INITIAL_GAP', ...
    'Position', mat2str([460, 50, 500, 80]));

% Connections inside TargetVehicle
add_line(tgt, 'Clock/1', 'BrakeTimeCheck/1', 'autorouting', 'smart');
add_line(tgt, 'Clock/1', 'TimeSinceBrake/1', 'autorouting', 'smart');
add_line(tgt, 'brake_time_const/1', 'TimeSinceBrake/2', 'autorouting', 'smart');
add_line(tgt, 'TimeSinceBrake/1', 'ClampTime/1', 'autorouting', 'smart');
add_line(tgt, 'ClampTime/1', 'DecelProduct/1', 'autorouting', 'smart');
add_line(tgt, 'target_decel_val/1', 'DecelProduct/2', 'autorouting', 'smart');

% BrakeSwitch: port1=speed_with_decel, port2=condition, port3=v_init
add_line(tgt, 'SpeedSum/1', 'BrakeSwitch/1', 'autorouting', 'smart');
add_line(tgt, 'BrakeTimeCheck/1', 'BrakeSwitch/2', 'autorouting', 'smart');
add_line(tgt, 'v_target_init/1', 'BrakeSwitch/3', 'autorouting', 'smart');

% SpeedSum: v_init + decel*time
add_line(tgt, 'v_target_init/1', 'SpeedSum/1', 'autorouting', 'smart');
add_line(tgt, 'DecelProduct/1', 'SpeedSum/2', 'autorouting', 'smart');

add_line(tgt, 'BrakeSwitch/1', 'ClampSpeed/1', 'autorouting', 'smart');
add_line(tgt, 'ClampSpeed/1', 'v_target/1', 'autorouting', 'smart');
add_line(tgt, 'ClampSpeed/1', 'pos_integrator_tgt/1', 'autorouting', 'smart');
add_line(tgt, 'pos_integrator_tgt/1', 'x_target/1', 'autorouting', 'smart');

%% ===== RELATIVE KINEMATICS =====
% distance = x_target - x_ego
% rel_speed = v_ego - v_target

% Distance: x_target - x_ego
add_block('simulink/Math Operations/Sum', [mdl '/DistCalc']);
set_param([mdl '/DistCalc'], 'Inputs', '+-', ...
    'Position', mat2str([X0+9*DX, Y0+3*DY, X0+9*DX+BW, Y0+3*DY+BH]));

% Relative speed: v_ego - v_target
add_block('simulink/Math Operations/Sum', [mdl '/RelSpeedCalc']);
set_param([mdl '/RelSpeedCalc'], 'Inputs', '+-', ...
    'Position', mat2str([X0+9*DX, Y0+5*DY, X0+9*DX+BW, Y0+5*DY+BH]));

% Clamp distance >= 0 (collision = contact)
add_block('simulink/Discontinuities/Saturation', [mdl '/DistClamp']);
set_param([mdl '/DistClamp'], ...
    'UpperLimit', 'inf', ...
    'LowerLimit', '0', ...
    'Position', mat2str([X0+10*DX, Y0+3*DY, X0+10*DX+BW, Y0+3*DY+BH]));

% Collision detector: distance <= 0.5 m AND v_ego > 0.5 m/s
add_block('simulink/Logic and Bit Operations/Compare To Constant', [mdl '/CollisionCheck']);
set_param([mdl '/CollisionCheck'], ...
    'const', '0.5', ...
    'relop', '<=', ...
    'Position', mat2str([X0+11*DX, Y0+3*DY, X0+11*DX+BW, Y0+3*DY+BH]));

% Connect kinematics
add_line(mdl, 'TargetVehicle/2', 'DistCalc/1', 'autorouting', 'smart');  % x_target
add_line(mdl, 'EgoDynamics/2', 'DistCalc/2', 'autorouting', 'smart');    % x_ego
add_line(mdl, 'DistCalc/1', 'DistClamp/1', 'autorouting', 'smart');
add_line(mdl, 'DistClamp/1', 'CollisionCheck/1', 'autorouting', 'smart');

add_line(mdl, 'EgoDynamics/1', 'RelSpeedCalc/1', 'autorouting', 'smart');  % v_ego
add_line(mdl, 'TargetVehicle/1', 'RelSpeedCalc/2', 'autorouting', 'smart'); % v_target

%% ===== OUTPUT PORTS =====
outX = X0 + 13*DX;

add_block('simulink/Sinks/Out1', [mdl '/ego_speed']);
set_param([mdl '/ego_speed'], 'Port', '1', 'Position', mat2str([outX, Y0+1*DY, outX+20, Y0+1*DY+20]));

add_block('simulink/Sinks/Out1', [mdl '/ego_accel']);
set_param([mdl '/ego_accel'], 'Port', '2', 'Position', mat2str([outX, Y0+2*DY, outX+20, Y0+2*DY+20]));

add_block('simulink/Sinks/Out1', [mdl '/target_speed']);
set_param([mdl '/target_speed'], 'Port', '3', 'Position', mat2str([outX, Y0+4*DY, outX+20, Y0+4*DY+20]));

add_block('simulink/Sinks/Out1', [mdl '/distance']);
set_param([mdl '/distance'], 'Port', '4', 'Position', mat2str([outX, Y0+3*DY, outX+20, Y0+3*DY+20]));

add_block('simulink/Sinks/Out1', [mdl '/rel_speed']);
set_param([mdl '/rel_speed'], 'Port', '5', 'Position', mat2str([outX, Y0+5*DY, outX+20, Y0+5*DY+20]));

add_block('simulink/Sinks/Out1', [mdl '/collision']);
set_param([mdl '/collision'], 'Port', '6', 'Position', mat2str([outX, Y0+6*DY, outX+20, Y0+6*DY+20]));

% Connect outputs
add_line(mdl, 'EgoDynamics/1', 'ego_speed/1', 'autorouting', 'smart');
add_line(mdl, 'EgoDynamics/3', 'ego_accel/1', 'autorouting', 'smart');
add_line(mdl, 'TargetVehicle/1', 'target_speed/1', 'autorouting', 'smart');
add_line(mdl, 'DistClamp/1', 'distance/1', 'autorouting', 'smart');
add_line(mdl, 'RelSpeedCalc/1', 'rel_speed/1', 'autorouting', 'smart');
add_line(mdl, 'CollisionCheck/1', 'collision/1', 'autorouting', 'smart');

%% ===== SCOPES AND LOGGING =====
% Add To Workspace blocks for data logging

add_block('simulink/Sinks/To Workspace', [mdl '/log_ego_speed']);
set_param([mdl '/log_ego_speed'], ...
    'VariableName', 'ego_speed_log', ...
    'SaveFormat', 'Timeseries', ...
    'Position', mat2str([outX+100, Y0+1*DY, outX+160, Y0+1*DY+20]));
add_line(mdl, 'EgoDynamics/1', 'log_ego_speed/1', 'autorouting', 'smart');

add_block('simulink/Sinks/To Workspace', [mdl '/log_ego_accel']);
set_param([mdl '/log_ego_accel'], ...
    'VariableName', 'ego_accel_log', ...
    'SaveFormat', 'Timeseries', ...
    'Position', mat2str([outX+100, Y0+2*DY, outX+160, Y0+2*DY+20]));
add_line(mdl, 'EgoDynamics/3', 'log_ego_accel/1', 'autorouting', 'smart');

add_block('simulink/Sinks/To Workspace', [mdl '/log_distance']);
set_param([mdl '/log_distance'], ...
    'VariableName', 'distance_log', ...
    'SaveFormat', 'Timeseries', ...
    'Position', mat2str([outX+100, Y0+3*DY, outX+160, Y0+3*DY+20]));
add_line(mdl, 'DistClamp/1', 'log_distance/1', 'autorouting', 'smart');

add_block('simulink/Sinks/To Workspace', [mdl '/log_target_speed']);
set_param([mdl '/log_target_speed'], ...
    'VariableName', 'target_speed_log', ...
    'SaveFormat', 'Timeseries', ...
    'Position', mat2str([outX+100, Y0+4*DY, outX+160, Y0+4*DY+20]));
add_line(mdl, 'TargetVehicle/1', 'log_target_speed/1', 'autorouting', 'smart');

add_block('simulink/Sinks/To Workspace', [mdl '/log_rel_speed']);
set_param([mdl '/log_rel_speed'], ...
    'VariableName', 'rel_speed_log', ...
    'SaveFormat', 'Timeseries', ...
    'Position', mat2str([outX+100, Y0+5*DY, outX+160, Y0+5*DY+20]));
add_line(mdl, 'RelSpeedCalc/1', 'log_rel_speed/1', 'autorouting', 'smart');

%% ===== SCOPE (dashboard view) =====
add_block('simulink/Sinks/Scope', [mdl '/Dashboard']);
set_param([mdl '/Dashboard'], ...
    'NumInputPorts', '4', ...
    'Position', mat2str([outX+200, Y0+2*DY, outX+250, Y0+5*DY]));

add_line(mdl, 'EgoDynamics/1', 'Dashboard/1', 'autorouting', 'smart');
add_line(mdl, 'DistClamp/1', 'Dashboard/2', 'autorouting', 'smart');
add_line(mdl, 'RelSpeedCalc/1', 'Dashboard/3', 'autorouting', 'smart');
add_line(mdl, 'EgoDynamics/3', 'Dashboard/4', 'autorouting', 'smart');

%% ===== STIMULUS: Default brake input for standalone testing =====
% When running standalone (no AEB controller connected), use a step input.
% Step from 0 to 5 bar at t=3s to test brake response.

add_block('simulink/Sources/Step', [mdl '/TestBrakeStep']);
set_param([mdl '/TestBrakeStep'], ...
    'Time', '3', ...
    'Before', '0', ...
    'After', '5', ...
    'Position', mat2str(pos(1,1)));

% Manual Switch: choose between test input and external input
add_block('simulink/Signal Routing/Manual Switch', [mdl '/InputSelect']);
set_param([mdl '/InputSelect'], 'Position', mat2str(pos(1,2)));

add_line(mdl, 'TestBrakeStep/1', 'InputSelect/1', 'autorouting', 'smart');
add_line(mdl, 'brake_cmd_bar/1', 'InputSelect/2', 'autorouting', 'smart');

% Disconnect direct brake_cmd_bar -> bar_to_norm and reconnect via switch
delete_line(mdl, 'brake_cmd_bar/1', 'bar_to_norm/1');
add_line(mdl, 'InputSelect/1', 'bar_to_norm/1', 'autorouting', 'smart');

%% ===== ADD ANNOTATIONS =====
add_block('built-in/Note', [mdl '/Note_Title']);
set_param([mdl '/Note_Title'], ...
    'Position', mat2str([X0, Y0 - 40, X0+600, Y0-10]), ...
    'Text', 'AEB Plant Model - Vehicle Dynamics, Brake Actuator, Target Profile & Relative Kinematics');

add_block('built-in/Note', [mdl '/Note_Actuator']);
set_param([mdl '/Note_Actuator'], ...
    'Position', mat2str([X0+DX, Y0+3.5*DY, X0+5*DX, Y0+3.5*DY+15]), ...
    'Text', ['Brake Actuator: dead_time=' num2str(BRAKE_DEAD_TIME*1000) 'ms, tau=' num2str(BRAKE_TAU*1000) 'ms, max=' num2str(BRAKE_MAX_DECEL) 'm/s^2']);

%% ===== SAVE MODEL =====
save_system(mdl, fullfile(pwd, [mdl '.slx']));
fprintf('Model "%s.slx" saved successfully.\n', mdl);

%% ===== SCENARIO HELPER FUNCTIONS =====
% Define scenario presets that can be called before simulation

fprintf('\n=== Plant Model Created ===\n');
fprintf('Default scenario: CCRs (ego=50 km/h, target=stationary, gap=100m)\n');
fprintf('\nTo change scenario, set workspace variables before running sim:\n');
fprintf('  CCRs 50:  SCENARIO_EGO_SPEED=13.89; SCENARIO_TARGET_SPEED=0; SCENARIO_INITIAL_GAP=100;\n');
fprintf('  CCRm:     SCENARIO_EGO_SPEED=13.89; SCENARIO_TARGET_SPEED=5.56; SCENARIO_INITIAL_GAP=100;\n');
fprintf('  CCRb:     SCENARIO_EGO_SPEED=13.89; SCENARIO_TARGET_SPEED=13.89; SCENARIO_INITIAL_GAP=40; SCENARIO_TARGET_DECEL=-2; SCENARIO_BRAKE_TIME=3;\n');
fprintf('\nRun:  out = sim(''AEB_Plant'');\n');
fprintf('Plot: AEB_Plant_plot(out);\n');