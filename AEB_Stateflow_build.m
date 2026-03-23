%% AEB_Stateflow_build.m
%
% Builds the AEB (Autonomous Emergency Braking) Stateflow model in Simulink.
% Implements the 7-state FSM matching c_embedded/src/aeb_fsm.c behavior.
%
% States:
%   OFF (0), STANDBY (1), WARNING (2),
%   BRAKE_L1 (3), BRAKE_L2 (4), BRAKE_L3 (5), POST_BRAKE (6)
%
% Inputs:  ttc [s], d_brake [m], distance [m], v_ego [m/s], v_rel [m/s],
%          is_closing [bool], brake_pedal [bool], steering_angle [deg],
%          accel_pedal [bool], fault [bool]
%
% Outputs: fsm_state [0-6], target_decel [m/s^2],
%          alert_visual [bool], alert_audible [bool], brake_active [bool]
%
% Requirements:
%   FR-FSM-001: 7-state FSM
%   FR-FSM-002: TTC-based state transitions
%   FR-FSM-003: Escalation is immediate; de-escalation uses debounce
%   FR-FSM-004: De-escalation debounce of 200 ms
%   FR-FSM-005: Fault -> OFF
%   FR-DEC-006: Brake pedal / steering override -> STANDBY (during braking)
%   FR-DEC-007: Steering override (|angle| > 5 deg) -> STANDBY
%   FR-DEC-008: Speed range [5, 60] km/h = [1.39, 16.67] m/s
%   FR-ALR-003: All threats from STANDBY enter WARNING first; WARNING must
%               last >= 800 ms before BRAKE_L1
%   FR-BRK-005: POST_BRAKE holds brake for 2.0 s then -> STANDBY
%   FR-BRK-006: Accelerator override only in POST_BRAKE -> STANDBY
%
% Ref: c_embedded/include/aeb_config.h, c_embedded/src/aeb_fsm.c

%% ===== FSM Parameters (from aeb_config.h) =====
TTC_WARNING      = 4.0;    % TTC threshold for WARNING [s]
TTC_BRAKE_L1     = 3.0;    % TTC threshold for BRAKE_L1 [s]
TTC_BRAKE_L2     = 2.2;    % TTC threshold for BRAKE_L2 [s]
TTC_BRAKE_L3     = 1.8;    % TTC threshold for BRAKE_L3 [s]
DECEL_BRAKE_L1   = 2.0;    % Target deceleration BRAKE_L1 [m/s^2]
DECEL_BRAKE_L2   = 4.0;    % Target deceleration BRAKE_L2 [m/s^2]
DECEL_BRAKE_L3   = 6.0;    % Target deceleration BRAKE_L3 [m/s^2]
V_EGO_MIN        = 1.39;   % Minimum active speed [m/s] = 5 km/h (matches C code)
V_EGO_MAX        = 16.67;  % Maximum active speed [m/s] = 60 km/h
HYSTERESIS_TIME  = 0.2;    % De-escalation debounce time [s]
STEERING_DEG     = 5.0;    % Steering override threshold [deg]
POST_BRAKE_TIME  = 2.0;    % POST_BRAKE hold time after stop [s]
WARNING_MIN_TIME = 0.8;    % Minimum time in WARNING before BRAKE_L1 [s]
CTRL_DT          = 0.01;   % Controller sample time [s] = 10 ms
D_BRAKE_L1       = 20.0;   % Distance floor for L1 braking [m]
D_BRAKE_L2       = 10.0;   % Distance floor for L2 braking [m]
D_BRAKE_L3       = 5.0;    % Distance floor for L3 braking [m]

%% Export parameters to base workspace (required for Stateflow Parameter scope)
assignin('base', 'TTC_WARNING',      TTC_WARNING);
assignin('base', 'TTC_BRAKE_L1',     TTC_BRAKE_L1);
assignin('base', 'TTC_BRAKE_L2',     TTC_BRAKE_L2);
assignin('base', 'TTC_BRAKE_L3',     TTC_BRAKE_L3);
assignin('base', 'DECEL_BRAKE_L1',   DECEL_BRAKE_L1);
assignin('base', 'DECEL_BRAKE_L2',   DECEL_BRAKE_L2);
assignin('base', 'DECEL_BRAKE_L3',   DECEL_BRAKE_L3);
assignin('base', 'V_EGO_MIN',        V_EGO_MIN);
assignin('base', 'V_EGO_MAX',        V_EGO_MAX);
assignin('base', 'HYSTERESIS_TIME',  HYSTERESIS_TIME);
assignin('base', 'STEERING_DEG',     STEERING_DEG);
assignin('base', 'POST_BRAKE_TIME',  POST_BRAKE_TIME);
assignin('base', 'WARNING_MIN_TIME', WARNING_MIN_TIME);
assignin('base', 'CTRL_DT',          CTRL_DT);
assignin('base', 'D_BRAKE_L1',       D_BRAKE_L1);
assignin('base', 'D_BRAKE_L2',       D_BRAKE_L2);
assignin('base', 'D_BRAKE_L3',       D_BRAKE_L3);

%% ===== Create Simulink Model =====
model_name = 'AEB_Stateflow';
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
new_system(model_name);
open_system(model_name);
set_param(model_name, ...
    'SolverType', 'Fixed-step', ...
    'Solver',     'ode3',       ...
    'FixedStep',  '0.001',      ...
    'StopTime',   '15.0');

%% ===== Add Stateflow Chart Block =====
chart_path = [model_name '/AEB_FSM'];
add_block('sflib/Chart', chart_path, 'Position', [250 80 720 450]);

%% ===== Configure Chart =====
rt  = sfroot;
mdl = rt.find('-isa', 'Simulink.BlockDiagram', 'Name', model_name);
ch  = mdl.find('-isa', 'Stateflow.Chart');
ch.ChartUpdate    = 'DISCRETE';
ch.SampleTime     = num2str(CTRL_DT);
ch.ActionLanguage = 'MATLAB';

%% ===== Chart Inputs =====
inDefs = {
    'ttc',            'double',  1;
    'd_brake',        'double',  2;
    'distance',       'double',  3;
    'v_ego',          'double',  4;
    'v_rel',          'double',  5;
    'is_closing',     'boolean', 6;
    'brake_pedal',    'boolean', 7;
    'steering_angle', 'double',  8;
    'accel_pedal',    'boolean', 9;
    'fault',          'boolean', 10;
};
for k = 1:size(inDefs, 1)
    d           = Stateflow.Data(ch);
    d.Name      = inDefs{k, 1};
    d.Scope     = 'Input';
    d.DataType  = inDefs{k, 2};
    d.Port      = inDefs{k, 3};
end

%% ===== Chart Outputs =====
outDefs = {
    'fsm_state',     'double',  1;
    'target_decel',  'double',  2;
    'alert_visual',  'boolean', 3;
    'alert_audible', 'boolean', 4;
    'brake_active',  'boolean', 5;
};
for k = 1:size(outDefs, 1)
    d                    = Stateflow.Data(ch);
    d.Name               = outDefs{k, 1};
    d.Scope              = 'Output';
    d.DataType           = outDefs{k, 2};
    d.Port               = outDefs{k, 3};
    d.Props.InitialValue = '0';
end

%% ===== Local Variables (timers + helpers) =====
locDefs = {
    'warning_entry_time', 'double',  '0.0';
    'debounce_timer',     'double',  '0.0';
    'post_brake_timer',   'double',  '0.0';
    'desired_state',      'double',  '1.0';
};
for k = 1:size(locDefs, 1)
    d                    = Stateflow.Data(ch);
    d.Name               = locDefs{k, 1};
    d.Scope              = 'Local';
    d.DataType           = locDefs{k, 2};
    d.Props.InitialValue = locDefs{k, 3};
end

%% ===== Parameters =====
parDefs = {
    'TTC_WARNING',      TTC_WARNING;
    'TTC_BRAKE_L1',     TTC_BRAKE_L1;
    'TTC_BRAKE_L2',     TTC_BRAKE_L2;
    'TTC_BRAKE_L3',     TTC_BRAKE_L3;
    'DECEL_BRAKE_L1',   DECEL_BRAKE_L1;
    'DECEL_BRAKE_L2',   DECEL_BRAKE_L2;
    'DECEL_BRAKE_L3',   DECEL_BRAKE_L3;
    'V_EGO_MIN',        V_EGO_MIN;
    'V_EGO_MAX',        V_EGO_MAX;
    'HYSTERESIS_TIME',  HYSTERESIS_TIME;
    'STEERING_DEG',     STEERING_DEG;
    'POST_BRAKE_TIME',  POST_BRAKE_TIME;
    'WARNING_MIN_TIME', WARNING_MIN_TIME;
    'CTRL_DT',          CTRL_DT;
    'D_BRAKE_L1',       D_BRAKE_L1;
    'D_BRAKE_L2',       D_BRAKE_L2;
    'D_BRAKE_L3',       D_BRAKE_L3;
};
for k = 1:size(parDefs, 1)
    d          = Stateflow.Data(ch);
    d.Name     = parDefs{k, 1};
    d.Scope    = 'Parameter';
    d.DataType = 'double';
end

%% ===== Shared action strings =====
% evaluate_threat: mirrors evaluate_threat() in aeb_fsm.c
% Includes is_closing guard and distance-floor logic.
desiredStateCode = [...
    '  if ~is_closing' newline ...
    '    desired_state = 1;' newline ...
    '  elseif ttc <= TTC_BRAKE_L3 || (d_brake >= distance)' newline ...
    '    desired_state = 5;' newline ...
    '  elseif ttc <= TTC_BRAKE_L2' newline ...
    '    desired_state = 4;' newline ...
    '  elseif ttc <= TTC_BRAKE_L1' newline ...
    '    desired_state = 3;' newline ...
    '  elseif ttc <= TTC_WARNING' newline ...
    '    desired_state = 2;' newline ...
    '  else' newline ...
    '    desired_state = 1;' newline ...
    '  end' newline ...
    '  %% Distance floor (prevents TTC-based de-escalation when close)' newline ...
    '  if is_closing' newline ...
    '    if distance <= D_BRAKE_L3 && desired_state < 5' newline ...
    '      desired_state = 5;' newline ...
    '    elseif distance <= D_BRAKE_L2 && desired_state < 4' newline ...
    '      desired_state = 4;' newline ...
    '    elseif distance <= D_BRAKE_L1 && desired_state < 3' newline ...
    '      desired_state = 3;' newline ...
    '    end' newline ...
    '  end' newline ...
];

%% ===== State Layout =====
W = 180; H = 120; GX = 50; GY = 50;
col = [GX, GX + W + 60, GX + 2*(W + 60)];
row = [GY, GY + H + 70, GY + 2*(H + 70)];

%% ===== Create States =====
sOFF  = Stateflow.State(ch);  sOFF.Name  = 'OFF';
sSTBY = Stateflow.State(ch);  sSTBY.Name = 'STANDBY';
sWRN  = Stateflow.State(ch);  sWRN.Name  = 'WARNING';
sBL1  = Stateflow.State(ch);  sBL1.Name  = 'BRAKE_L1';
sBL2  = Stateflow.State(ch);  sBL2.Name  = 'BRAKE_L2';
sBL3  = Stateflow.State(ch);  sBL3.Name  = 'BRAKE_L3';
sPOST = Stateflow.State(ch);  sPOST.Name = 'POST_BRAKE';

sOFF.Position  = [col(1), row(1), W, H];
sSTBY.Position = [col(2), row(1), W, H];
sWRN.Position  = [col(3), row(1), W, H];
sBL1.Position  = [col(1), row(2), W, H];
sBL2.Position  = [col(2), row(2), W, H];
sBL3.Position  = [col(3), row(2), W, H];
sPOST.Position = [col(2), row(3), W, H];

%% ===== State Labels =====

% --- OFF ---
sOFF.LabelString = [...
    'OFF' newline ...
    'entry:' newline ...
    '  fsm_state = 0;' newline ...
    '  target_decel = 0.0;' newline ...
    '  alert_visual = false;' newline ...
    '  alert_audible = false;' newline ...
    '  brake_active = false;' newline ...
    '  debounce_timer = 0.0;' newline ...
    '  warning_entry_time = 0.0;' newline ...
    '  post_brake_timer = 0.0;' newline ...
    '  desired_state = 0.0;' ...
];

% --- STANDBY ---
sSTBY.LabelString = [...
    'STANDBY' newline ...
    'entry:' newline ...
    '  fsm_state = 1;' newline ...
    '  target_decel = 0.0;' newline ...
    '  alert_visual = false;' newline ...
    '  alert_audible = false;' newline ...
    '  brake_active = false;' newline ...
    '  debounce_timer = 0.0;' newline ...
    '  warning_entry_time = 0.0;' newline ...
    '  post_brake_timer = 0.0;' newline ...
    '  desired_state = 1.0;' ...
];

% --- WARNING ---
% during: accumulate warning_entry_time always;
%         compute desired_state; accumulate debounce only for de-escalation
sWRN.LabelString = [...
    'WARNING' newline ...
    'entry:' newline ...
    '  fsm_state = 2;' newline ...
    '  target_decel = 0.0;' newline ...
    '  alert_visual = true;' newline ...
    '  alert_audible = true;' newline ...
    '  brake_active = false;' newline ...
    '  debounce_timer = 0.0;' newline ...
    '  warning_entry_time = 0.0;' newline ...
    '  desired_state = 2.0;' newline ...
    'during:' newline ...
    '  warning_entry_time = warning_entry_time + CTRL_DT;' newline ...
    desiredStateCode ...
    '  if desired_state < 2' newline ...
    '    debounce_timer = debounce_timer + CTRL_DT;' newline ...
    '  else' newline ...
    '    debounce_timer = 0.0;' newline ...
    '  end' ...
];

% --- BRAKE_L1 ---
sBL1.LabelString = [...
    'BRAKE_L1' newline ...
    'entry:' newline ...
    '  fsm_state = 3;' newline ...
    '  target_decel = DECEL_BRAKE_L1;' newline ...
    '  alert_visual = true;' newline ...
    '  alert_audible = true;' newline ...
    '  brake_active = true;' newline ...
    '  debounce_timer = 0.0;' newline ...
    '  desired_state = 3.0;' newline ...
    'during:' newline ...
    desiredStateCode ...
    '  if desired_state < 3' newline ...
    '    debounce_timer = debounce_timer + CTRL_DT;' newline ...
    '  else' newline ...
    '    debounce_timer = 0.0;' newline ...
    '  end' ...
];

% --- BRAKE_L2 ---
sBL2.LabelString = [...
    'BRAKE_L2' newline ...
    'entry:' newline ...
    '  fsm_state = 4;' newline ...
    '  target_decel = DECEL_BRAKE_L2;' newline ...
    '  alert_visual = true;' newline ...
    '  alert_audible = true;' newline ...
    '  brake_active = true;' newline ...
    '  debounce_timer = 0.0;' newline ...
    '  desired_state = 4.0;' newline ...
    'during:' newline ...
    desiredStateCode ...
    '  if desired_state < 4' newline ...
    '    debounce_timer = debounce_timer + CTRL_DT;' newline ...
    '  else' newline ...
    '    debounce_timer = 0.0;' newline ...
    '  end' ...
];

% --- BRAKE_L3 ---
sBL3.LabelString = [...
    'BRAKE_L3' newline ...
    'entry:' newline ...
    '  fsm_state = 5;' newline ...
    '  target_decel = DECEL_BRAKE_L3;' newline ...
    '  alert_visual = true;' newline ...
    '  alert_audible = true;' newline ...
    '  brake_active = true;' newline ...
    '  debounce_timer = 0.0;' newline ...
    '  desired_state = 5.0;' newline ...
    'during:' newline ...
    desiredStateCode ...
    '  if desired_state < 5' newline ...
    '    debounce_timer = debounce_timer + CTRL_DT;' newline ...
    '  else' newline ...
    '    debounce_timer = 0.0;' newline ...
    '  end' ...
];

% --- POST_BRAKE ---
% Fix #6: alert_visual = true (brake lights stay on), alert_audible = false
sPOST.LabelString = [...
    'POST_BRAKE' newline ...
    'entry:' newline ...
    '  fsm_state = 6;' newline ...
    '  target_decel = DECEL_BRAKE_L3;' newline ...
    '  alert_visual = true;' newline ...
    '  alert_audible = false;' newline ...
    '  brake_active = true;' newline ...
    '  post_brake_timer = 0.0;' newline ...
    '  debounce_timer = 0.0;' newline ...
    '  desired_state = 6.0;' newline ...
    'during:' newline ...
    '  post_brake_timer = post_brake_timer + CTRL_DT;' ...
];

%% ===== Default Transition (-> STANDBY on chart initialization) =====
dtrans                   = Stateflow.Transition(ch);
dtrans.Destination       = sSTBY;
dtrans.DestinationOClock = 9;
dtrans.SourceEndpoint    = [col(2) + W/2, row(1) - 35];

%% ===== Transition Helper =====
function lbl = mkLbl(cond, action)
    if isempty(cond) && isempty(action)
        lbl = '';
    elseif isempty(action)
        lbl = ['[' cond ']'];
    elseif isempty(cond)
        lbl = ['{' action '}'];
    else
        lbl = ['[' cond ']{' action '}'];
    end
end

function t = mkTr(src, dst, cond, prio)
    t                 = Stateflow.Transition(src.Chart);
    t.Source          = src;
    t.Destination     = dst;
    t.LabelString     = mkLbl(cond, '');
    t.ExecutionOrder  = prio;
end

% Convenience condition strings
% NOM: normal operating mode (no fault, speed in range, no driver override)
% Note: driver override (brake_pedal + steering) only checked via OVR_BRAKE
%       for braking states. accel_pedal only checked in POST_BRAKE via OVR_POST.
NOM = ['~fault && v_ego >= V_EGO_MIN && v_ego <= V_EGO_MAX'];

% Override during active braking: brake pedal OR steering > 5 deg
OVR_BRAKE = ['~fault && (brake_pedal || abs(steering_angle) > STEERING_DEG)'];

% Override for non-braking states: speed out of range or driver input
OVR_SPEED = ['~fault && (v_ego < V_EGO_MIN || v_ego > V_EGO_MAX)'];

% Override for POST_BRAKE: accelerator pedal only
OVR_POST = ['~fault && accel_pedal'];

%% ===== Transitions: FROM OFF =====
% OFF -> STANDBY when fault clears (C code: case AEB_OFF -> AEB_STANDBY)
mkTr(sOFF, sSTBY, '~fault', 1);

%% ===== Transitions: FROM STANDBY =====
% P1: Fault -> OFF
mkTr(sSTBY, sOFF, 'fault', 1);
% P2: Speed out of range -> stay STANDBY (implicit, no transition needed)
% P3: Any threat (desired >= WARNING) -> WARNING first (matches C code lines 228-240)
%     C code: if desired >= WARNING then transition_to(WARNING)
%     All TTC threats enter WARNING first; escalation happens from WARNING.
mkTr(sSTBY, sWRN, [NOM ' && is_closing && ' ...
     '(ttc <= TTC_WARNING || d_brake >= distance ' ...
     '|| distance <= D_BRAKE_L1)'], 2);

%% ===== Transitions: FROM WARNING =====
% P1: Fault -> OFF
mkTr(sWRN, sOFF,  'fault', 1);
% P2: Speed out of range -> STANDBY
mkTr(sWRN, sSTBY, OVR_SPEED, 2);
% P3: Driver override (brake/steering) -> STANDBY
mkTr(sWRN, sSTBY, OVR_BRAKE, 3);
% P4-P6: TTC escalation gated by 800 ms minimum in WARNING (FR-ALR-003)
%         Matches C code: WARNING case, s_warning_accum >= WARNING_TO_BRAKE_MIN
mkTr(sWRN, sBL3, [NOM ' && desired_state == 5' ...
     ' && warning_entry_time >= WARNING_MIN_TIME'], 4);
mkTr(sWRN, sBL2, [NOM ' && desired_state == 4' ...
     ' && warning_entry_time >= WARNING_MIN_TIME'], 5);
mkTr(sWRN, sBL1, [NOM ' && desired_state == 3' ...
     ' && warning_entry_time >= WARNING_MIN_TIME'], 6);
% P7: De-escalation to STANDBY (debounced, FR-FSM-004)
mkTr(sWRN, sSTBY, [NOM ' && desired_state <= 1' ...
     ' && debounce_timer >= HYSTERESIS_TIME'], 7);

%% ===== Transitions: FROM BRAKE_L1 =====
mkTr(sBL1, sOFF,  'fault', 1);
mkTr(sBL1, sSTBY, OVR_SPEED, 2);
% Driver override (brake pedal or steering) -> STANDBY
mkTr(sBL1, sSTBY, OVR_BRAKE, 3);
% Vehicle stopped -> POST_BRAKE
mkTr(sBL1, sPOST, [NOM ' && v_ego <= 0.01'], 4);
% Escalation: immediate (C code lines 289-294)
mkTr(sBL1, sBL3,  [NOM ' && v_ego > 0.01 && desired_state == 5'], 5);
mkTr(sBL1, sBL2,  [NOM ' && v_ego > 0.01 && desired_state == 4'], 6);
% De-escalation: single step to WARNING only (C code line 301: transition_to(AEB_WARNING))
mkTr(sBL1, sWRN,  [NOM ' && v_ego > 0.01 && desired_state < 3' ...
     ' && debounce_timer >= HYSTERESIS_TIME'], 7);

%% ===== Transitions: FROM BRAKE_L2 =====
mkTr(sBL2, sOFF,  'fault', 1);
mkTr(sBL2, sSTBY, OVR_SPEED, 2);
mkTr(sBL2, sSTBY, OVR_BRAKE, 3);
mkTr(sBL2, sPOST, [NOM ' && v_ego <= 0.01'], 4);
% Escalation: immediate to L3 (C code line 321)
mkTr(sBL2, sBL3,  [NOM ' && v_ego > 0.01 && desired_state == 5'], 5);
% De-escalation: single step to L1 only (C code line 330: transition_to(AEB_BRAKE_L1))
mkTr(sBL2, sBL1,  [NOM ' && v_ego > 0.01 && desired_state < 4' ...
     ' && debounce_timer >= HYSTERESIS_TIME'], 6);

%% ===== Transitions: FROM BRAKE_L3 =====
mkTr(sBL3, sOFF,  'fault', 1);
mkTr(sBL3, sSTBY, OVR_SPEED, 2);
mkTr(sBL3, sSTBY, OVR_BRAKE, 3);
mkTr(sBL3, sPOST, [NOM ' && v_ego <= 0.01'], 4);
% De-escalation: single step (C code lines 351-358)
% If desired is L2 or L1 -> transition_to(desired)
% If desired is WARNING or STANDBY -> transition_to(AEB_POST_BRAKE)
mkTr(sBL3, sBL2,  [NOM ' && v_ego > 0.01 && desired_state == 4' ...
     ' && debounce_timer >= HYSTERESIS_TIME'], 5);
mkTr(sBL3, sBL1,  [NOM ' && v_ego > 0.01 && desired_state == 3' ...
     ' && debounce_timer >= HYSTERESIS_TIME'], 6);
mkTr(sBL3, sPOST, [NOM ' && v_ego > 0.01 && desired_state <= 2' ...
     ' && debounce_timer >= HYSTERESIS_TIME'], 7);

%% ===== Transitions: FROM POST_BRAKE =====
mkTr(sPOST, sOFF,  'fault', 1);
% Accelerator override only in POST_BRAKE (FR-BRK-006)
mkTr(sPOST, sSTBY, OVR_POST, 2);
% Timer expiry -> release brake (FR-BRK-005)
mkTr(sPOST, sSTBY, ['~fault && post_brake_timer >= POST_BRAKE_TIME'], 3);

%% ===== Add Simulink I/O Ports =====
inNames  = {'In_ttc','In_d_brake','In_distance','In_v_ego','In_v_rel', ...
            'In_is_closing','In_brake_pedal','In_steering_angle', ...
            'In_accel_pedal','In_fault'};
outNames = {'Out_fsm_state','Out_target_decel','Out_alert_visual', ...
            'Out_alert_audible','Out_brake_active'};

yIn = 90;
for k = 1:numel(inNames)
    blkPath = [model_name '/' inNames{k}];
    add_block('simulink/Sources/In1', blkPath, ...
              'Position', [30, yIn, 80, yIn+20]);
    add_line(model_name, [inNames{k} '/1'], ['AEB_FSM/' num2str(k)], ...
             'autorouting', 'on');
    yIn = yIn + 35;
end

yOut = 110;
for k = 1:numel(outNames)
    blkPath = [model_name '/' outNames{k}];
    add_block('simulink/Sinks/Out1', blkPath, ...
              'Position', [780, yOut, 830, yOut+20]);
    add_line(model_name, ['AEB_FSM/' num2str(k)], [outNames{k} '/1'], ...
             'autorouting', 'on');
    yOut = yOut + 50;
end

%% ===== Save Model =====
save_system(model_name, fullfile(pwd, [model_name '.slx']));
fprintf('\n=== AEB Stateflow model built successfully ===\n');
fprintf('File : %s.slx\n', model_name);
fprintf('\nStates:\n');
fprintf('  0 = OFF        | 1 = STANDBY    | 2 = WARNING\n');
fprintf('  3 = BRAKE_L1   | 4 = BRAKE_L2   | 5 = BRAKE_L3\n');
fprintf('  6 = POST_BRAKE\n');
fprintf('\nTTC thresholds:\n');
fprintf('  WARNING  : TTC <= %.1f s\n', TTC_WARNING);
fprintf('  BRAKE_L1 : TTC <= %.1f s  ->  %.0f m/s^2\n', TTC_BRAKE_L1, DECEL_BRAKE_L1);
fprintf('  BRAKE_L2 : TTC <= %.1f s  ->  %.0f m/s^2\n', TTC_BRAKE_L2, DECEL_BRAKE_L2);
fprintf('  BRAKE_L3 : TTC <= %.1f s  ->  %.0f m/s^2\n', TTC_BRAKE_L3, DECEL_BRAKE_L3);
fprintf('\nDistance floors: L1=%.0fm  L2=%.0fm  L3=%.0fm\n', ...
    D_BRAKE_L1, D_BRAKE_L2, D_BRAKE_L3);
fprintf('Sample time : %.0f ms  |  Speed range: %.0f-%.0f km/h\n', ...
    CTRL_DT*1000, V_EGO_MIN*3.6, V_EGO_MAX*3.6);
fprintf('\nFixes applied vs previous version:\n');
fprintf('  [1] V_EGO_MIN = 1.39 m/s (5 km/h) -- matches aeb_config.h\n');
fprintf('  [2] STANDBY always enters WARNING first -- matches aeb_fsm.c\n');
fprintf('  [3] is_closing input added -- no braking for receding targets\n');
fprintf('  [4] Distance floor (D_BRAKE_L1/L2/L3) -- prevents premature de-escalation\n');
fprintf('  [5] De-escalation is single-step only -- matches aeb_fsm.c\n');
fprintf('  [6] POST_BRAKE alert_visual = true -- matches aeb_fsm.c\n');
fprintf('  [7] Override differentiation: brake/steering during braking,\n');
fprintf('      accelerator only in POST_BRAKE\n');