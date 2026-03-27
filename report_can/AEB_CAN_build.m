%% AEB_CAN_build.m
% Gera AEB_CAN.slx: modelo do barramento CAN do sistema AEB.
%
% Mensagens (can/aeb_system.dbc, 500 kbps):
%   AEB_BrakeCmd    0x080  4 bytes  10ms  ASIL-B  (AliveCounter + CRC-4)
%   AEB_EgoVehicle  0x100  8 bytes  10ms
%   AEB_RadarTarget 0x120  8 bytes  20ms  (AliveCounter)
%   AEB_FSMState    0x200  4 bytes  50ms
%   AEB_Alert       0x300  2 bytes  evento
%
% Requisitos: MATLAB R2025b + Simulink
%
% Uso:
%   run('AEB_CAN_build.m')           % cria o modelo
%   sim('AEB_CAN')                   % simula (cenário padrão)
%   AEB_CAN_scenarios('fault_crc')   % injeta falha
%   AEB_CAN_plot                     % plota resultados

clear; close all; clc;

%% ── 0. Configuração ──────────────────────────────────────────────────────────
script_dir = fileparts(mfilename('fullpath'));
mdl = 'AEB_CAN';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);
sys = mdl;

set_param(sys, ...
    'Solver',            'ode4', ...
    'SolverType',        'Fixed-step', ...
    'FixedStep',         '0.001', ...
    'StartTime',         '0', ...
    'StopTime',          '5', ...
    'SignalLogging',     'on', ...
    'SignalLoggingName', 'logsout');

%% ── 1. Dados de cenário padrão (CCRs 50 km/h) ───────────────────────────────
t = (0:0.001:5)';
N = length(t);

% Ego: 50 km/h, frena com -4 m/s² a partir de t=2s
v_ego  = max(0, 13.89 - 4.0 * max(t - 2.0, 0));
d_tgt  = max(0.5, 50 - cumtrapz(t, v_ego));
v_rel  = v_ego;  % alvo parado
ttc    = min(20, d_tgt ./ max(v_rel, 0.01));
a_ego  = [0; diff(v_ego)/0.001];

% FSM (baseado em TTC)
fsm = ones(N,1);
fsm(ttc <= 4.0) = 2;  % WARNING
fsm(ttc <= 3.0) = 3;  % BRAKE_L1
fsm(ttc <= 2.2) = 4;  % BRAKE_L2
fsm(ttc <= 1.8) = 5;  % BRAKE_L3
fsm(v_ego < 0.1) = 1; % volta a STANDBY quando parou

% Pressão de freio
bpress = zeros(N,1);
bpress(fsm==3) = 2.0; bpress(fsm==4) = 4.0; bpress(fsm==5) = 6.0;
breq   = double(bpress > 0);
bmode  = max(0, fsm - 1);  % 0=Off, 1=Warning, 2=L1, 3=L2, 4=L3

% Alert
alevel = zeros(N,1);
alevel(fsm==2)=1; alevel(fsm==3)=2; alevel(fsm>=4)=3;
atype  = double(fsm >= 2) * 3;  % 3=BOTH
aactive= double(fsm >= 2);
buzzer = zeros(N,1);
buzzer(fsm==2)=1; buzzer(fsm==3)=2; buzzer(fsm==4)=4; buzzer(fsm==5)=3;

% TTCThreshold por estado
ttc_thr = 4.0*ones(N,1);
ttc_thr(fsm==3)=3.0; ttc_thr(fsm==4)=2.2; ttc_thr(fsm==5)=1.8;

% Confiança radar
conf = 15*ones(N,1);

% Injeção de falhas
fault_dropout = double(t >= 3.5 & t < 3.6);   % 100ms dropout
fault_crc     = double(t >= 4.0 & t < 4.05);  % 50ms CRC corrompido
fault_alive   = double(t >= 4.5 & t < 4.7);   % 200ms alive travado

% Exportar para workspace base (formato [t, dado])
vars = {'breq','bpress','bmode','v_ego','a_ego','d_tgt','v_rel','ttc','fsm',...
        'conf','alevel','atype','aactive','buzzer','ttc_thr',...
        'fault_dropout','fault_crc','fault_alive'};
for k = 1:numel(vars)
    assignin('base', ['ws_' vars{k}], [t, eval(vars{k})]);
end

%% ── 2. Construir subsistemas ─────────────────────────────────────────────────
bld_tx_brakecmd(sys);
bld_tx_egovehicle(sys);
bld_tx_radartarget(sys);
bld_tx_fsmstate(sys);
bld_tx_alert(sys);
bld_fault_inj(sys);
bld_rx_brakecmd(sys);
bld_rx_egovehicle(sys);
bld_rx_radartarget(sys);
bld_rx_fsmstate(sys);
bld_rx_alert(sys);
bld_diagnosticos(sys);

%% ── 3. Blocos From Workspace (top-level) ────────────────────────────────────
fw_list = {
    'FW_breq',   'ws_breq',   [50,100,170,115];
    'FW_bpress', 'ws_bpress', [50,130,170,145];
    'FW_bmode',  'ws_bmode',  [50,160,170,175];
    'FW_v_ego',  'ws_v_ego',  [50,220,170,235];
    'FW_a_ego',  'ws_a_ego',  [50,250,170,265];
    'FW_d_tgt',  'ws_d_tgt',  [50,330,170,345];
    'FW_v_rel',  'ws_v_rel',  [50,360,170,375];
    'FW_ttc',    'ws_ttc',    [50,390,170,405];
    'FW_conf',   'ws_conf',   [50,420,170,435];
    'FW_fsm',    'ws_fsm',    [50,480,170,495];
    'FW_alevel', 'ws_alevel', [50,510,170,525];
    'FW_ttc_thr','ws_ttc_thr',[50,540,170,555];
    'FW_atype',  'ws_atype',  [50,600,170,615];
    'FW_aactive','ws_aactive',[50,630,170,645];
    'FW_buzzer', 'ws_buzzer', [50,660,170,675];
    'FW_fdrop',  'ws_fault_dropout',[50,740,170,755];
    'FW_fcrc',   'ws_fault_crc',   [50,770,170,785];
    'FW_falive', 'ws_fault_alive', [50,800,170,815];
};
for k = 1:size(fw_list,1)
    blk = [sys '/' fw_list{k,1}];
    add_block('simulink/Sources/From Workspace', blk, ...
        'VariableName', fw_list{k,2}, ...
        'Position',     fw_list{k,3}, ...
        'Interpolate',  'off');
end

%% ── 4. Posicionar subsistemas ────────────────────────────────────────────────
set_param([sys '/TX_BrakeCmd'],    'Position',[280, 90,430,185]);
set_param([sys '/TX_EgoVehicle'],  'Position',[280,215,430,285]);
set_param([sys '/TX_RadarTarget'], 'Position',[280,320,430,435]);
set_param([sys '/TX_FSMState'],    'Position',[280,470,430,560]);
set_param([sys '/TX_Alert'],       'Position',[280,590,430,670]);
set_param([sys '/Injecao_Falhas'], 'Position',[520, 90,680,230]);
set_param([sys '/RX_BrakeCmd'],    'Position',[760, 90,920,210]);
set_param([sys '/RX_EgoVehicle'],  'Position',[760,235,920,305]);
set_param([sys '/RX_RadarTarget'], 'Position',[760,330,920,445]);
set_param([sys '/RX_FSMState'],    'Position',[760,470,920,545]);
set_param([sys '/RX_Alert'],       'Position',[760,570,920,640]);
set_param([sys '/Diagnosticos'],   'Position',[1000, 90,1160,210]);

%% ── 5. Conexões top-level ────────────────────────────────────────────────────
% TX_BrakeCmd
add_line(sys,'FW_breq/1',  'TX_BrakeCmd/1','autorouting','on');
add_line(sys,'FW_bpress/1','TX_BrakeCmd/2','autorouting','on');
add_line(sys,'FW_bmode/1', 'TX_BrakeCmd/3','autorouting','on');

% TX_EgoVehicle
add_line(sys,'FW_v_ego/1','TX_EgoVehicle/1','autorouting','on');
add_line(sys,'FW_a_ego/1','TX_EgoVehicle/2','autorouting','on');

% TX_RadarTarget
add_line(sys,'FW_d_tgt/1','TX_RadarTarget/1','autorouting','on');
add_line(sys,'FW_v_rel/1','TX_RadarTarget/2','autorouting','on');
add_line(sys,'FW_ttc/1',  'TX_RadarTarget/3','autorouting','on');
add_line(sys,'FW_conf/1', 'TX_RadarTarget/4','autorouting','on');

% TX_FSMState
add_line(sys,'FW_fsm/1',    'TX_FSMState/1','autorouting','on');
add_line(sys,'FW_alevel/1', 'TX_FSMState/2','autorouting','on');
add_line(sys,'FW_breq/1',   'TX_FSMState/3','autorouting','on');
add_line(sys,'FW_ttc_thr/1','TX_FSMState/4','autorouting','on');

% TX_Alert
add_line(sys,'FW_atype/1',  'TX_Alert/1','autorouting','on');
add_line(sys,'FW_aactive/1','TX_Alert/2','autorouting','on');
add_line(sys,'FW_buzzer/1', 'TX_Alert/3','autorouting','on');

% Injecao_Falhas ← TX_BrakeCmd + fault signals
for k=1:5
    add_line(sys,['TX_BrakeCmd/' num2str(k)],['Injecao_Falhas/' num2str(k)],'autorouting','on');
end
add_line(sys,'FW_fdrop/1', 'Injecao_Falhas/6','autorouting','on');
add_line(sys,'FW_fcrc/1',  'Injecao_Falhas/7','autorouting','on');
add_line(sys,'FW_falive/1','Injecao_Falhas/8','autorouting','on');

% RX_BrakeCmd ← Injecao_Falhas
for k=1:6
    add_line(sys,['Injecao_Falhas/' num2str(k)],['RX_BrakeCmd/' num2str(k)],'autorouting','on');
end

% RX_EgoVehicle ← TX_EgoVehicle
for k=1:4
    add_line(sys,['TX_EgoVehicle/' num2str(k)],['RX_EgoVehicle/' num2str(k)],'autorouting','on');
end

% RX_RadarTarget ← TX_RadarTarget
for k=1:5
    add_line(sys,['TX_RadarTarget/' num2str(k)],['RX_RadarTarget/' num2str(k)],'autorouting','on');
end

% RX_FSMState ← TX_FSMState
for k=1:4
    add_line(sys,['TX_FSMState/' num2str(k)],['RX_FSMState/' num2str(k)],'autorouting','on');
end

% RX_Alert ← TX_Alert
for k=1:3
    add_line(sys,['TX_Alert/' num2str(k)],['RX_Alert/' num2str(k)],'autorouting','on');
end

% Diagnosticos ← RX_BrakeCmd erros
add_line(sys,'RX_BrakeCmd/4','Diagnosticos/1','autorouting','on');
add_line(sys,'RX_BrakeCmd/5','Diagnosticos/2','autorouting','on');
add_line(sys,'RX_BrakeCmd/6','Diagnosticos/3','autorouting','on');

%% ── 6. Blocos To Workspace ───────────────────────────────────────────────────
tw_list = {
    'TW_breq_dec',   'breq_dec',   [1250, 90,1350,105], 'RX_BrakeCmd/1';
    'TW_bpress_dec', 'bpress_dec', [1250,120,1350,135], 'RX_BrakeCmd/2';
    'TW_bmode_dec',  'bmode_dec',  [1250,150,1350,165], 'RX_BrakeCmd/3';
    'TW_crc_err',    'crc_err',    [1250,180,1350,195], 'RX_BrakeCmd/4';
    'TW_alive_err',  'alive_err',  [1250,210,1350,225], 'RX_BrakeCmd/5';
    'TW_timeout_err','timeout_err',[1250,240,1350,255], 'RX_BrakeCmd/6';
    'TW_dtc1004',    'dtc_c1004',  [1250,290,1350,305], 'Diagnosticos/1';
    'TW_dtc1005',    'dtc_c1005',  [1250,320,1350,335], 'Diagnosticos/2';
    'TW_dtc1010',    'dtc_c1010',  [1250,350,1350,365], 'Diagnosticos/3';
    'TW_err_count',  'err_count',  [1250,380,1350,395], 'Diagnosticos/4';
    'TW_spd_dec',    'spd_dec',    [1250,420,1350,435], 'RX_EgoVehicle/1';
    'TW_accel_dec',  'accel_dec',  [1250,450,1350,465], 'RX_EgoVehicle/2';
    'TW_dist_dec',   'dist_dec',   [1250,490,1350,505], 'RX_RadarTarget/1';
    'TW_vrel_dec',   'vrel_dec',   [1250,520,1350,535], 'RX_RadarTarget/2';
    'TW_ttc_dec',    'ttc_dec',    [1250,550,1350,565], 'RX_RadarTarget/3';
    'TW_fsm_dec',    'fsm_dec',    [1250,590,1350,605], 'RX_FSMState/1';
    'TW_alert_dec',  'alert_dec',  [1250,620,1350,635], 'RX_Alert/1';
};
for k=1:size(tw_list,1)
    blk = [sys '/' tw_list{k,1}];
    add_block('simulink/Sinks/To Workspace', blk, ...
        'VariableName', tw_list{k,2}, 'SaveFormat','Array', 'Position', tw_list{k,3});
    add_line(sys, tw_list{k,4}, [tw_list{k,1} '/1'], 'autorouting','on');
end

% Terminadores para saídas não logadas
terms = {
    'Term_ego3',  'RX_EgoVehicle/3';
    'Term_ego4',  'RX_EgoVehicle/4';
    'Term_rt4',   'RX_RadarTarget/4';
    'Term_rt5',   'RX_RadarTarget/5';
    'Term_fsm2',  'RX_FSMState/2';
    'Term_fsm3',  'RX_FSMState/3';
    'Term_fsm4',  'RX_FSMState/4';
    'Term_alt2',  'RX_Alert/2';
    'Term_alt3',  'RX_Alert/3';
};
for k=1:size(terms,1)
    blk = [sys '/' terms{k,1}];
    add_block('simulink/Sinks/Terminator', blk, 'Position',[1250,650+20*k,1270,665+20*k]);
    add_line(sys, terms{k,2}, [terms{k,1} '/1'], 'autorouting','on');
end

% Scope
add_block('simulink/Sinks/Scope',[sys '/Scope_CAN'],...
    'NumInputPorts','4','Position',[1250,870,1330,930]);
add_line(sys,'RX_BrakeCmd/4','Scope_CAN/1','autorouting','on');
add_line(sys,'RX_BrakeCmd/5','Scope_CAN/2','autorouting','on');
add_line(sys,'Diagnosticos/1','Scope_CAN/3','autorouting','on');
add_line(sys,'Diagnosticos/2','Scope_CAN/4','autorouting','on');

%% ── 7. Auto-layout e salvamento ──────────────────────────────────────────────
Simulink.BlockDiagram.arrangeSystem(sys, 'FullLayout', 'on');
lns = find_system(sys,'FindAll','on','type','line');
if ~isempty(lns), Simulink.BlockDiagram.routeLine(lns); end

save_system(mdl, fullfile(script_dir, [mdl '.slx']));
fprintf('\n=== AEB_CAN.slx salvo: %s ===\n', script_dir);
fprintf('  sim(''AEB_CAN'')                     — cenário padrão\n');
fprintf('  AEB_CAN_scenarios(''fault_crc'')     — falha CRC\n');
fprintf('  AEB_CAN_scenarios(''fault_alive'')   — alive travado\n');
fprintf('  AEB_CAN_scenarios(''fault_dropout'') — perda de mensagem\n');
fprintf('  AEB_CAN_plot                        — plotar resultados\n\n');

%% ══════════════════════════════════════════════════════════════════════════════
%% Funções locais
%% ══════════════════════════════════════════════════════════════════════════════

function bld_tx_brakecmd(sys)
%BLD_TX_BRAKECMD  TX_BrakeCmd: codifica AEB_BrakeCmd 0x080 (10ms, ASIL-B)
% Entradas: brake_req(1), brake_press_bar(2), brake_mode(3)
% Saídas:   bc_req_r(1), bc_press_r(2), bc_mode_r(3), alive(4), crc(5)
    p = [sys '/TX_BrakeCmd'];
    add_block('built-in/Subsystem', p);
    del_defaults(p);
    in_names = {'brake_req','brake_press','brake_mode'};
    for k=1:3
        add_block('built-in/Inport',[p '/' in_names{k}],'Port',num2str(k),...
            'Position',[30,40+60*(k-1),60,55+60*(k-1)]);
    end
    add_block('simulink/User-Defined Functions/MATLAB Function',[p '/Encode_BC'],...
        'Position',[120,20,290,210]);
    out_names = {'bc_req_r','bc_press_r','bc_mode_r','alive','crc'};
    for k=1:5
        add_block('built-in/Outport',[p '/' out_names{k}],'Port',num2str(k),...
            'Position',[380,35+45*(k-1),410,50+45*(k-1)]);
    end
    for k=1:3
        add_line(p,[in_names{k} '/1'],['Encode_BC/' num2str(k)],'autorouting','on');
    end
    for k=1:5
        add_line(p,['Encode_BC/' num2str(k)],[out_names{k} '/1'],'autorouting','on');
    end
    code = [...
        'function [bc_req_r, bc_press_r, bc_mode_r, alive, crc] = Encode_BC(brake_req, brake_press, brake_mode)\n'...
        '%% AEB_BrakeCmd encoder — DBC ID 0x080, periodo 10ms, ASIL-B\n'...
        '%% BrakeRequest :0|1@1+  (1,0)   | BrakePressure:1|15@1+ (0.1,0)\n'...
        '%% BrakeMode    :16|3@1+ (1,0)   | AliveCounter:24|4     | CRC:28|4\n'...
        '%% CRC-4 = (alive XOR brake_req XOR brake_mode) AND 0x0F\n'...
        'persistent alive_cnt;\n'...
        'if isempty(alive_cnt), alive_cnt = uint8(0); end\n'...
        '\n'...
        '%% Codificacao DBC: physical -> raw integer\n'...
        'bc_req_r   = double(brake_req > 0);\n'...
        'bc_press_r = double(min(round(brake_press / 0.1), 32767));\n'...
        'bc_mode_r  = double(min(max(round(brake_mode), 0), 5));\n'...
        '\n'...
        '%% AliveCounter: contador rolante de 4 bits, modulo 16\n'...
        'alive = double(alive_cnt);\n'...
        'alive_cnt = bitand(alive_cnt + uint8(1), uint8(15));\n'...
        '\n'...
        '%% CRC-4: aeb_controller_node.cpp compute_crc4(b0=alive, b1=req, b2=mode)\n'...
        'crc = double(bitand(bitxor(bitxor(uint8(alive),uint8(bc_req_r)),uint8(bc_mode_r)),uint8(15)));\n'...
    ];
    set_mlfn_code(p, 'Encode_BC', code);
end

function bld_tx_egovehicle(sys)
%BLD_TX_EGOVEHICLE  TX_EgoVehicle: codifica AEB_EgoVehicle 0x100 (10ms)
% Entradas: v_ego(1), a_ego(2)
% Saídas:   spd_r(1), accel_r(2), yaw_r(3), steer_r(4)
    p = [sys '/TX_EgoVehicle'];
    add_block('built-in/Subsystem', p);
    del_defaults(p);
    in_names = {'v_ego','a_ego'};
    for k=1:2
        add_block('built-in/Inport',[p '/' in_names{k}],'Port',num2str(k),...
            'Position',[30,40+70*(k-1),60,55+70*(k-1)]);
    end
    add_block('simulink/User-Defined Functions/MATLAB Function',[p '/Encode_Ego'],...
        'Position',[120,20,290,170]);
    out_names = {'spd_r','accel_r','yaw_r','steer_r'};
    for k=1:4
        add_block('built-in/Outport',[p '/' out_names{k}],'Port',num2str(k),...
            'Position',[380,35+40*(k-1),410,50+40*(k-1)]);
    end
    for k=1:2
        add_line(p,[in_names{k} '/1'],['Encode_Ego/' num2str(k)],'autorouting','on');
    end
    for k=1:4
        add_line(p,['Encode_Ego/' num2str(k)],[out_names{k} '/1'],'autorouting','on');
    end
    code = [...
        'function [spd_r, accel_r, yaw_r, steer_r] = Encode_Ego(v_ego, a_ego)\n'...
        '%% AEB_EgoVehicle encoder — DBC ID 0x100, periodo 10ms\n'...
        '%% VehicleSpeed:0|16@1+  (0.01,0)      | LongAccel:16|16@1+ (0.001,-32)\n'...
        '%% YawRate:32|16@1+      (0.01,-327.68) | SteeringAngle:48|16@1+ (0.1,-3276.8)\n'...
        'spd_r   = double(min(round(v_ego / 0.01), 65535));\n'...
        'accel_r = double(min(max(round((a_ego + 32) / 0.001), 0), 65535));\n'...
        'yaw_r   = double(round((0 + 327.68) / 0.01));   %% yaw = 0 (simulacao)\n'...
        'steer_r = double(round((0 + 3276.8) / 0.1));    %% steer = 0 (simulacao)\n'...
    ];
    set_mlfn_code(p, 'Encode_Ego', code);
end

function bld_tx_radartarget(sys)
%BLD_TX_RADARTARGET  TX_RadarTarget: codifica AEB_RadarTarget 0x120 (20ms)
% Entradas: distance(1), v_rel(2), ttc(3), conf(4)
% Saídas:   dist_r(1), spd_r(2), ttc_r(3), conf_r(4), alive(5)
    p = [sys '/TX_RadarTarget'];
    add_block('built-in/Subsystem', p);
    del_defaults(p);
    in_names = {'distance','v_rel','ttc','conf'};
    for k=1:4
        add_block('built-in/Inport',[p '/' in_names{k}],'Port',num2str(k),...
            'Position',[30,40+60*(k-1),60,55+60*(k-1)]);
    end
    add_block('simulink/User-Defined Functions/MATLAB Function',[p '/Encode_RT'],...
        'Position',[120,20,290,270]);
    out_names = {'dist_r','spd_r','ttc_r','conf_r','alive_rt'};
    for k=1:5
        add_block('built-in/Outport',[p '/' out_names{k}],'Port',num2str(k),...
            'Position',[380,35+45*(k-1),410,50+45*(k-1)]);
    end
    for k=1:4
        add_line(p,[in_names{k} '/1'],['Encode_RT/' num2str(k)],'autorouting','on');
    end
    for k=1:5
        add_line(p,['Encode_RT/' num2str(k)],[out_names{k} '/1'],'autorouting','on');
    end
    code = [...
        'function [dist_r, spd_r, ttc_r, conf_r, alive_rt] = Encode_RT(distance, v_rel, ttc, conf)\n'...
        '%% AEB_RadarTarget encoder — DBC ID 0x120, periodo 20ms\n'...
        '%% TargetDistance:0|16@1+ (0.01,0) | RelativeSpeed:16|16@1+ (0.01,-327.68)\n'...
        '%% TTC:32|16@1+ (0.001,0)          | Confidence:48|8@1+ (1,0)\n'...
        'persistent alive_cnt_rt;\n'...
        'if isempty(alive_cnt_rt), alive_cnt_rt = uint8(0); end\n'...
        '\n'...
        'dist_r = double(min(round(distance / 0.01), 65535));\n'...
        'spd_r  = double(min(max(round((v_rel + 327.68) / 0.01), 0), 65535));\n'...
        'ttc_r  = double(min(round(ttc / 0.001), 65535));\n'...
        'conf_r = double(min(max(round(conf), 0), 15));\n'...
        '\n'...
        '%% AliveCounter 4 bits, modulo 16\n'...
        'alive_rt = double(alive_cnt_rt);\n'...
        'alive_cnt_rt = bitand(alive_cnt_rt + uint8(1), uint8(15));\n'...
    ];
    set_mlfn_code(p, 'Encode_RT', code);
end

function bld_tx_fsmstate(sys)
%BLD_TX_FSMSTATE  TX_FSMState: codifica AEB_FSMState 0x200 (50ms)
% Entradas: fsm(1), alert_level(2), brake_active(3), ttc_thr(4)
% Saídas:   fsm_r(1), alert_r(2), brake_r(3), ttc_thr_r(4)
    p = [sys '/TX_FSMState'];
    add_block('built-in/Subsystem', p);
    del_defaults(p);
    in_names = {'fsm','alert_level','brake_active','ttc_thr'};
    for k=1:4
        add_block('built-in/Inport',[p '/' in_names{k}],'Port',num2str(k),...
            'Position',[30,40+55*(k-1),60,55+55*(k-1)]);
    end
    add_block('simulink/User-Defined Functions/MATLAB Function',[p '/Encode_FSM'],...
        'Position',[120,20,290,250]);
    out_names = {'fsm_r','alert_r','brake_r','ttc_thr_r'};
    for k=1:4
        add_block('built-in/Outport',[p '/' out_names{k}],'Port',num2str(k),...
            'Position',[380,35+45*(k-1),410,50+45*(k-1)]);
    end
    for k=1:4
        add_line(p,[in_names{k} '/1'],['Encode_FSM/' num2str(k)],'autorouting','on');
    end
    for k=1:4
        add_line(p,['Encode_FSM/' num2str(k)],[out_names{k} '/1'],'autorouting','on');
    end
    code = [...
        'function [fsm_r, alert_r, brake_r, ttc_thr_r] = Encode_FSM(fsm, alert_level, brake_active, ttc_thr)\n'...
        '%% AEB_FSMState encoder — DBC ID 0x200, periodo 50ms\n'...
        '%% FSMState:0|8@1+ (1,0) | AlertLevel:8|8@1+ (1,0)\n'...
        '%% BrakeActive:16|8@1+   | TTCThreshold:24|8@1+ (0.1,0)\n'...
        'fsm_r     = double(min(max(round(fsm), 0), 6));\n'...
        'alert_r   = double(min(max(round(alert_level), 0), 3));\n'...
        'brake_r   = double(brake_active > 0);\n'...
        'ttc_thr_r = double(min(round(ttc_thr / 0.1), 255));\n'...
    ];
    set_mlfn_code(p, 'Encode_FSM', code);
end

function bld_tx_alert(sys)
%BLD_TX_ALERT  TX_Alert: codifica AEB_Alert 0x300 (evento)
% Entradas: alert_type(1), alert_active(2), buzzer_cmd(3)
% Saídas:   atype_r(1), aactive_r(2), buzzer_r(3)
    p = [sys '/TX_Alert'];
    add_block('built-in/Subsystem', p);
    del_defaults(p);
    in_names = {'alert_type','alert_active','buzzer_cmd'};
    for k=1:3
        add_block('built-in/Inport',[p '/' in_names{k}],'Port',num2str(k),...
            'Position',[30,40+60*(k-1),60,55+60*(k-1)]);
    end
    add_block('simulink/User-Defined Functions/MATLAB Function',[p '/Encode_Alert'],...
        'Position',[120,20,290,200]);
    out_names = {'atype_r','aactive_r','buzzer_r'};
    for k=1:3
        add_block('built-in/Outport',[p '/' out_names{k}],'Port',num2str(k),...
            'Position',[380,35+50*(k-1),410,50+50*(k-1)]);
    end
    for k=1:3
        add_line(p,[in_names{k} '/1'],['Encode_Alert/' num2str(k)],'autorouting','on');
    end
    for k=1:3
        add_line(p,['Encode_Alert/' num2str(k)],[out_names{k} '/1'],'autorouting','on');
    end
    code = [...
        'function [atype_r, aactive_r, buzzer_r] = Encode_Alert(alert_type, alert_active, buzzer_cmd)\n'...
        '%% AEB_Alert encoder — DBC ID 0x300, event-driven\n'...
        '%% AlertType:0|8@1+ (1,0) | AlertActive:8|1@1+ | BuzzerCmd:9|3@1+\n'...
        'atype_r   = double(min(max(round(alert_type), 0), 3));\n'...
        'aactive_r = double(alert_active > 0);\n'...
        'buzzer_r  = double(min(max(round(buzzer_cmd), 0), 7));\n'...
    ];
    set_mlfn_code(p, 'Encode_Alert', code);
end

function bld_fault_inj(sys)
%BLD_FAULT_INJ  Injecao_Falhas: injeta falhas no AEB_BrakeCmd (ASIL-B)
% Entradas: bc_req_r(1), bc_press_r(2), bc_mode_r(3), alive(4), crc(5),
%           f_drop(6), f_crc(7), f_alive(8)
% Saídas:   bc_req_rx(1), bc_press_rx(2), bc_mode_rx(3), alive_rx(4), crc_rx(5), valid(6)
    p = [sys '/Injecao_Falhas'];
    add_block('built-in/Subsystem', p);
    del_defaults(p);
    in_names = {'bc_req_r','bc_press_r','bc_mode_r','alive','crc','f_drop','f_crc','f_alive'};
    for k=1:8
        add_block('built-in/Inport',[p '/' in_names{k}],'Port',num2str(k),...
            'Position',[30,30+50*(k-1),60,45+50*(k-1)]);
    end
    add_block('simulink/User-Defined Functions/MATLAB Function',[p '/Fault_Inj'],...
        'Position',[130,150,310,430]);
    out_names = {'bc_req_rx','bc_press_rx','bc_mode_rx','alive_rx','crc_rx','valid'};
    for k=1:6
        add_block('built-in/Outport',[p '/' out_names{k}],'Port',num2str(k),...
            'Position',[410,35+55*(k-1),440,50+55*(k-1)]);
    end
    for k=1:8
        add_line(p,[in_names{k} '/1'],['Fault_Inj/' num2str(k)],'autorouting','on');
    end
    for k=1:6
        add_line(p,['Fault_Inj/' num2str(k)],[out_names{k} '/1'],'autorouting','on');
    end
    code = [...
        'function [bc_req_rx, bc_press_rx, bc_mode_rx, alive_rx, crc_rx, valid] = ...\n'...
        '    Fault_Inj(bc_req_r, bc_press_r, bc_mode_r, alive, crc, f_drop, f_crc, f_alive)\n'...
        '%% Injecao de falhas no AEB_BrakeCmd (mensagem ASIL-B)\n'...
        '%%  f_drop  = 1: dropout (supressao do frame, valid=0)\n'...
        '%%  f_crc   = 1: corrupcao de CRC (XOR 0x0F — DTC C1004)\n'...
        '%%  f_alive = 1: AliveCounter congelado (nao incrementa — DTC C1005)\n'...
        'persistent frozen_alive;\n'...
        'if isempty(frozen_alive), frozen_alive = 0; end\n'...
        '\n'...
        'if f_drop > 0\n'...
        '    bc_req_rx = 0;  bc_press_rx = 0;  bc_mode_rx = 0;\n'...
        '    alive_rx  = frozen_alive;  crc_rx = 0;  valid = 0;\n'...
        '    return;\n'...
        'end\n'...
        'bc_req_rx  = bc_req_r;\n'...
        'bc_press_rx = bc_press_r;\n'...
        'bc_mode_rx = bc_mode_r;\n'...
        'valid      = 1;\n'...
        '\n'...
        'if f_alive > 0\n'...
        '    alive_rx = frozen_alive;  %% trava o contador\n'...
        'else\n'...
        '    frozen_alive = alive;\n'...
        '    alive_rx = alive;\n'...
        'end\n'...
        '\n'...
        'if f_crc > 0\n'...
        '    crc_rx = double(bitxor(uint8(crc), uint8(15)));  %% inverte 4 bits\n'...
        'else\n'...
        '    crc_rx = crc;\n'...
        'end\n'...
    ];
    set_mlfn_code(p, 'Fault_Inj', code);
end

function bld_rx_brakecmd(sys)
%BLD_RX_BRAKECMD  RX_BrakeCmd: decodifica e verifica AEB_BrakeCmd 0x080
% Entradas: bc_req(1), bc_press(2), bc_mode(3), alive(4), crc(5), valid(6)
% Saídas: brake_req_d(1), brake_press_d(2), brake_mode_d(3),
%         crc_err(4), alive_err(5), timeout_err(6)
    p = [sys '/RX_BrakeCmd'];
    add_block('built-in/Subsystem', p);
    del_defaults(p);
    in_names = {'bc_req','bc_press','bc_mode','alive','crc','valid'};
    for k=1:6
        add_block('built-in/Inport',[p '/' in_names{k}],'Port',num2str(k),...
            'Position',[30,30+50*(k-1),60,45+50*(k-1)]);
    end
    add_block('simulink/User-Defined Functions/MATLAB Function',[p '/Decode_BC'],...
        'Position',[130,80,310,380]);
    out_names = {'brake_req_d','brake_press_d','brake_mode_d','crc_err','alive_err','timeout_err'};
    for k=1:6
        add_block('built-in/Outport',[p '/' out_names{k}],'Port',num2str(k),...
            'Position',[410,35+50*(k-1),440,50+50*(k-1)]);
    end
    for k=1:6
        add_line(p,[in_names{k} '/1'],['Decode_BC/' num2str(k)],'autorouting','on');
    end
    for k=1:6
        add_line(p,['Decode_BC/' num2str(k)],[out_names{k} '/1'],'autorouting','on');
    end
    code = [...
        'function [brake_req_d, brake_press_d, brake_mode_d, crc_err, alive_err, timeout_err] = ...\n'...
        '    Decode_BC(bc_req, bc_press, bc_mode, alive, crc, valid)\n'...
        '%% RX_BrakeCmd: decodificacao DBC + verificacao CRC-4 + AliveCounter + timeout\n'...
        '%% Detecta: C1004 (CRC incorreto), C1005 (alive travado), C1010 (timeout)\n'...
        'persistent last_alive timeout_cnt;\n'...
        'if isempty(last_alive),  last_alive  = -1; end\n'...
        'if isempty(timeout_cnt), timeout_cnt = 0;  end\n'...
        '\n'...
        'if ~valid\n'...
        '    timeout_cnt = timeout_cnt + 1;\n'...
        '    brake_req_d  = 0;  brake_press_d = 0;  brake_mode_d = 0;\n'...
        '    crc_err = 0;  alive_err = 0;\n'...
        '    timeout_err = double(timeout_cnt > 10);\n'...
        '    return;\n'...
        'end\n'...
        'timeout_cnt = 0;  timeout_err = 0;\n'...
        '\n'...
        '%% Verificacao CRC-4\n'...
        'exp_crc = bitand(bitxor(bitxor(uint8(alive),uint8(bc_req)),uint8(bc_mode)),uint8(15));\n'...
        'crc_err = double(double(exp_crc) ~= crc);\n'...
        '\n'...
        '%% Verificacao AliveCounter: deve incrementar +1 mod 16\n'...
        'if last_alive < 0\n'...
        '    alive_err = 0;\n'...
        'else\n'...
        '    alive_err = double(alive ~= mod(last_alive + 1, 16));\n'...
        'end\n'...
        'last_alive = alive;\n'...
        '\n'...
        '%% Decodificacao DBC (raw -> physical)\n'...
        'brake_req_d   = double(bc_req > 0);\n'...
        'brake_press_d = bc_press * 0.1;\n'...
        'brake_mode_d  = bc_mode;\n'...
    ];
    set_mlfn_code(p, 'Decode_BC', code);
end

function bld_rx_egovehicle(sys)
%BLD_RX_EGOVEHICLE  RX_EgoVehicle: decodifica AEB_EgoVehicle 0x100
% Entradas: spd_r(1), accel_r(2), yaw_r(3), steer_r(4)
% Saídas: v_ego_d(1), a_ego_d(2), yaw_d(3), steer_d(4)
    p = [sys '/RX_EgoVehicle'];
    add_block('built-in/Subsystem', p);
    del_defaults(p);
    in_names = {'spd_r','accel_r','yaw_r','steer_r'};
    for k=1:4
        add_block('built-in/Inport',[p '/' in_names{k}],'Port',num2str(k),...
            'Position',[30,40+55*(k-1),60,55+55*(k-1)]);
    end
    add_block('simulink/User-Defined Functions/MATLAB Function',[p '/Decode_Ego'],...
        'Position',[120,20,290,250]);
    out_names = {'v_ego_d','a_ego_d','yaw_d','steer_d'};
    for k=1:4
        add_block('built-in/Outport',[p '/' out_names{k}],'Port',num2str(k),...
            'Position',[380,35+45*(k-1),410,50+45*(k-1)]);
    end
    for k=1:4
        add_line(p,[in_names{k} '/1'],['Decode_Ego/' num2str(k)],'autorouting','on');
    end
    for k=1:4
        add_line(p,['Decode_Ego/' num2str(k)],[out_names{k} '/1'],'autorouting','on');
    end
    code = [...
        'function [v_ego_d, a_ego_d, yaw_d, steer_d] = Decode_Ego(spd_r, accel_r, yaw_r, steer_r)\n'...
        '%% AEB_EgoVehicle decoder — DBC ID 0x100, 10ms\n'...
        'v_ego_d = spd_r   * 0.01;\n'...
        'a_ego_d = accel_r * 0.001 - 32;\n'...
        'yaw_d   = yaw_r   * 0.01  - 327.68;\n'...
        'steer_d = steer_r * 0.1   - 3276.8;\n'...
    ];
    set_mlfn_code(p, 'Decode_Ego', code);
end

function bld_rx_radartarget(sys)
%BLD_RX_RADARTARGET  RX_RadarTarget: decodifica AEB_RadarTarget 0x120
% Entradas: dist_r(1), spd_r(2), ttc_r(3), conf_r(4), alive_rt(5)
% Saídas: dist_d(1), vrel_d(2), ttc_d(3), conf_d(4), alive_err_rt(5)
    p = [sys '/RX_RadarTarget'];
    add_block('built-in/Subsystem', p);
    del_defaults(p);
    in_names = {'dist_r','spd_r','ttc_r','conf_r','alive_rt'};
    for k=1:5
        add_block('built-in/Inport',[p '/' in_names{k}],'Port',num2str(k),...
            'Position',[30,40+55*(k-1),60,55+55*(k-1)]);
    end
    add_block('simulink/User-Defined Functions/MATLAB Function',[p '/Decode_RT'],...
        'Position',[120,20,290,300]);
    out_names = {'dist_d','vrel_d','ttc_d','conf_d','alive_err_rt'};
    for k=1:5
        add_block('built-in/Outport',[p '/' out_names{k}],'Port',num2str(k),...
            'Position',[380,35+50*(k-1),410,50+50*(k-1)]);
    end
    for k=1:5
        add_line(p,[in_names{k} '/1'],['Decode_RT/' num2str(k)],'autorouting','on');
    end
    for k=1:5
        add_line(p,['Decode_RT/' num2str(k)],[out_names{k} '/1'],'autorouting','on');
    end
    code = [...
        'function [dist_d, vrel_d, ttc_d, conf_d, alive_err_rt] = Decode_RT(dist_r, spd_r, ttc_r, conf_r, alive_rt)\n'...
        '%% AEB_RadarTarget decoder — DBC ID 0x120, 20ms\n'...
        'persistent last_alive_rt;\n'...
        'if isempty(last_alive_rt), last_alive_rt = -1; end\n'...
        '\n'...
        'dist_d = dist_r * 0.01;\n'...
        'vrel_d = spd_r  * 0.01 - 327.68;\n'...
        'ttc_d  = ttc_r  * 0.001;\n'...
        'conf_d = conf_r;\n'...
        '\n'...
        'if last_alive_rt < 0\n'...
        '    alive_err_rt = 0;\n'...
        'else\n'...
        '    alive_err_rt = double(alive_rt ~= mod(last_alive_rt + 1, 16));\n'...
        'end\n'...
        'last_alive_rt = alive_rt;\n'...
    ];
    set_mlfn_code(p, 'Decode_RT', code);
end

function bld_rx_fsmstate(sys)
%BLD_RX_FSMSTATE  RX_FSMState: decodifica AEB_FSMState 0x200
% Entradas: fsm_r(1), alert_r(2), brake_r(3), ttc_thr_r(4)
% Saídas: fsm_d(1), alert_d(2), brake_d(3), ttc_thr_d(4)
    p = [sys '/RX_FSMState'];
    add_block('built-in/Subsystem', p);
    del_defaults(p);
    in_names = {'fsm_r','alert_r','brake_r','ttc_thr_r'};
    for k=1:4
        add_block('built-in/Inport',[p '/' in_names{k}],'Port',num2str(k),...
            'Position',[30,40+55*(k-1),60,55+55*(k-1)]);
    end
    add_block('simulink/User-Defined Functions/MATLAB Function',[p '/Decode_FSM'],...
        'Position',[120,20,290,250]);
    out_names = {'fsm_d','alert_d','brake_d','ttc_thr_d'};
    for k=1:4
        add_block('built-in/Outport',[p '/' out_names{k}],'Port',num2str(k),...
            'Position',[380,35+45*(k-1),410,50+45*(k-1)]);
    end
    for k=1:4
        add_line(p,[in_names{k} '/1'],['Decode_FSM/' num2str(k)],'autorouting','on');
    end
    for k=1:4
        add_line(p,['Decode_FSM/' num2str(k)],[out_names{k} '/1'],'autorouting','on');
    end
    code = [...
        'function [fsm_d, alert_d, brake_d, ttc_thr_d] = Decode_FSM(fsm_r, alert_r, brake_r, ttc_thr_r)\n'...
        '%% AEB_FSMState decoder — DBC ID 0x200, 50ms\n'...
        'fsm_d     = fsm_r;\n'...
        'alert_d   = alert_r;\n'...
        'brake_d   = brake_r;\n'...
        'ttc_thr_d = ttc_thr_r * 0.1;\n'...
    ];
    set_mlfn_code(p, 'Decode_FSM', code);
end

function bld_rx_alert(sys)
%BLD_RX_ALERT  RX_Alert: decodifica AEB_Alert 0x300
% Entradas: atype_r(1), aactive_r(2), buzzer_r(3)
% Saídas: alert_type_d(1), alert_active_d(2), buzzer_d(3)
    p = [sys '/RX_Alert'];
    add_block('built-in/Subsystem', p);
    del_defaults(p);
    in_names = {'atype_r','aactive_r','buzzer_r'};
    for k=1:3
        add_block('built-in/Inport',[p '/' in_names{k}],'Port',num2str(k),...
            'Position',[30,40+60*(k-1),60,55+60*(k-1)]);
    end
    add_block('simulink/User-Defined Functions/MATLAB Function',[p '/Decode_Alert'],...
        'Position',[120,20,290,200]);
    out_names = {'alert_type_d','alert_active_d','buzzer_d'};
    for k=1:3
        add_block('built-in/Outport',[p '/' out_names{k}],'Port',num2str(k),...
            'Position',[380,35+50*(k-1),410,50+50*(k-1)]);
    end
    for k=1:3
        add_line(p,[in_names{k} '/1'],['Decode_Alert/' num2str(k)],'autorouting','on');
    end
    for k=1:3
        add_line(p,['Decode_Alert/' num2str(k)],[out_names{k} '/1'],'autorouting','on');
    end
    code = [...
        'function [alert_type_d, alert_active_d, buzzer_d] = Decode_Alert(atype_r, aactive_r, buzzer_r)\n'...
        '%% AEB_Alert decoder — DBC ID 0x300, event-driven\n'...
        'alert_type_d   = atype_r;\n'...
        'alert_active_d = aactive_r;\n'...
        'buzzer_d       = buzzer_r;\n'...
    ];
    set_mlfn_code(p, 'Decode_Alert', code);
end

function bld_diagnosticos(sys)
%BLD_DIAGNOSTICOS  Diagnosticos: debounce e confirmacao de DTCs
% Entradas: crc_err(1), alive_err(2), timeout_err(3)
% Saídas: dtc_c1004(1), dtc_c1005(2), dtc_c1010(3), err_count(4)
    p = [sys '/Diagnosticos'];
    add_block('built-in/Subsystem', p);
    del_defaults(p);
    in_names = {'crc_err','alive_err','timeout_err'};
    for k=1:3
        add_block('built-in/Inport',[p '/' in_names{k}],'Port',num2str(k),...
            'Position',[30,40+60*(k-1),60,55+60*(k-1)]);
    end
    add_block('simulink/User-Defined Functions/MATLAB Function',[p '/DTC_Logic'],...
        'Position',[120,20,290,220]);
    out_names = {'dtc_c1004','dtc_c1005','dtc_c1010','err_count'};
    for k=1:4
        add_block('built-in/Outport',[p '/' out_names{k}],'Port',num2str(k),...
            'Position',[380,35+45*(k-1),410,50+45*(k-1)]);
    end
    for k=1:3
        add_line(p,[in_names{k} '/1'],['DTC_Logic/' num2str(k)],'autorouting','on');
    end
    for k=1:4
        add_line(p,['DTC_Logic/' num2str(k)],[out_names{k} '/1'],'autorouting','on');
    end
    code = [...
        'function [dtc_c1004, dtc_c1005, dtc_c1010, err_count] = DTC_Logic(crc_err, alive_err, timeout_err)\n'...
        '%% Diagnosticos CAN — debounce 3 ciclos (SENSOR_FAULT_CYCLES=3, aeb_config.h)\n'...
        '%%  C1004: Erro de CRC no AEB_BrakeCmd\n'...
        '%%  C1005: AliveCounter travado\n'...
        '%%  C1010: Timeout de mensagem\n'...
        'persistent cnt_crc cnt_alive cnt_tmo;\n'...
        'if isempty(cnt_crc), cnt_crc=0; cnt_alive=0; cnt_tmo=0; end\n'...
        '\n'...
        'if crc_err,     cnt_crc   = min(cnt_crc + 1, 5);\n'...
        'else            cnt_crc   = max(cnt_crc - 1, 0); end\n'...
        'if alive_err,   cnt_alive = min(cnt_alive + 1, 5);\n'...
        'else            cnt_alive = max(cnt_alive - 1, 0); end\n'...
        'if timeout_err, cnt_tmo   = min(cnt_tmo + 1, 5);\n'...
        'else            cnt_tmo   = max(cnt_tmo - 1, 0); end\n'...
        '\n'...
        'dtc_c1004 = double(cnt_crc   >= 3);\n'...
        'dtc_c1005 = double(cnt_alive >= 3);\n'...
        'dtc_c1010 = double(cnt_tmo   >= 3);\n'...
        'err_count = double(cnt_crc + cnt_alive + cnt_tmo);\n'...
    ];
    set_mlfn_code(p, 'DTC_Logic', code);
end

%% ── Auxiliares ───────────────────────────────────────────────────────────────

function del_defaults(p)
% Remove Inport/Outport e linha defaults criados pelo add_block Subsystem
    lns = find_system(p,'FindAll','on','SearchDepth',1,'type','line');
    for k = 1:numel(lns), delete_line(lns(k)); end
    blks = find_system(p,'SearchDepth',1,'Type','Block');
    for k = 1:numel(blks)
        if strcmp(blks{k}, p), continue; end
        bt = get_param(blks{k},'BlockType');
        if any(strcmp(bt,{'Inport','Outport'}))
            delete_block(blks{k});
        end
    end
end

function set_mlfn_code(subsys, block_name, code_str)
% Define o Script de um bloco MATLAB Function via sfroot()
    full_path = [subsys '/' block_name];
    rt = sfroot();
    charts = rt.find('-isa','Stateflow.EMChart');
    chart = [];
    for k = 1:numel(charts)
        if strcmp(charts(k).Path, full_path)
            chart = charts(k);
            break;
        end
    end
    if isempty(chart)
        warning('MATLAB Function nao encontrado: %s', full_path);
        return;
    end
    chart.Script = sprintf(code_str);
end
