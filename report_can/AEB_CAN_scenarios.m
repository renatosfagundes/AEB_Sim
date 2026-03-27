function AEB_CAN_scenarios(scenario)
%AEB_CAN_SCENARIOS  Configura cenário de simulação para AEB_CAN.slx
%
% Uso:
%   AEB_CAN_scenarios('nominal')        % CCRs 50 km/h, sem falhas
%   AEB_CAN_scenarios('fault_crc')      % CRC corrompido em t=4s (DTC C1004)
%   AEB_CAN_scenarios('fault_alive')    % AliveCounter travado em t=4.5s (DTC C1005)
%   AEB_CAN_scenarios('fault_dropout')  % Dropout 100ms em t=3.5s (DTC C1010)
%   AEB_CAN_scenarios('fault_combined') % CRC + dropout simultâneos
%   AEB_CAN_scenarios('ccrs_30')        % CCRs 30 km/h (30 km/h = 8.33 m/s)
%   AEB_CAN_scenarios('high_speed')     % 60 km/h (limite V_EGO_MAX)
%
%   out = sim('AEB_CAN');               % simular após configurar cenário

if nargin < 1, scenario = 'nominal'; end

t = (0:0.001:5)';
N = length(t);

%% ── Dinâmica base por cenário ────────────────────────────────────────────────
switch scenario
    case 'ccrs_30'
        v0 = 8.33;   d0 = 30; decel = 4.0; t_brake = 1.5;
    case 'high_speed'
        v0 = 16.67;  d0 = 60; decel = 4.0; t_brake = 2.5;
    otherwise  % nominal + todas as variantes de falha usam mesma dinâmica
        v0 = 13.89;  d0 = 50; decel = 4.0; t_brake = 2.0;
end

v_ego = max(0, v0 - decel * max(t - t_brake, 0));
d_tgt = max(0.5, d0 - cumtrapz(t, v_ego));
v_rel = v_ego;
ttc   = min(20, d_tgt ./ max(v_rel, 0.01));
a_ego = [0; diff(v_ego)/0.001];

% FSM
fsm = ones(N,1);
fsm(ttc <= 4.0) = 2; fsm(ttc <= 3.0) = 3;
fsm(ttc <= 2.2) = 4; fsm(ttc <= 1.8) = 5;
fsm(v_ego < 0.1) = 1;

bpress = zeros(N,1);
bpress(fsm==3)=2.0; bpress(fsm==4)=4.0; bpress(fsm==5)=6.0;
breq  = double(bpress > 0);
bmode = max(0, fsm - 1);

alevel = zeros(N,1);
alevel(fsm==2)=1; alevel(fsm==3)=2; alevel(fsm>=4)=3;
atype  = double(fsm >= 2) * 3;
aactive= double(fsm >= 2);
buzzer = zeros(N,1);
buzzer(fsm==2)=1; buzzer(fsm==3)=2; buzzer(fsm==4)=4; buzzer(fsm==5)=3;
ttc_thr= 4.0*ones(N,1);
ttc_thr(fsm==3)=3.0; ttc_thr(fsm==4)=2.2; ttc_thr(fsm==5)=1.8;
conf = 15*ones(N,1);

%% ── Perfis de falha ──────────────────────────────────────────────────────────
fault_dropout = zeros(N,1);
fault_crc     = zeros(N,1);
fault_alive   = zeros(N,1);

switch scenario
    case 'fault_dropout'
        fault_dropout = double(t >= 3.5 & t < 3.6);
    case 'fault_crc'
        fault_crc = double(t >= 4.0 & t < 4.05);
    case 'fault_alive'
        fault_alive = double(t >= 4.5 & t < 4.7);
    case 'fault_combined'
        fault_crc     = double(t >= 3.8 & t < 3.85);
        fault_dropout = double(t >= 4.0 & t < 4.1);
        fault_alive   = double(t >= 4.5 & t < 4.65);
    % 'nominal', 'ccrs_30', 'high_speed': sem falhas (já zeros)
end

%% ── Exportar para workspace base ─────────────────────────────────────────────
vars = {'breq','bpress','bmode','v_ego','a_ego','d_tgt','v_rel','ttc','fsm',...
        'conf','alevel','atype','aactive','buzzer','ttc_thr',...
        'fault_dropout','fault_crc','fault_alive'};
for k = 1:numel(vars)
    assignin('base', ['ws_' vars{k}], [t, eval(vars{k})]);
end

fprintf('Cenario ''%s'' configurado. Execute: out = sim(''AEB_CAN'')\n', scenario);
end
