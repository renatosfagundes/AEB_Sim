function AEB_Controller_scenarios(scenario)
%AEB_CONTROLLER_SCENARIOS  Configura cenário de simulação para AEB_Controller.slx
%
% Uso:
%   AEB_Controller_scenarios('ccrs_50')       % CCRs 50 km/h (padrão Euro NCAP)
%   AEB_Controller_scenarios('ccrs_30')       % CCRs 30 km/h
%   AEB_Controller_scenarios('ccrm')          % CCRm — alvo em movimento (20 km/h)
%   AEB_Controller_scenarios('override')      % Intervenção do motorista em t=3s
%   AEB_Controller_scenarios('fault_sensor')  % Perda de confiança em t=3.5s
%   AEB_Controller_scenarios('aeb_disabled')  % AEB desabilitado (flag=0)
%
%   out = sim('AEB_Controller');              % simular após configurar cenário

if nargin < 1, scenario = 'ccrs_50'; end

DT   = 0.001;
TEND = 10.0;
t    = (0:DT:TEND)';
N    = length(t);

%% ── Parâmetros AEB (aeb_config.h) ────────────────────────────────────────────
TTC_W  = 4.0;  TTC_L1 = 3.0;  TTC_L2 = 2.2;  TTC_L3 = 1.8;
D_L1   = 20.0; D_L2   = 10.0; D_L3   = 5.0;
DECEL_L3 = 6.0;
V_MIN  = 1.39; V_MAX  = 16.67;
WARN_MIN = 0.8;  HYST = 0.2;  POST = 2.0;
V_REL_MIN = 0.5;

%% ── Dinâmica do cenário ───────────────────────────────────────────────────────
switch scenario
    case 'ccrs_30'
        v0 = 8.33;   v_tgt0 = 0;     d0 = 30;  decel_env = 4.0;
        t_start = 1.0;
    case 'ccrm'
        % CCRm: alvo se move a 20 km/h (5.56 m/s), ego a 50 km/h
        v0 = 13.89;  v_tgt0 = 5.56;  d0 = 50;  decel_env = 3.5;
        t_start = 1.0;
    otherwise  % ccrs_50, override, fault_sensor, aeb_disabled
        v0 = 13.89;  v_tgt0 = 0;     d0 = 50;  decel_env = 4.0;
        t_start = 1.0;
end

%% ── Sinais de percepção (pré-computados com física + FSM) ────────────────────
v_ego  = zeros(N,1);
v_tgt  = zeros(N,1);
v_ego(1) = v0;
v_tgt(1) = v_tgt0;
d_tgt = d0 * ones(N,1);
a_ego = zeros(N,1);
conf  = 15 * ones(N,1);
aeb_en = ones(N,1);
steer  = zeros(N,1);
bpedal = zeros(N,1);

% FSM state para pré-computação da dinâmica
st = 1;  % STANDBY
warn_acc = 0;  deb_t = 0;  pb_t = 0;

for k = 2:N
    tk = t(k);

    % ── Cenário override: pedal de freio a partir de t=3s ──────────────────
    if strcmp(scenario,'override') && tk >= 3.0 && tk < 4.0
        bpedal(k) = 1;
    end

    % ── Cenário fault_sensor: confiança cai para 0 em t=3.5–4.5s ──────────
    if strcmp(scenario,'fault_sensor') && tk >= 3.5 && tk < 4.5
        conf(k) = 0;
    end

    % ── Cenário aeb_disabled: AEB desabilitado ─────────────────────────────
    if strcmp(scenario,'aeb_disabled')
        aeb_en(k) = 0;
    end

    % ── Override ativo? ────────────────────────────────────────────────────
    override = (bpedal(k) ~= 0) || (abs(steer(k)) > 5.0);

    % ── TTC e d_brake ─────────────────────────────────────────────────────
    v_rel_k = v_ego(k-1) - v_tgt(k-1);
    if v_rel_k > V_REL_MIN && d_tgt(k-1) > 0
        ttc_k = min(d_tgt(k-1) / v_rel_k, 10.0);
    else
        ttc_k = 10.0;
    end
    d_brake_k = (v_ego(k-1)^2) / (2 * DECEL_L3);

    % ── Ameaça desejada ────────────────────────────────────────────────────
    in_window = (v_ego(k-1) >= V_MIN) && (v_ego(k-1) <= V_MAX);
    conf_ok   = conf(k) >= 10;
    en_ok     = aeb_en(k) ~= 0;

    if override || ~in_window || ~conf_ok || ~en_ok
        des = 1;  % STANDBY
    else
        if ttc_k <= TTC_L3 || d_brake_k >= d_tgt(k-1)
            des = 5;
        elseif ttc_k <= TTC_L2 || d_tgt(k-1) <= D_L2
            des = 4;
        elseif ttc_k <= TTC_L1 || d_tgt(k-1) <= D_L1
            des = 3;
        elseif ttc_k <= TTC_W
            des = 2;
        else
            des = 1;
        end
    end

    % ── FSM com timers ─────────────────────────────────────────────────────
    switch st
        case {1}  % STANDBY
            if des >= 2
                st = 2; warn_acc = 0; deb_t = 0;
            end
        case 2  % WARNING
            warn_acc = warn_acc + DT;
            if des < 2
                deb_t = deb_t + DT;
                if deb_t >= HYST, st = 1; deb_t = 0; end
            else
                deb_t = 0;
                if warn_acc >= WARN_MIN && des >= 3
                    st = des;
                end
            end
        case {3,4,5}  % BRAKE_Lx
            if des < st
                deb_t = deb_t + DT;
                if deb_t >= HYST
                    if des < 2
                        st = 6; pb_t = 0;
                    else
                        st = des;
                    end
                    deb_t = 0;
                end
            else
                deb_t = 0;
                if des > st, st = des; end
            end
            if v_ego(k-1) < V_MIN && st >= 3
                st = 6; pb_t = 0;
            end
        case 6  % POST_BRAKE
            pb_t = pb_t + DT;
            if pb_t >= POST
                st = 1;
            end
    end

    % ── Desaceleração alvo e dinâmica do ego ───────────────────────────────
    DEC = [0, 0, 0, 2.0, 4.0, 6.0, 6.0];
    if st <= numel(DEC)
        decel_cmd = DEC(st);
    else
        decel_cmd = 0;
    end

    a_ego(k) = -decel_cmd;
    v_ego(k) = max(0, v_ego(k-1) + a_ego(k) * DT);
    v_tgt(k) = v_tgt0;  % alvo estático (ou constante para CCRm)

    % ── Integração da distância ────────────────────────────────────────────
    v_rel_new = v_ego(k) - v_tgt(k);
    d_tgt(k)  = max(0.1, d_tgt(k-1) - v_rel_new * DT);
end

%% ── Exportar para workspace base ──────────────────────────────────────────────
ws_d_tgt  = [t, d_tgt];
ws_v_ego  = [t, v_ego];
ws_v_tgt  = [t, v_tgt];
ws_v_rel  = [t, v_ego - v_tgt];
ws_a_ego  = [t, a_ego];
ws_conf   = [t, conf];
ws_aeb_en = [t, aeb_en];
ws_steer  = [t, steer];
ws_bpedal = [t, bpedal];

vars = {'ws_d_tgt','ws_v_ego','ws_v_tgt','ws_v_rel','ws_a_ego',...
        'ws_conf','ws_aeb_en','ws_steer','ws_bpedal'};
for k = 1:numel(vars)
    assignin('base', vars{k}, eval(vars{k}));
end

fprintf('Cenario ''%s'' configurado. Execute: out = sim(''AEB_Controller'')\n', scenario);
end
