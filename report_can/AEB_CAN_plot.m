function AEB_CAN_plot(out)
%AEB_CAN_PLOT  Plota resultados da simulação AEB_CAN.slx
%
% Uso:
%   out = sim('AEB_CAN');
%   AEB_CAN_plot(out)
%   % ou sem argumento (usa workspace global):
%   AEB_CAN_plot

%% ── Recuperar sinais ─────────────────────────────────────────────────────────
if nargin < 1
    % Tenta variáveis do workspace base
    vars = {'t','breq_dec','bpress_dec','bmode_dec','crc_err','alive_err',...
            'timeout_err','dtc_c1004','dtc_c1005','dtc_c1010','err_count',...
            'dist_dec','vrel_dec','ttc_dec','spd_dec','fsm_dec'};
    missing = {};
    for k=1:numel(vars)
        if ~evalin('base',['exist(''' vars{k} ''',''var'')'])
            missing{end+1} = vars{k}; %#ok<AGROW>
        end
    end
    if ~isempty(missing)
        error('Variaveis ausentes no workspace: %s\nExecute primeiro: out = sim(''AEB_CAN'')', ...
              strjoin(missing,', '));
    end
    t          = evalin('base','t');
    breq_dec   = evalin('base','breq_dec');
    bpress_dec = evalin('base','bpress_dec');
    crc_err    = evalin('base','crc_err');
    alive_err  = evalin('base','alive_err');
    timeout_err= evalin('base','timeout_err');
    dtc_c1004  = evalin('base','dtc_c1004');
    dtc_c1005  = evalin('base','dtc_c1005');
    dtc_c1010  = evalin('base','dtc_c1010');
    err_count  = evalin('base','err_count');
    dist_dec   = evalin('base','dist_dec');
    vrel_dec   = evalin('base','vrel_dec');
    ttc_dec    = evalin('base','ttc_dec');
    spd_dec    = evalin('base','spd_dec');
    fsm_dec    = evalin('base','fsm_dec');
else
    % Extrai de 'out' (resultado do sim())
    get_sig = @(name) get_signal(out, name);
    t          = out.tout;
    breq_dec   = get_sig('breq_dec');
    bpress_dec = get_sig('bpress_dec');
    crc_err    = get_sig('crc_err');
    alive_err  = get_sig('alive_err');
    timeout_err= get_sig('timeout_err');
    dtc_c1004  = get_sig('dtc_c1004');
    dtc_c1005  = get_sig('dtc_c1005');
    dtc_c1010  = get_sig('dtc_c1010');
    err_count  = get_sig('err_count');
    dist_dec   = get_sig('dist_dec');
    vrel_dec   = get_sig('vrel_dec');
    ttc_dec    = get_sig('ttc_dec');
    spd_dec    = get_sig('spd_dec');
    fsm_dec    = get_sig('fsm_dec');
end

%% ── Figura 1: Sinais de percepção decodificados ──────────────────────────────
figure('Name','CAN — Sinais Decodificados','NumberTitle','off','Position',[100,100,900,700]);
colors = {[0.2 0.5 0.9], [0.9 0.3 0.2], [0.2 0.7 0.3], [0.6 0.2 0.7]};

subplot(4,1,1);
plot(t, dist_dec, 'Color', colors{1}, 'LineWidth', 1.5);
ylabel('Distância (m)'); grid on;
title('AEB\_RadarTarget: TargetDistance decodificado (0x120)');

subplot(4,1,2);
plot(t, vrel_dec, 'Color', colors{2}, 'LineWidth', 1.5);
ylabel('v_{rel} (m/s)'); grid on;
title('AEB\_RadarTarget: RelativeSpeed decodificado');

subplot(4,1,3);
plot(t, ttc_dec, 'Color', colors{3}, 'LineWidth', 1.5);
hold on;
yline(4.0,'--k','TTC_{warn}=4.0s','LabelHorizontalAlignment','left');
yline(3.0,'--','Color',[0.8 0.4 0],'Label','TTC_{L1}=3.0s','LabelHorizontalAlignment','left');
yline(1.8,'--r','TTC_{L3}=1.8s','LabelHorizontalAlignment','left');
ylabel('TTC (s)'); ylim([0 10]); grid on;
title('TTC calculado — limiares de frenagem');

subplot(4,1,4);
plot(t, spd_dec * 3.6, 'Color', colors{4}, 'LineWidth', 1.5);
ylabel('v_{ego} (km/h)'); xlabel('Tempo (s)'); grid on;
title('AEB\_EgoVehicle: VehicleSpeed decodificado (0x100)');

sgtitle('Sinais CAN Decodificados — AEB\_CAN.slx');

%% ── Figura 2: Comando de freio (AEB_BrakeCmd) ───────────────────────────────
figure('Name','CAN — AEB\_BrakeCmd (0x080 ASIL-B)','NumberTitle','off','Position',[100,200,900,600]);

subplot(3,1,1);
plot(t, bpress_dec, 'Color', colors{1}, 'LineWidth', 1.5);
ylabel('Pressão (bar)'); grid on;
title('AEB\_BrakeCmd: BrakePressure decodificado — 0=Off, 2=L1, 4=L2, 6=L3');

subplot(3,1,2);
stairs(t, breq_dec, 'Color', colors{2}, 'LineWidth', 1.5);
ylabel('BrakeRequest'); ylim([-0.1 1.3]); grid on;
title('BrakeRequest (0=inativo, 1=freio ativo)');

subplot(3,1,3);
stairs(t, fsm_dec, 'Color', colors{4}, 'LineWidth', 1.5);
yticks(0:6); yticklabels({'OFF','STANDBY','WARNING','L1','L2','L3','POST'});
ylabel('Estado FSM'); xlabel('Tempo (s)'); grid on;
title('Estado FSM (AEB\_FSMState 0x200)');

sgtitle('Comando de Freio CAN — AEB\_BrakeCmd ASIL-B');

%% ── Figura 3: Diagnóstico CAN (falhas e DTCs) ────────────────────────────────
figure('Name','CAN — Diagnóstico e DTCs','NumberTitle','off','Position',[100,300,900,700]);

subplot(4,1,1);
plot(t, crc_err, 'r', 'LineWidth', 1.5); hold on;
plot(t, alive_err, 'b', 'LineWidth', 1.5);
plot(t, timeout_err, 'k', 'LineWidth', 1.5);
legend('CRC Error','AliveCounter Error','Timeout Error','Location','northeast');
ylabel('Flag (0/1)'); ylim([-0.1 1.3]); grid on;
title('Flags de erro instantâneo (RX\_BrakeCmd)');

subplot(4,1,2);
stairs(t, dtc_c1004, 'r',  'LineWidth', 2); hold on;
stairs(t, dtc_c1005, 'b',  'LineWidth', 2);
stairs(t, dtc_c1010, 'k--','LineWidth', 2);
legend('DTC C1004 (CRC)','DTC C1005 (Alive)','DTC C1010 (Timeout)','Location','northeast');
ylabel('DTC Ativo'); ylim([-0.1 1.3]); grid on;
title('DTCs confirmados (debounce 3 ciclos)');

subplot(4,1,3);
plot(t, err_count, 'Color', [0.6 0.2 0.7], 'LineWidth', 1.5);
ylabel('Contagem'); grid on;
title('Contador acumulado de erros');

subplot(4,1,4);
fault_total = dtc_c1004 | dtc_c1005 | dtc_c1010;
area(t, fault_total, 'FaceColor', [1 0.3 0.2], 'FaceAlpha', 0.4, 'EdgeColor','r');
ylabel('DTC Ativo'); xlabel('Tempo (s)'); ylim([-0.1 1.3]); grid on;
title('Janela de DTC ativo (qualquer DTC confirmado)');

sgtitle('Diagnóstico CAN — AEB\_BrakeCmd ASIL-B');

fprintf('Plots gerados com sucesso.\n');
end

%% ── Auxiliar ─────────────────────────────────────────────────────────────────
function data = get_signal(out, sig_name)
%GET_SIGNAL  Extrai sinal de simulação do struct de saída
    try
        data = out.get(sig_name).signals.values;
        if size(data,2) > 1, data = data(:,1); end
    catch
        warning('Sinal ''%s'' nao encontrado em out. Retornando zeros.', sig_name);
        data = zeros(length(out.tout), 1);
    end
end
