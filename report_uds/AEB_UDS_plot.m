function AEB_UDS_plot(out)
%AEB_UDS_PLOT  Plota resultados da simulação AEB_UDS.slx
%
% Uso:
%   AEB_UDS_scenarios('dtc_lifecycle');
%   out = sim('AEB_UDS');
%   AEB_UDS_plot(out)

%% ── Recuperar sinais ─────────────────────────────────────────────────────────
if nargin < 1
    vars = {'t','session_state','sec_unlocked','dtc_count','fault_lamp',...
            'aeb_enabled','ttc_warn_cal','ttc_l1_cal','kp_cal','resp_len'};
    missing = {};
    for k = 1:numel(vars)
        if ~evalin('base',['exist(''' vars{k} ''',''var'')'])
            missing{end+1} = vars{k}; %#ok<AGROW>
        end
    end
    if ~isempty(missing)
        error('Variaveis ausentes: %s\nExecute: out = sim(''AEB_UDS'')',...
              strjoin(missing,', '));
    end
    t            = evalin('base','t');
    session_state= evalin('base','session_state');
    sec_unlocked = evalin('base','sec_unlocked');
    dtc_count    = evalin('base','dtc_count');
    fault_lamp   = evalin('base','fault_lamp');
    aeb_enabled  = evalin('base','aeb_enabled');
    ttc_warn_cal = evalin('base','ttc_warn_cal');
    ttc_l1_cal   = evalin('base','ttc_l1_cal');
    kp_cal       = evalin('base','kp_cal');
    resp_len     = evalin('base','resp_len');
else
    get_sig = @(name) get_signal(out, name);
    t            = out.tout;
    session_state= get_sig('session_state');
    sec_unlocked = get_sig('sec_unlocked');
    dtc_count    = get_sig('dtc_count');
    fault_lamp   = get_sig('fault_lamp');
    aeb_enabled  = get_sig('aeb_enabled');
    ttc_warn_cal = get_sig('ttc_warn_cal');
    ttc_l1_cal   = get_sig('ttc_l1_cal');
    kp_cal       = get_sig('kp_cal');
    resp_len     = get_sig('resp_len');
end

colors = {[0.2 0.5 0.9],[0.9 0.3 0.2],[0.2 0.7 0.3],...
          [0.6 0.2 0.7],[0.9 0.6 0.1],[0.1 0.7 0.7]};

%% ── Figura 1: Sessão e Segurança ─────────────────────────────────────────────
figure('Name','UDS --- Sessão e Segurança','NumberTitle','off',...
       'Position',[100,100,1000,600]);

subplot(3,1,1);
stairs(t, session_state, 'Color', colors{4}, 'LineWidth',2);
yticks([1 2 3]);
yticklabels({'DEFAULT','EXTENDED','PROGRAMMING'});
ylim([0.5 3.5]); ylabel('Sessão'); grid on;
title('Estado da Sessão UDS (SID 0x10 DiagnosticSessionControl)');

subplot(3,1,2);
stairs(t, sec_unlocked, 'Color', colors{1}, 'LineWidth',2);
ylim([-0.1 1.3]); ylabel('Desbloqueado'); grid on;
title('Security Access desbloqueado (SID 0x27 --- LFSR seed + seed-key)');

subplot(3,1,3);
stairs(t, resp_len, 'Color', colors{5}, 'LineWidth',1.5);
ylabel('Bytes resp.'); xlabel('Tempo (s)'); grid on;
title('Comprimento da resposta UDS (0 = nenhuma requisição)');

sgtitle('Gerenciamento de Sessão e Security Access --- AEB\_UDS.slx');

%% ── Figura 2: DTCs e Lamp de Falha ───────────────────────────────────────────
figure('Name','UDS --- DTCs','NumberTitle','off',...
       'Position',[100,200,1000,600]);

subplot(3,1,1);
stairs(t, dtc_count, 'Color', colors{2}, 'LineWidth',2);
ylabel('Qtd DTCs'); ylim([-0.2 5]); grid on;
title('Contador de DTCs confirmados (debounce aplicado)');

subplot(3,1,2);
area(t, fault_lamp, 'FaceColor',[1 0.3 0.2],'FaceAlpha',0.4,'EdgeColor','r');
ylabel('Lamp (0/1)'); ylim([-0.1 1.3]); grid on;
title('MIL --- Malfunction Indicator Lamp (qualquer DTC ativo)');

subplot(3,1,3);
stairs(t, aeb_enabled, 'Color', colors{3}, 'LineWidth',2);
ylim([-0.1 1.3]); ylabel('AEB enable'); grid on;
title('AEB Enable --- RoutineControl 0x0301 (0=desativado, 1=ativo)');
xlabel('Tempo (s)');

sgtitle('DTCs, Lamp de Falha e Enable AEB --- SID 0x19 / 0x14 / 0x31');

%% ── Figura 3: Parâmetros calibrados (WriteDataByIdentifier) ──────────────────
figure('Name','UDS --- Calibração','NumberTitle','off',...
       'Position',[100,300,1000,600]);

subplot(3,1,1);
plot(t, ttc_warn_cal, 'Color', colors{1}, 'LineWidth',1.5);
ylabel('TTC warn (s)'); ylim([2 5]); grid on;
title('TTC Warning calibrado (DID 0xF200) --- default=4.0s');

subplot(3,1,2);
plot(t, ttc_l1_cal, 'Color', colors{5}, 'LineWidth',1.5);
ylabel('TTC L1 (s)'); ylim([1.5 4]); grid on;
title('TTC L1 calibrado (DID 0xF201) --- default=3.0s');

subplot(3,1,3);
plot(t, kp_cal, 'Color', colors{4}, 'LineWidth',1.5);
ylabel('Kp PID'); ylim([0 15]); grid on;
title('Ganho Kp calibrado (DID 0xF202) --- default=10.0');
xlabel('Tempo (s)');

sgtitle('Parâmetros Calibráveis via UDS WriteDataByIdentifier (SID 0x2E)');

fprintf('Plots gerados com sucesso.\n');
end

%% ── Auxiliar ─────────────────────────────────────────────────────────────────
function data = get_signal(out, sig_name)
    try
        data = out.get(sig_name).signals.values;
        if size(data,2) > 1, data = data(:,1); end
    catch
        warning('Sinal ''%s'' nao encontrado. Retornando zeros.',sig_name);
        data = zeros(length(out.tout),1);
    end
end
