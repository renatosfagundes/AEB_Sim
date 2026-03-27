function AEB_Controller_plot(out)
%AEB_CONTROLLER_PLOT  Plota resultados da simulação AEB_Controller.slx
%
% Uso:
%   out = sim('AEB_Controller');
%   AEB_Controller_plot(out)

%% ── Recuperar sinais ─────────────────────────────────────────────────────────
if nargin < 1
    vars = {'t','fsm_state','decel_target','brake_pct','brake_bar',...
            'ttc_out','d_brake_out','is_closing','alert_level','alert_active',...
            'buzzer_cmd','override_out','warn_timer','state_timer'};
    missing = {};
    for k = 1:numel(vars)
        if ~evalin('base',['exist(''' vars{k} ''',''var'')'])
            missing{end+1} = vars{k}; %#ok<AGROW>
        end
    end
    if ~isempty(missing)
        error('Variaveis ausentes no workspace: %s\nExecute primeiro: out = sim(''AEB_Controller'')', ...
              strjoin(missing,', '));
    end
    t           = evalin('base','t');
    fsm_state   = evalin('base','fsm_state');
    decel_target= evalin('base','decel_target');
    brake_pct   = evalin('base','brake_pct');
    brake_bar   = evalin('base','brake_bar');
    ttc_out     = evalin('base','ttc_out');
    d_brake_out = evalin('base','d_brake_out');
    is_closing  = evalin('base','is_closing');
    alert_level = evalin('base','alert_level');
    alert_active= evalin('base','alert_active');
    buzzer_cmd  = evalin('base','buzzer_cmd');
    override_out= evalin('base','override_out');
    warn_timer  = evalin('base','warn_timer');
    state_timer = evalin('base','state_timer');
else
    get_sig = @(name) get_signal(out, name);
    t           = out.tout;
    fsm_state   = get_sig('fsm_state');
    decel_target= get_sig('decel_target');
    brake_pct   = get_sig('brake_pct');
    brake_bar   = get_sig('brake_bar');
    ttc_out     = get_sig('ttc_out');
    d_brake_out = get_sig('d_brake_out');
    is_closing  = get_sig('is_closing');
    alert_level = get_sig('alert_level');
    alert_active= get_sig('alert_active');
    buzzer_cmd  = get_sig('buzzer_cmd');
    override_out= get_sig('override_out');
    warn_timer  = get_sig('warn_timer');
    state_timer = get_sig('state_timer');
end

colors = {[0.2 0.5 0.9], [0.9 0.3 0.2], [0.2 0.7 0.3], ...
          [0.6 0.2 0.7], [0.9 0.6 0.1], [0.1 0.7 0.7]};

%% ── Figura 1: TTC e estado FSM ───────────────────────────────────────────────
figure('Name','Controlador AEB --- TTC e FSM','NumberTitle','off',...
       'Position',[100,100,1000,700]);

subplot(3,1,1);
plot(t, ttc_out, 'Color', colors{1}, 'LineWidth', 1.5); hold on;
yline(4.0,'--k','TTC_{warn}=4.0s','LabelHorizontalAlignment','left');
yline(3.0,'--','Color',[0.8 0.4 0],'Label','TTC_{L1}=3.0s','LabelHorizontalAlignment','left');
yline(2.2,'--','Color',[0.9 0.3 0.2],'Label','TTC_{L2}=2.2s','LabelHorizontalAlignment','left');
yline(1.8,'--r','TTC_{L3}=1.8s','LabelHorizontalAlignment','left');
plot(t, d_brake_out, ':', 'Color', colors{3}, 'LineWidth', 1.2);
ylabel('TTC (s) / d_{brake} (m)'); ylim([0 12]); grid on;
legend('TTC','TTC_{warn}','TTC_{L1}','TTC_{L2}','TTC_{L3}','d_{brake}(m)',...
       'Location','northeast','FontSize',7);
title('TTC calculado e d_{brake} --- limiares de intervenção');

subplot(3,1,2);
stairs(t, fsm_state, 'Color', colors{4}, 'LineWidth', 2);
yticks(0:6);
yticklabels({'OFF','STANDBY','WARNING','BRAKE\_L1','BRAKE\_L2','BRAKE\_L3','POST'});
ylabel('Estado FSM'); ylim([-0.5 6.5]); grid on;
title('Estado FSM (7 estados --- aeb\_fsm.c)');

subplot(3,1,3);
plot(t, warn_timer, 'Color', colors{5}, 'LineWidth', 1.5); hold on;
yline(0.8,'--k','warn\_min=0.8s','LabelHorizontalAlignment','left');
plot(t, state_timer, 'Color', colors{2}, 'LineWidth', 1.2, 'LineStyle',':');
ylabel('Timer (s)'); grid on;
legend('warn\_timer','warn\_min=0.8s','state\_timer','Location','northeast');
title('Acumulador de aviso e timer de estado');
xlabel('Tempo (s)');

sgtitle('AEB Controller --- TTC e FSM');

%% ── Figura 2: Comando de freio (PID) ─────────────────────────────────────────
figure('Name','Controlador AEB --- Freio PID','NumberTitle','off',...
       'Position',[100,200,1000,600]);

subplot(3,1,1);
plot(t, decel_target, 'Color', colors{2}, 'LineWidth', 1.5); hold on;
yline(2.0,'--','Color',[0.5 0.5 0.5],'Label','L1=2 m/s^2','LabelHorizontalAlignment','left');
yline(4.0,'--','Color',[0.7 0.4 0],'Label','L2=4 m/s^2','LabelHorizontalAlignment','left');
yline(6.0,'--r','L3=6 m/s^2','LabelHorizontalAlignment','left');
ylabel('Desac. alvo (m/s^2)'); ylim([-0.5 8]); grid on;
title('Desaceleração alvo (saída FSM --- build\_output)');

subplot(3,1,2);
plot(t, brake_pct, 'Color', colors{1}, 'LineWidth', 1.5); hold on;
yline(20,'--','Color',[0.5 0.5 0.5],'Label','L1=20%','LabelHorizontalAlignment','left');
yline(40,'--','Color',[0.7 0.4 0],'Label','L2=40%','LabelHorizontalAlignment','left');
yline(60,'--r','L3=60%','LabelHorizontalAlignment','left');
ylabel('Freio (%)'); ylim([-2 105]); grid on;
title('Saída PID --- brake\_pct (0-100%) com limitação de jerk');

subplot(3,1,3);
plot(t, brake_bar, 'Color', colors{3}, 'LineWidth', 1.5);
ylabel('Pressão (bar)'); grid on;
title('Pressão de freio (bar) --- brake\_bar = brake\_pct * 0.1');
xlabel('Tempo (s)');

sgtitle('Controlador AEB --- Resposta PID (Kp=10, Ki=0.05)');

%% ── Figura 3: Alertas e override ─────────────────────────────────────────────
figure('Name','Controlador AEB --- Alertas','NumberTitle','off',...
       'Position',[100,300,1000,600]);

subplot(4,1,1);
stairs(t, alert_level, 'Color', colors{5}, 'LineWidth', 2);
yticks(0:3);
yticklabels({'OFF','AVISO','PARCIAL','TOTAL'});
ylabel('Nível'); ylim([-0.5 3.5]); grid on;
title('Nível de alerta (aeb\_alert.c)');

subplot(4,1,2);
stairs(t, alert_active, 'Color', colors{1}, 'LineWidth', 1.5);
ylabel('Ativo (0/1)'); ylim([-0.1 1.3]); grid on;
title('Alerta ativo (visual ou sonoro)');

subplot(4,1,3);
stairs(t, buzzer_cmd, 'Color', colors{2}, 'LineWidth', 2);
yticks(0:4);
yticklabels({'Off','SingleBeep','DoubleBeep','Continuous','FastPulse'});
ylabel('BuzzerCmd'); ylim([-0.5 4.5]); grid on;
title('Comando buzzer --- mapeamento DBC 0x300');

subplot(4,1,4);
stairs(t, override_out, 'Color', colors{6}, 'LineWidth', 2);
ylabel('Override (0/1)'); ylim([-0.1 1.3]); grid on;
title('Override do motorista (pedal freio ou volante > 5 graus)');
xlabel('Tempo (s)');

sgtitle('Alertas e Override --- AEB Controller');

fprintf('Plots gerados com sucesso.\n');
end

%% ── Auxiliar ─────────────────────────────────────────────────────────────────
function data = get_signal(out, sig_name)
    try
        data = out.get(sig_name).signals.values;
        if size(data,2) > 1, data = data(:,1); end
    catch
        warning('Sinal ''%s'' nao encontrado em out. Retornando zeros.', sig_name);
        data = zeros(length(out.tout), 1);
    end
end
