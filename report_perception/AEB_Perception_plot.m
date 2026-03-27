%% AEB_Perception_plot.m
%
% Plots simulation results from the AEB Perception model.
% Usage: AEB_Perception_plot(out)
%   where out = sim('AEB_Perception');

function AEB_Perception_plot(out)

    figure('Name', 'AEB Perception Simulation Results', ...
           'Position', [100, 100, 950, 750], ...
           'Color', 'w');

    % Extract timeseries
    t         = out.d_meas_log.Time;
    d_true    = out.d_true_log.Data;
    d_meas    = out.d_meas_log.Data;
    vr_meas   = out.vr_meas_log.Data;
    ve_meas   = out.ve_meas_log.Data;
    conf      = out.conf_log.Data;
    fault     = out.fault_log.Data;

    colors.true   = [0.1  0.1  0.8];  % dark blue
    colors.radar  = [0.85 0.33 0.10]; % orange
    colors.fused  = [0.1  0.6  0.1];  % green
    colors.fault  = [0.8  0.0  0.0];  % red

    % ---------------------------------------------------------------
    % Subplot 1: Distance comparison
    % ---------------------------------------------------------------
    subplot(4,1,1);
    plot(t, d_true, '-',  'Color', colors.true,  'LineWidth', 2.0); hold on;
    plot(t, d_meas, '--', 'Color', colors.fused, 'LineWidth', 1.5);
    ylabel('Distância [m]');
    title('Distância: Verdadeira vs. Medida (Fusão)');
    legend('d_{true}', 'd_{fused}', 'Location', 'best');
    grid on;
    xlim([0 max(t)]);

    % ---------------------------------------------------------------
    % Subplot 2: Speed measurements
    % ---------------------------------------------------------------
    subplot(4,1,2);
    plot(t, vr_meas * 3.6, '-', 'Color', colors.fused, 'LineWidth', 1.5); hold on;
    plot(t, ve_meas * 3.6, '--','Color', colors.radar,  'LineWidth', 1.5);
    ylabel('Velocidade [km/h]');
    title('Velocidades Medidas (após Fusão)');
    legend('v_{rel}', 'v_{ego}', 'Location', 'best');
    grid on;
    xlim([0 max(t)]);

    % ---------------------------------------------------------------
    % Subplot 3: Confidence
    % ---------------------------------------------------------------
    subplot(4,1,3);
    plot(t, conf, '-', 'Color', [0.2 0.6 0.2], 'LineWidth', 1.5);
    ylim([-0.1 1.3]);
    yticks([0 0.5 1]);
    yticklabels({'0 (falha)', '0.5', '1 (ok)'});
    ylabel('Confiança');
    title('Nível de Confiança do Sensor');
    grid on;
    xlim([0 max(t)]);

    % ---------------------------------------------------------------
    % Subplot 4: Fault flag
    % ---------------------------------------------------------------
    subplot(4,1,4);
    stairs(t, fault, '-', 'Color', colors.fault, 'LineWidth', 2.0);
    ylim([-0.2 1.5]);
    yticks([0 1]);
    yticklabels({'OK', 'FALHA'});
    ylabel('Sensor Fault');
    xlabel('Tempo [s]');
    title('Flag de Falha do Sensor');
    grid on;
    xlim([0 max(t)]);

    % ---------------------------------------------------------------
    % Print summary
    % ---------------------------------------------------------------
    fprintf('\n=== Resultados da Percepção ===\n');
    fprintf('Tempo de simulação: %.1f s\n', max(t));

    fault_idx = find(fault > 0.5, 1, 'first');
    if ~isempty(fault_idx)
        fprintf('FALHA DETECTADA em t=%.3f s\n', t(fault_idx));
        fault_end = find(fault(fault_idx:end) < 0.5, 1, 'first');
        if isempty(fault_end)
            fprintf('  Falha persistente até o fim da simulação\n');
        else
            fprintf('  Falha resolvida em t=%.3f s (duração=%.3f s)\n', ...
                t(fault_idx + fault_end - 1), t(fault_end));
        end
    else
        fprintf('Nenhuma falha detectada\n');
    end

    noise_err = d_meas - d_true;
    fprintf('Erro médio de distância: %.4f m\n', mean(abs(noise_err)));
    fprintf('Erro máximo de distância: %.4f m\n', max(abs(noise_err)));
    fprintf('RMSE distância: %.4f m\n', sqrt(mean(noise_err.^2)));

    % Confidence stats
    mean_conf = mean(conf);
    min_conf  = min(conf);
    fprintf('Confiança média: %.3f | mínima: %.3f\n', mean_conf, min_conf);
end
