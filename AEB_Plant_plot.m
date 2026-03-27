%% AEB_Plant_plot.m
%
% Plots simulation results from the AEB Plant model.
% Usage: AEB_Plant_plot(out)
%   where out = sim('AEB_Plant');

function AEB_Plant_plot(out)

    figure('Name', 'AEB Plant Simulation Results', ...
           'Position', [100, 100, 900, 700], ...
           'Color', 'w');

    % Extract timeseries from workspace logging
    t_ego   = out.ego_speed_log.Time;
    v_ego   = out.ego_speed_log.Data;
    a_ego   = out.ego_accel_log.Data;
    d       = out.distance_log.Data;
    v_tgt   = out.target_speed_log.Data;
    v_rel   = out.rel_speed_log.Data;

    % --- Subplot 1: Speeds ---
    subplot(4,1,1);
    plot(t_ego, v_ego * 3.6, 'b-', 'LineWidth', 1.5); hold on;
    plot(t_ego, v_tgt * 3.6, 'r--', 'LineWidth', 1.5);
    ylabel('Velocidade [km/h]');
    legend('v_{ego}', 'v_{target}', 'Location', 'best');
    title('Velocidades dos Veículos');
    grid on;
    xlim([0 max(t_ego)]);

    % --- Subplot 2: Distance ---
    subplot(4,1,2);
    plot(t_ego, d, 'k-', 'LineWidth', 1.5);
    ylabel('Distância [m]');
    title('Distância Inter-Veicular');
    grid on;
    xlim([0 max(t_ego)]);

    % --- Subplot 3: Relative Speed ---
    subplot(4,1,3);
    plot(t_ego, v_rel, 'm-', 'LineWidth', 1.5);
    ylabel('v_{rel} [m/s]');
    title('Velocidade Relativa (v_{ego} - v_{target})');
    grid on;
    xlim([0 max(t_ego)]);

    % --- Subplot 4: Acceleration ---
    subplot(4,1,4);
    plot(t_ego, a_ego, 'Color', [0.8 0.2 0.2], 'LineWidth', 1.5);
    ylabel('Aceleração [m/s^2]');
    xlabel('Tempo [s]');
    title('Aceleração do Ego (negativo = frenagem)');
    grid on;
    xlim([0 max(t_ego)]);

    % Find key events
    idx_collision = find(d <= 0.5, 1, 'first');
    idx_stopped   = find(v_ego < 0.01, 1, 'first');

    fprintf('\n=== Resultados da Simulação ===\n');
    fprintf('Velocidade inicial ego:  %.1f km/h\n', v_ego(1)*3.6);
    fprintf('Velocidade inicial alvo: %.1f km/h\n', v_tgt(1)*3.6);
    fprintf('Distância inicial:       %.1f m\n', d(1));

    if ~isempty(idx_stopped) && (isempty(idx_collision) || idx_stopped < idx_collision)
        fprintf('RESULTADO: Veículo PAROU em t=%.2f s\n', t_ego(idx_stopped));
        fprintf('  Distância final:  %.2f m\n', d(idx_stopped));
        fprintf('  Velocidade final: %.2f km/h\n', v_ego(idx_stopped)*3.6);
    elseif ~isempty(idx_collision)
        fprintf('RESULTADO: COLISÃO em t=%.2f s\n', t_ego(idx_collision));
        fprintf('  Velocidade de impacto: %.1f km/h\n', v_ego(idx_collision)*3.6);
        fprintf('  Distância no impacto:  %.2f m\n', d(idx_collision));
    else
        fprintf('RESULTADO: Simulação terminou sem parada ou colisão.\n');
        fprintf('  Distância final:  %.2f m\n', d(end));
        fprintf('  Velocidade final: %.2f km/h\n', v_ego(end)*3.6);
    end
end