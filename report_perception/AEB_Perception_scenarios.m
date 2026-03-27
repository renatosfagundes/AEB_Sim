%% AEB_Perception_scenarios.m
%
% Configures workspace variables for different perception test scenarios.
% Call before running sim('AEB_Perception').
%
% Usage:
%   AEB_Perception_scenarios('nominal');
%   out = sim('AEB_Perception');
%   AEB_Perception_plot(out);

function AEB_Perception_scenarios(scenario_name)

    SIM_DT        = 0.001;
    SIM_STOP_TIME = 15;
    t = (0:SIM_DT:SIM_STOP_TIME)';

    switch lower(scenario_name)

        case 'nominal'
            % Ego at 50 km/h approaching stationary target at 80 m
            v_ego  = 50/3.6;
            v_tgt  = 0;
            d0     = 80;
            fi_on  = 0;
            fi_t0  = 99;
            fi_dur = 0;
            desc   = 'Nominal: 50 km/h → parado, 80 m, sem falhas';

        case 'nominal_30'
            % 30 km/h, shorter gap
            v_ego  = 30/3.6;
            v_tgt  = 0;
            d0     = 60;
            fi_on  = 0;
            fi_t0  = 99;
            fi_dur = 0;
            desc   = 'Nominal: 30 km/h → parado, 60 m';

        case 'fault_radar'
            % Nominal approach + radar fault injection at t=4s for 2s
            v_ego  = 50/3.6;
            v_tgt  = 0;
            d0     = 80;
            fi_on  = 1;
            fi_t0  = 4.0;
            fi_dur = 2.0;
            desc   = 'Falha Radar: injeção em t=4s por 2s';

        case 'fault_short'
            % Short 0.5s fault — should NOT latch (below 3-cycle threshold
            % at the 1ms level it will latch; model uses 1ms cycles)
            v_ego  = 50/3.6;
            v_tgt  = 0;
            d0     = 80;
            fi_on  = 1;
            fi_t0  = 5.0;
            fi_dur = 0.002;  % only 2 cycles — below threshold
            desc   = 'Falha curta (2 ms): abaixo do limiar de detecção';

        case 'long_range'
            % Ego at 50 km/h, gap 180 m — tests lidar range limit (100 m)
            v_ego  = 50/3.6;
            v_tgt  = 0;
            d0     = 180;
            fi_on  = 0;
            fi_t0  = 99;
            fi_dur = 0;
            desc   = 'Longo alcance: 180 m (acima do limite LiDAR 100 m)';

        case 'moving_target'
            % Both moving at different speeds
            v_ego  = 50/3.6;
            v_tgt  = 20/3.6;
            d0     = 100;
            fi_on  = 0;
            fi_t0  = 99;
            fi_dur = 0;
            desc   = 'Alvo em movimento: ego 50 km/h, alvo 20 km/h';

        case 'sustained_fault'
            % Fault lasts entire simulation — sensor declared failed
            v_ego  = 50/3.6;
            v_tgt  = 0;
            d0     = 80;
            fi_on  = 1;
            fi_t0  = 1.0;
            fi_dur = SIM_STOP_TIME;
            desc   = 'Falha sustentada: sensor declarado falho permanentemente';

        otherwise
            error('Cenário desconhecido: %s\nDisponíveis: nominal, nominal_30, fault_radar, fault_short, long_range, moving_target, sustained_fault', ...
                scenario_name);
    end

    % Build timeseries
    v_rel  = v_ego - v_tgt;
    d_true = max(0, d0 - v_rel * t);
    v_ego_ts  = v_ego * ones(size(t));
    v_rel_ts  = v_rel * ones(size(t));
    fi = double(t >= fi_t0 & t < fi_t0 + fi_dur) * fi_on;

    assignin('base', 'ts_true_distance',   timeseries(d_true, t));
    assignin('base', 'ts_true_rel_speed',  timeseries(v_rel_ts, t));
    assignin('base', 'ts_true_ego_speed',  timeseries(v_ego_ts, t));
    assignin('base', 'ts_fault_injection', timeseries(fi, t));

    assignin('base', 'SCENARIO_TRUE_DISTANCE',   d0);
    assignin('base', 'SCENARIO_TRUE_REL_SPEED',  v_rel);
    assignin('base', 'SCENARIO_TRUE_EGO_SPEED',  v_ego);
    assignin('base', 'SCENARIO_FAULT_INJECTION', fi_on);
    assignin('base', 'SCENARIO_FAULT_START',     fi_t0);
    assignin('base', 'SCENARIO_FAULT_DURATION',  fi_dur);

    fprintf('Cenário "%s" carregado.\n', scenario_name);
    fprintf('  %s\n', desc);
    fprintf('  Velocidade ego:    %.1f km/h\n', v_ego*3.6);
    fprintf('  Velocidade alvo:   %.1f km/h\n', v_tgt*3.6);
    fprintf('  Distância inicial: %.1f m\n', d0);
    if fi_on
        fprintf('  Injeção de falha:  t=%.1f s por %.3f s\n', fi_t0, fi_dur);
    else
        fprintf('  Injeção de falha:  desativada\n');
    end
end
