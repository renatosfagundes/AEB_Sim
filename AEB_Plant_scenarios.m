%% AEB_Plant_scenarios.m
%
% Configures workspace variables for different Euro NCAP AEB scenarios.
% Call before running sim('AEB_Plant').
%
% Usage:
%   AEB_Plant_scenarios('CCRs_50');  % Set CCRs at 50 km/h
%   out = sim('AEB_Plant');
%   AEB_Plant_plot(out);

function AEB_Plant_scenarios(scenario_name)

    switch lower(scenario_name)

        case 'ccrs_20'
            % CCRs: ego at 20 km/h, target stationary
            assignin('base', 'SCENARIO_EGO_SPEED',    20/3.6);
            assignin('base', 'SCENARIO_TARGET_SPEED',  0);
            assignin('base', 'SCENARIO_INITIAL_GAP',   40);
            assignin('base', 'SCENARIO_TARGET_DECEL',  0);
            assignin('base', 'SCENARIO_BRAKE_TIME',    99);

        case 'ccrs_30'
            assignin('base', 'SCENARIO_EGO_SPEED',    30/3.6);
            assignin('base', 'SCENARIO_TARGET_SPEED',  0);
            assignin('base', 'SCENARIO_INITIAL_GAP',   60);
            assignin('base', 'SCENARIO_TARGET_DECEL',  0);
            assignin('base', 'SCENARIO_BRAKE_TIME',    99);

        case 'ccrs_40'
            assignin('base', 'SCENARIO_EGO_SPEED',    40/3.6);
            assignin('base', 'SCENARIO_TARGET_SPEED',  0);
            assignin('base', 'SCENARIO_INITIAL_GAP',   80);
            assignin('base', 'SCENARIO_TARGET_DECEL',  0);
            assignin('base', 'SCENARIO_BRAKE_TIME',    99);

        case 'ccrs_50'
            assignin('base', 'SCENARIO_EGO_SPEED',    50/3.6);
            assignin('base', 'SCENARIO_TARGET_SPEED',  0);
            assignin('base', 'SCENARIO_INITIAL_GAP',   100);
            assignin('base', 'SCENARIO_TARGET_DECEL',  0);
            assignin('base', 'SCENARIO_BRAKE_TIME',    99);

        case 'ccrm'
            % CCRm: ego at 50 km/h, target at 20 km/h
            assignin('base', 'SCENARIO_EGO_SPEED',    50/3.6);
            assignin('base', 'SCENARIO_TARGET_SPEED',  20/3.6);
            assignin('base', 'SCENARIO_INITIAL_GAP',   100);
            assignin('base', 'SCENARIO_TARGET_DECEL',  0);
            assignin('base', 'SCENARIO_BRAKE_TIME',    99);

        case 'ccrb'
            % CCRb: ego at 50 km/h, target at 50 km/h, target brakes at t=3s
            assignin('base', 'SCENARIO_EGO_SPEED',    50/3.6);
            assignin('base', 'SCENARIO_TARGET_SPEED',  50/3.6);
            assignin('base', 'SCENARIO_INITIAL_GAP',   40);
            assignin('base', 'SCENARIO_TARGET_DECEL', -2.0);
            assignin('base', 'SCENARIO_BRAKE_TIME',    3.0);

        case 'ccrb_hard'
            % CCRb hard: target brakes at -6 m/s^2
            assignin('base', 'SCENARIO_EGO_SPEED',    50/3.6);
            assignin('base', 'SCENARIO_TARGET_SPEED',  50/3.6);
            assignin('base', 'SCENARIO_INITIAL_GAP',   40);
            assignin('base', 'SCENARIO_TARGET_DECEL', -6.0);
            assignin('base', 'SCENARIO_BRAKE_TIME',    3.0);

        case 'no_aeb'
            % No AEB baseline: ego at 50 km/h, target stationary, no brake input
            assignin('base', 'SCENARIO_EGO_SPEED',    50/3.6);
            assignin('base', 'SCENARIO_TARGET_SPEED',  0);
            assignin('base', 'SCENARIO_INITIAL_GAP',   100);
            assignin('base', 'SCENARIO_TARGET_DECEL',  0);
            assignin('base', 'SCENARIO_BRAKE_TIME',    99);

        otherwise
            error('Unknown scenario: %s\nAvailable: CCRs_20, CCRs_30, CCRs_40, CCRs_50, CCRm, CCRb, CCRb_hard, no_aeb', scenario_name);
    end

    fprintf('Scenario "%s" loaded.\n', scenario_name);
    fprintf('  Ego speed:     %.1f km/h\n', evalin('base', 'SCENARIO_EGO_SPEED') * 3.6);
    fprintf('  Target speed:  %.1f km/h\n', evalin('base', 'SCENARIO_TARGET_SPEED') * 3.6);
    fprintf('  Initial gap:   %.1f m\n',    evalin('base', 'SCENARIO_INITIAL_GAP'));
    fprintf('  Target decel:  %.1f m/s^2\n', evalin('base', 'SCENARIO_TARGET_DECEL'));
    fprintf('  Brake time:    %.1f s\n',    evalin('base', 'SCENARIO_BRAKE_TIME'));
end