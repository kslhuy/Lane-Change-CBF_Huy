%% Plot details for one saved attack case
% Run main_mean_attack_plot.m first. This script uses all_case_vehicles
% saved in the MATLAB workspace and does not rerun the simulation.

script_dir = fileparts(mfilename('fullpath'));
if exist('plot_observer_weight_diagnostics', 'file') ~= 2 && ~isempty(script_dir)
    addpath(fullfile(script_dir, 'Function'));
end

% Leave empty to choose from the MATLAB command window every time.
% Set a number here to always plot that case without the prompt.
% Example:
%   case_to_plot = 3;
%   plot_saved_attack_case_details
case_to_plot = [4];

% Plot switches. Change these to control how many figures are generated.
plot_motion_history = true;
plot_relative_motion_history = true;
plot_fleet_error_all = true;
plot_fleet_error_each_observer = true;
plot_global_state_each_observer = false;
plot_local_error_each_observer = false;
plot_controller_gamma = false;
plot_trust_summary = true;
plot_trip_trust_details = false;
plot_trust_debug_diagnostics = true;
trust_debug_target_id = []; % empty -> attacker_id_for_plot
plot_weight_diagnostics = true;
plot_attack_values = true;

if ~exist('all_case_vehicles', 'var') || isempty(all_case_vehicles)
    error(['No saved attack-case data found. Run main_mean_attack_plot.m first, ' ...
           'then run this script in the same MATLAB workspace.']);
end

total_saved_cases = numel(all_case_vehicles);
available_case_numbers = nan(1, total_saved_cases);

for idx = 1:total_saved_cases
    if exist('all_case_scenarios', 'var') && numel(all_case_scenarios) >= idx && ...
            ~isempty(all_case_scenarios{idx}) && isfield(all_case_scenarios{idx}, 'case_number')
        available_case_numbers(idx) = all_case_scenarios{idx}.case_number;
    elseif exist('attack_case_numbers', 'var') && numel(attack_case_numbers) >= idx
        available_case_numbers(idx) = attack_case_numbers(idx);
    else
        available_case_numbers(idx) = idx;
    end
end

fprintf('\nSaved attack cases available:\n');
for idx = 1:total_saved_cases
    case_label = sprintf('Case %d', available_case_numbers(idx));
    if exist('attack_descriptions', 'var') && numel(attack_descriptions) >= idx
        case_label = sprintf('%s - %s', case_label, attack_descriptions{idx});
    end
    fprintf('  index %d -> %s\n', idx, case_label);
end

if isempty(case_to_plot)
    case_to_plot = input('Enter attack case number to plot: ');
end

case_idx = find(available_case_numbers == case_to_plot, 1);

% If the entered value is not a case number, allow it as a cell index.
if isempty(case_idx) && isnumeric(case_to_plot) && isscalar(case_to_plot) && ...
        case_to_plot >= 1 && case_to_plot <= total_saved_cases
    case_idx = case_to_plot;
end

if isempty(case_idx)
    error('Requested case %s is not available in all_case_vehicles.', mat2str(case_to_plot));
end

selected_case_number = available_case_numbers(case_idx);
selected_case_vehicles = all_case_vehicles{case_idx};
selected_case_scenario = struct();

if exist('all_case_scenarios', 'var') && numel(all_case_scenarios) >= case_idx
    selected_case_scenario = all_case_scenarios{case_idx};
end

vehicles = selected_case_vehicles;
num_vehicles_case = length(vehicles);
scenario_config = vehicles(1).scenarios_config;
dt_case = scenario_config.dt;

if exist('attacker_vehicle_id', 'var')
    attacker_id_for_plot = attacker_vehicle_id;
else
    attacker_id_for_plot = 1;
end

if exist('trust_threshold', 'var')
    trust_threshold_for_plot = trust_threshold;
else
    trust_threshold_for_plot = 0.5;
end

fprintf('\nPlotting details for saved attack Case %d (cell index %d).\n', ...
    selected_case_number, case_idx);
fprintf('Saved selected-case data as selected_case_vehicles in the workspace.\n');

plot_simulator = Simulator([], [], vehicles, dt_case, false);

if plot_motion_history
    plot_simulator.plot_movement_log_simple(vehicles, scenario_config, num_vehicles_case);
    sgtitle(sprintf('Case %d - Vehicle Motion History', selected_case_number));
end

if plot_relative_motion_history
    plot_simulator.plot_relative_movement_log_simple(vehicles, scenario_config, num_vehicles_case);
    sgtitle(sprintf('Case %d - Relative Motion History', selected_case_number));
end

if plot_fleet_error_all
    plot_simulator.plot_ground_error_global_est_ALL(vehicles);
    sgtitle(sprintf('Case %d - Fleet Estimation Error, All Observers', selected_case_number));
end

if plot_fleet_error_each_observer
    for vehicle_id = 1:num_vehicles_case
        vehicles(vehicle_id).plot_ground_error_global_est(vehicles);
        sgtitle(sprintf('Case %d - Fleet Estimation Error from Observer V%d', ...
            selected_case_number, vehicle_id));
    end
end

if plot_global_state_each_observer
    for vehicle_id = 1:num_vehicles_case
        vehicles(vehicle_id).observer.plot_global_state_log();
        sgtitle(sprintf('Case %d - Global State Estimates from Observer V%d', ...
            selected_case_number, vehicle_id));
    end
end

if plot_local_error_each_observer
    for vehicle_id = 1:num_vehicles_case
        vehicles(vehicle_id).observer.plot_error_local_estimated();
        sgtitle(sprintf('Case %d - Local Estimation Error V%d', ...
            selected_case_number, vehicle_id));
    end
end

if plot_controller_gamma
    for vehicle_id = 1:num_vehicles_case
        if ~isempty(vehicles(vehicle_id).u1_log)
            vehicles(vehicle_id).plot_u1_u2_gamma();
            sgtitle(sprintf('Case %d - Controller and Gamma V%d', ...
                selected_case_number, vehicle_id));
        end
    end
end

if plot_trust_summary
    plot_simulator.plot_all_trust_log(vehicles);
    sgtitle(sprintf('Case %d - All Trust Values', selected_case_number));

    plot_all_trust_log_exclude_attacker_case(vehicles, attacker_id_for_plot, selected_case_number);
end

if plot_trip_trust_details
    for host_id = 1:num_vehicles_case
        for target_id = 1:num_vehicles_case
            if host_id == target_id
                continue;
            end

            if numel(vehicles(host_id).trip_models) >= target_id && ...
                    ~isempty(vehicles(host_id).trip_models{target_id})
                vehicles(host_id).trip_models{target_id}.plot_trust_log(host_id, target_id);
                sgtitle(sprintf('Case %d - TrIP Trust V%d to V%d', ...
                    selected_case_number, host_id, target_id));
            end
        end
    end
end

if plot_trust_debug_diagnostics
    if isempty(trust_debug_target_id)
        trust_debug_target_id = attacker_id_for_plot;
    end

    plot_trip_trust_debug_case(vehicles, selected_case_scenario, ...
        selected_case_number, attacker_id_for_plot, trust_debug_target_id, ...
        trust_threshold_for_plot);
end

if plot_weight_diagnostics
    weight_focus_target_id = attacker_id_for_plot;
    if exist('trust_debug_target_id', 'var') && ~isempty(trust_debug_target_id)
        weight_focus_target_id = trust_debug_target_id;
    end
    [~, weight_attack_start_s, weight_attack_end_s] = ...
        case_time_settings(vehicles, selected_case_scenario);

    plot_observer_weight_diagnostics(vehicles, attacker_id_for_plot, ...
        weight_attack_start_s, weight_attack_end_s, ...
        trust_threshold_for_plot, selected_case_number, weight_focus_target_id);
end

if plot_attack_values
    attack_module_case = [];
    if isprop(vehicles(1), 'center_communication') && ...
            ~isempty(vehicles(1).center_communication) && ...
            isprop(vehicles(1).center_communication, 'attack_module')
        attack_module_case = vehicles(1).center_communication.attack_module;
    end

    if ~isempty(attack_module_case)
        attack_module_case.plotAttackValues('data_plot', 'local');
        title(sprintf('Case %d - Local Attack Values', selected_case_number));

        attack_module_case.plotAttackValues('data_plot', 'global');
        title(sprintf('Case %d - Global Attack Values', selected_case_number));
    else
        warning('No attack_module found for this saved case. Skipping attack-value plots.');
    end
end

fprintf('Finished plotting Case %d.\n', selected_case_number);

function plot_all_trust_log_exclude_attacker_case(collected_car, attacker_id, case_number)
    nb_vehicles = length(collected_car);
    num_plots = max(1, nb_vehicles - 1);
    num_cols = ceil(sqrt(num_plots));
    num_rows = ceil(num_plots / num_cols);

    figure("Name", "Case " + num2str(case_number) + ...
        " Trust Values Excluding Attacker", "NumberTitle", "off");

    subplot_idx = 1;
    for host_id = 1:nb_vehicles
        if host_id == attacker_id
            continue;
        end

        time_steps = size(collected_car(host_id).trust_log, 2);
        dt_config = collected_car(host_id).scenarios_config.dt;
        time_vector = 0:dt_config:(time_steps - 1) * dt_config;

        subplot(num_rows, num_cols, subplot_idx);
        hold on;
        for target_id = 1:nb_vehicles
            plot(time_vector, squeeze(collected_car(host_id).trust_log(1, :, target_id)), ...
                'DisplayName', ['Vehicle ' num2str(target_id)], 'LineWidth', 1);
        end
        hold off;
        title(['Trust Log for V' num2str(host_id)]);
        legend show;
        grid on;
        subplot_idx = subplot_idx + 1;
    end

    xlabel('Time (s)');
    sgtitle(sprintf('Case %d - Trust Values Excluding Attacker V%d', ...
        case_number, attacker_id));
end

function plot_trip_trust_debug_case(collected_car, scenario_info, case_number, ...
        attacker_id, focus_target_id, trust_threshold)
    nb_vehicles = length(collected_car);
    [dt_config, attack_start_s, attack_end_s] = case_time_settings(collected_car, scenario_info);

    if isempty(focus_target_id) || focus_target_id < 1 || focus_target_id > nb_vehicles
        focus_target_id = attacker_id;
    end

    fprintf('\n%s\n', repmat('=', 1, 118));
    fprintf('TrIP trust debug summary - Case %d, focus target V%d, attack window %.3f-%.3fs\n', ...
        case_number, focus_target_id, attack_start_s, attack_end_s);
    fprintf('%s\n', repmat('-', 1, 118));
    fprintf('%-8s %-8s | %8s %8s %8s | %-14s %8s | %8s %8s %8s | %-14s %8s\n', ...
        'Host', 'Target', 'N_Final', 'N_Local', 'N_Global', 'N_Bottleneck', 'N_Value', ...
        'A_Final', 'A_Local', 'A_Global', 'A_Bottleneck', 'A_Value');
    fprintf('%s\n', repmat('-', 1, 118));

    focus_host_ids = [];
    focus_series = {};
    pair_labels = {};
    normal_component_means = [];
    attack_component_means = [];
    component_labels = trust_component_labels();

    for host_id = 1:nb_vehicles
        for target_id = 1:nb_vehicles
            if host_id == target_id
                continue;
            end

            trust_model = get_trip_model_for_pair(collected_car, host_id, target_id);
            if isempty(trust_model)
                continue;
            end

            series = collect_trip_debug_series(trust_model, dt_config);
            if series.n == 0
                continue;
            end

            normal_mask = series.time < attack_start_s;
            attack_mask = series.time >= attack_start_s & series.time <= attack_end_s;
            if ~any(normal_mask)
                normal_mask = series.time <= attack_start_s;
            end

            normal_final = finite_mean(series.final(normal_mask));
            normal_local = finite_mean(series.local(normal_mask));
            normal_global = finite_mean(series.global(normal_mask));
            attack_final = finite_mean(series.final(attack_mask));
            attack_local = finite_mean(series.local(attack_mask));
            attack_global = finite_mean(series.global(attack_mask));

            [normal_bottle, normal_bottle_value] = trust_bottleneck(series, normal_mask);
            [attack_bottle, attack_bottle_value] = trust_bottleneck(series, attack_mask);
            fprintf('V%-7d V%-7d | %8.3f %8.3f %8.3f | %-14s %8.3f | %8.3f %8.3f %8.3f | %-14s %8.3f\n', ...
                host_id, target_id, normal_final, normal_local, normal_global, ...
                normal_bottle, normal_bottle_value, attack_final, attack_local, ...
                attack_global, attack_bottle, attack_bottle_value);

            pair_labels{end + 1} = sprintf('V%d->V%d', host_id, target_id); %#ok<AGROW>
            normal_component_means(end + 1, :) = component_mean_row(series, normal_mask); %#ok<AGROW>
            attack_component_means(end + 1, :) = component_mean_row(series, attack_mask); %#ok<AGROW>

            if target_id == focus_target_id
                focus_host_ids(end + 1) = host_id; %#ok<AGROW>
                focus_series{end + 1} = series; %#ok<AGROW>
            end
        end
    end
    fprintf('%s\n\n', repmat('=', 1, 118));

    if isempty(pair_labels)
        warning('No TrIP trust model logs found for Case %d.', case_number);
        return;
    end

    plot_component_mean_heatmaps(case_number, normal_component_means, ...
        attack_component_means, pair_labels, component_labels);

    if isempty(focus_series)
        warning('No TrIP trust logs found for focus target V%d.', focus_target_id);
        return;
    end

    plot_focus_trust_breakdown(case_number, focus_target_id, focus_host_ids, ...
        focus_series, attack_start_s, attack_end_s, trust_threshold);
    plot_focus_bottleneck_heatmap(case_number, focus_target_id, focus_host_ids, ...
        focus_series, attack_start_s, attack_end_s);
    plot_focus_discrepancy(case_number, focus_target_id, focus_host_ids, ...
        focus_series, attack_start_s, attack_end_s);
end

function [dt_config, attack_start_s, attack_end_s] = case_time_settings(collected_car, scenario_info)
    dt_config = 0.01;
    if ~isempty(collected_car) && isprop(collected_car(1), 'scenarios_config') && ...
            ~isempty(collected_car(1).scenarios_config)
        dt_config = collected_car(1).scenarios_config.dt;
    end

    attack_start_s = 10;
    attack_end_s = 15;
    if isstruct(scenario_info)
        if isfield(scenario_info, 'dt') && ~isempty(scenario_info.dt)
            dt_config = scenario_info.dt;
        end
        if isfield(scenario_info, 't_start') && ~isempty(scenario_info.t_start)
            attack_start_s = scenario_info.t_start;
        end
        if isfield(scenario_info, 't_end') && ~isempty(scenario_info.t_end)
            attack_end_s = scenario_info.t_end;
        end
    end
end

function trust_model = get_trip_model_for_pair(collected_car, host_id, target_id)
    trust_model = [];
    if host_id > length(collected_car) || ~isprop(collected_car(host_id), 'trip_models')
        return;
    end
    trip_models = collected_car(host_id).trip_models;
    if numel(trip_models) >= target_id && ~isempty(trip_models{target_id})
        trust_model = trip_models{target_id};
    end
end

function series = collect_trip_debug_series(trust_model, dt_config)
    field_map = {
        'final',        'final_score_log';
        'local',        'trust_sample_log';
        'gamma_cross',  'gamma_cross_log';
        'gamma_local',  'gamma_local_log';
        'gamma_self',   'gamma_local_our_self_log';
        'v',            'v_score_log';
        'd',            'd_score_log';
        'a',            'a_score_log';
        'h',            'h_score_log';
        'beacon',       'beacon_score_log';
        'beacon_local', 'beacon_score_local_log';
        'beacon_global','beacon_score_global_log';
        'physical',     'physical_valid_log';
        'temporal',     'temporal_score_log';
        'D_pos',        'D_pos_log';
        'D_vel',        'D_vel_log';
        'D_theta',      'D_theta_log';
        'D_acc',        'D_acc_log';
        'D_total',      'D_total_log';
        'flag_attack',  'flag_taget_attk_log';
        'flag_global',  'flag_glob_est_check_log';
        'flag_local',   'flag_local_est_check_log';
    };

    raw_values = cell(size(field_map, 1), 1);
    n = 0;
    for idx = 1:size(field_map, 1)
        raw_values{idx} = get_trip_log(trust_model, field_map{idx, 2});
        n = max(n, numel(raw_values{idx}));
    end

    series = struct();
    series.n = n;
    if n > 0
        series.time = (0:(n - 1)) * dt_config;
    else
        series.time = [];
    end
    for idx = 1:size(field_map, 1)
        series.(field_map{idx, 1}) = pad_log(raw_values{idx}, n);
    end

    series.global = series.gamma_cross .* series.gamma_local;
end

function values = get_trip_log(trust_model, field_name)
    values = [];
    if isempty(trust_model)
        return;
    end
    if isobject(trust_model)
        if isprop(trust_model, field_name)
            values = trust_model.(field_name);
        end
    elseif isstruct(trust_model) && isfield(trust_model, field_name)
        values = trust_model.(field_name);
    end
    if isempty(values)
        return;
    end
    values = double(values(:))';
    values(~isfinite(values)) = NaN;
end

function padded = pad_log(values, n)
    padded = NaN(1, n);
    if isempty(values) || n == 0
        return;
    end
    m = min(numel(values), n);
    padded(1:m) = values(1:m);
end

function labels = trust_component_labels()
    labels = {'v', 'd', 'a', 'h', 'beaconL', 'beaconG', ...
        'gammaCross', 'gammaLocal', 'gammaSelf'};
end

function row = component_mean_row(series, mask)
    names = {'v', 'd', 'a', 'h', 'beacon_local', 'beacon_global', ...
        'gamma_cross', 'gamma_local', 'gamma_self'};
    row = NaN(1, numel(names));
    for idx = 1:numel(names)
        row(idx) = finite_mean(series.(names{idx})(mask));
    end
end

function [label, value] = trust_bottleneck(series, mask)
    labels = trust_component_labels();
    means = component_mean_row(series, mask);
    valid = isfinite(means);
    if ~any(valid)
        label = 'n/a';
        value = NaN;
        return;
    end
    valid_indices = find(valid);
    [value, local_idx] = min(means(valid));
    label = labels{valid_indices(local_idx)};
end

function value = finite_mean(values)
    values = values(isfinite(values));
    if isempty(values)
        value = NaN;
    else
        value = mean(values);
    end
end

function plot_component_mean_heatmaps(case_number, normal_means, attack_means, pair_labels, component_labels)
    figure("Name", sprintf("Case %d Trust Bottleneck Component Means", case_number), ...
        "NumberTitle", "off", "Position", [80, 80, 1250, 650]);

    subplot(1, 2, 1);
    imagesc(normal_means, [0, 1]);
    colorbar;
    title('Normal window mean scores');
    set(gca, 'XTick', 1:numel(component_labels), 'XTickLabel', component_labels, ...
        'YTick', 1:numel(pair_labels), 'YTickLabel', pair_labels);
    xtickangle(45);
    grid on;

    subplot(1, 2, 2);
    imagesc(attack_means, [0, 1]);
    colorbar;
    title('Attack window mean scores');
    set(gca, 'XTick', 1:numel(component_labels), 'XTickLabel', component_labels, ...
        'YTick', 1:numel(pair_labels), 'YTickLabel', pair_labels);
    xtickangle(45);
    grid on;

    sgtitle(sprintf('Case %d - Trust Bottleneck Heatmap (low value = limiting trust)', case_number));
end

function plot_focus_trust_breakdown(case_number, focus_target_id, host_ids, ...
        focus_series, attack_start_s, attack_end_s, trust_threshold)
    num_hosts = numel(host_ids);
    figure("Name", sprintf("Case %d Trust Debug Focus V%d", case_number, focus_target_id), ...
        "NumberTitle", "off", "Position", [100, 60, 1450, max(500, 260 * num_hosts)]);

    for row_idx = 1:num_hosts
        host_id = host_ids(row_idx);
        series = focus_series{row_idx};

        subplot(num_hosts, 3, (row_idx - 1) * 3 + 1);
        plot(series.time, series.final, 'LineWidth', 1.5, 'DisplayName', 'Final');
        hold on;
        plot(series.time, series.local, 'LineWidth', 1.2, 'DisplayName', 'Local');
        plot(series.time, series.global, 'LineWidth', 1.2, 'DisplayName', 'GammaCross*GammaLocal');
        yline(trust_threshold, 'r:', 'DisplayName', 'Threshold');
        apply_trust_axes(sprintf('V%d -> V%d trust', host_id, focus_target_id), ...
            attack_start_s, attack_end_s);
        legend('show', 'Location', 'best');

        subplot(num_hosts, 3, (row_idx - 1) * 3 + 2);
        plot(series.time, series.v, 'DisplayName', 'v');
        hold on;
        plot(series.time, series.d, 'DisplayName', 'd');
        plot(series.time, series.a, 'DisplayName', 'a');
        % plot(series.time, series.h, 'DisplayName', 'h');
        plot(series.time, series.beacon_local, '--', 'DisplayName', 'beacon local');
        plot(series.time, series.beacon_global, '--', 'DisplayName', 'beacon global');
        yline(trust_threshold, 'r:', 'HandleVisibility', 'off');
        apply_trust_axes(sprintf('V%d -> V%d local components', host_id, focus_target_id), ...
            attack_start_s, attack_end_s);
        legend('show', 'Location', 'best');

        subplot(num_hosts, 3, (row_idx - 1) * 3 + 3);
        plot(series.time, series.gamma_cross, 'DisplayName', 'gamma cross');
        hold on;
        plot(series.time, series.gamma_local, 'DisplayName', 'gamma local');
        plot(series.time, series.gamma_self, '--', 'DisplayName', 'gamma self');
        yline(trust_threshold, 'r:', 'HandleVisibility', 'off');
        apply_trust_axes(sprintf('V%d -> V%d global factors', host_id, focus_target_id), ...
            attack_start_s, attack_end_s);
        legend('show', 'Location', 'best');
    end

    sgtitle(sprintf('Case %d - TrIP Trust Debug for Target V%d', case_number, focus_target_id));
end

function plot_focus_bottleneck_heatmap(case_number, focus_target_id, host_ids, ...
        focus_series, attack_start_s, attack_end_s)
    labels = trust_component_labels();
    num_hosts = numel(host_ids);
    max_n = 0;
    dt = 0.01;
    for idx = 1:num_hosts
        max_n = max(max_n, focus_series{idx}.n);
        if focus_series{idx}.n > 1
            dt = focus_series{idx}.time(2) - focus_series{idx}.time(1);
        end
    end

    bottleneck_idx = NaN(num_hosts, max_n);
    for row_idx = 1:num_hosts
        series = focus_series{row_idx};
        values = [
            pad_log(series.v, max_n);
            pad_log(series.d, max_n);
            pad_log(series.a, max_n);
            % pad_log(series.h, max_n);
            pad_log(series.beacon_local, max_n);
            pad_log(series.beacon_global, max_n);
            pad_log(series.gamma_cross, max_n);
            pad_log(series.gamma_local, max_n);
            pad_log(series.gamma_self, max_n)
        ];
        for sample_idx = 1:max_n
            sample_values = values(:, sample_idx);
            valid = isfinite(sample_values);
            if any(valid)
                valid_indices = find(valid);
                [~, local_min_idx] = min(sample_values(valid));
                bottleneck_idx(row_idx, sample_idx) = valid_indices(local_min_idx);
            end
        end
    end

    time_vector = (0:(max_n - 1)) * dt;
    figure("Name", sprintf("Case %d Focus V%d Bottleneck Timeline", case_number, focus_target_id), ...
        "NumberTitle", "off", "Position", [120, 120, 1200, 420]);
    imagesc(time_vector, 1:num_hosts, bottleneck_idx);
    colormap(parula(numel(labels)));
    caxis([0.5, numel(labels) + 0.5]);
    cb = colorbar;
    cb.Ticks = 1:numel(labels);
    cb.TickLabels = labels;
    set(gca, 'YTick', 1:num_hosts, ...
        'YTickLabel', arrayfun(@(v) sprintf('V%d', v), host_ids, 'UniformOutput', false));
    xlabel('Time (s)');
    ylabel('Host vehicle');
    title(sprintf('Case %d - Lowest Trust Contributor for Target V%d', case_number, focus_target_id));
    add_attack_xlines(attack_start_s, attack_end_s);
    grid on;
end

function plot_focus_discrepancy(case_number, focus_target_id, host_ids, ...
        focus_series, attack_start_s, attack_end_s)
    has_discrepancy = false;
    for idx = 1:numel(focus_series)
        series = focus_series{idx};
        has_discrepancy = has_discrepancy || any(isfinite(series.D_pos)) || ...
            any(isfinite(series.D_vel)) || any(isfinite(series.D_theta)) || ...
            any(isfinite(series.D_acc)) || any(isfinite(series.D_total));
    end

    figure("Name", sprintf("Case %d Focus V%d Global Discrepancy", case_number, focus_target_id), ...
        "NumberTitle", "off", "Position", [140, 140, 1200, max(420, 230 * numel(host_ids))]);

    if ~has_discrepancy
        axis off;
        text(0.05, 0.65, 'No D\_pos/D\_theta/D\_vel/D\_acc logs are available in this saved case.', ...
            'FontSize', 12, 'Interpreter', 'none');
        text(0.05, 0.50, 'Rerun main_mean_attack_plot.m after this update to populate global discrepancy logs.', ...
            'FontSize', 12, 'Interpreter', 'none');
        title(sprintf('Case %d - Global Discrepancy Logs Missing for Target V%d', ...
            case_number, focus_target_id));
        return;
    end

    for row_idx = 1:numel(host_ids)
        host_id = host_ids(row_idx);
        series = focus_series{row_idx};
        subplot(numel(host_ids), 1, row_idx);
        plot(series.time, series.D_total, 'k', 'LineWidth', 1.3, 'DisplayName', 'D total');
        hold on;
        plot(series.time, series.D_pos, 'DisplayName', 'D pos');
        plot(series.time, series.D_theta, 'DisplayName', 'D theta');
        plot(series.time, series.D_vel, 'DisplayName', 'D vel');
        plot(series.time, series.D_acc, 'DisplayName', 'D acc');
        ylabel('D');
        title(sprintf('Global Mahalanobis terms V%d -> V%d', host_id, focus_target_id));
        add_attack_xlines(attack_start_s, attack_end_s);
        grid on;
        legend('show', 'Location', 'best');
    end
    xlabel('Time (s)');
    sgtitle(sprintf('Case %d - Global Trust Discrepancy Terms for Target V%d', ...
        case_number, focus_target_id));
end

function apply_trust_axes(title_text, attack_start_s, attack_end_s)
    ylim([0, 1.05]);
    xlabel('Time (s)');
    ylabel('Trust / score');
    title(title_text);
    grid on;
    add_attack_xlines(attack_start_s, attack_end_s);
end

function add_attack_xlines(attack_start_s, attack_end_s)
    if isfinite(attack_start_s)
        xline(attack_start_s, 'k--', 'HandleVisibility', 'off');
    end
    if isfinite(attack_end_s)
        xline(attack_end_s, 'k--', 'HandleVisibility', 'off');
    end
end
