function diagnostics = plot_observer_weight_diagnostics(collected_car, attacker_id, ...
        attack_start_s, attack_end_s, trust_threshold, case_number, focus_target_id)
%PLOT_OBSERVER_WEIGHT_DIAGNOSTICS Python-style observer weight diagnostics.
% The observer weight layout is:
%   weights(1)              direct/local anchor w0
%   weights(vehicle_id + 1) source vehicle's global estimate
%
% The function focuses on one target, usually the attacker, and also shows
% whether the attacker is still used as a source for other targets.

    if nargin < 2 || isempty(attacker_id)
        attacker_id = 1;
    end
    if nargin < 3 || isempty(attack_start_s)
        attack_start_s = NaN;
    end
    if nargin < 4 || isempty(attack_end_s)
        attack_end_s = NaN;
    end
    if nargin < 5 || isempty(trust_threshold)
        trust_threshold = 0.5;
    end
    if nargin < 6 || isempty(case_number)
        case_number = NaN;
    end
    if nargin < 7 || isempty(focus_target_id)
        focus_target_id = attacker_id;
    end

    nb_vehicles = length(collected_car);
    if nb_vehicles == 0
        warning('No vehicles available for observer weight diagnostics.');
        diagnostics = struct();
        return;
    end

    if attacker_id < 1 || attacker_id > nb_vehicles
        attacker_id = 1;
    end
    if focus_target_id < 1 || focus_target_id > nb_vehicles
        focus_target_id = attacker_id;
    end

    dt_config = 0.01;
    if isprop(collected_car(1), 'scenarios_config') && ...
            ~isempty(collected_car(1).scenarios_config)
        dt_config = collected_car(1).scenarios_config.dt;
    end

    host_ids = setdiff(1:nb_vehicles, attacker_id);
    if isempty(host_ids)
        host_ids = 1:nb_vehicles;
    end

    time_steps = observer_weight_time_steps(collected_car, host_ids);
    if time_steps == 0
        warning('No trust or observer weight logs found. Skipping observer weight diagnostics.');
        diagnostics = struct();
        return;
    end

    time_vector = (0:(time_steps - 1)) * dt_config;
    num_hosts = numel(host_ids);

    diagnostics = struct();
    diagnostics.time = time_vector;
    diagnostics.host_ids = host_ids;
    diagnostics.attacker_id = attacker_id;
    diagnostics.focus_target_id = focus_target_id;
    diagnostics.attacker_trust = NaN(num_hosts, time_steps);
    diagnostics.focus_trust = NaN(num_hosts, time_steps);
    diagnostics.controller_gamma = NaN(num_hosts, time_steps);
    diagnostics.trusted_neighbor_count = NaN(num_hosts, time_steps);
    diagnostics.active_source_count = NaN(num_hosts, time_steps);
    diagnostics.focus_w0 = NaN(num_hosts, time_steps);
    diagnostics.focus_w_self = NaN(num_hosts, time_steps);
    diagnostics.focus_neighbor_sum = NaN(num_hosts, time_steps);
    diagnostics.focus_source_weights = NaN(num_hosts, time_steps, nb_vehicles);
    diagnostics.attacker_direct_weight = NaN(num_hosts, time_steps);
    diagnostics.attacker_as_source_max = NaN(num_hosts, time_steps);
    diagnostics.attacker_as_source_sum = NaN(num_hosts, time_steps);
    diagnostics.attacker_total_influence = NaN(num_hosts, time_steps);

    for row_idx = 1:num_hosts
        host_id = host_ids(row_idx);
        host_vehicle = collected_car(host_id);

        diagnostics.attacker_trust(row_idx, :) = read_trust_trace( ...
            host_vehicle, attacker_id, time_steps);
        diagnostics.focus_trust(row_idx, :) = read_trust_trace( ...
            host_vehicle, focus_target_id, time_steps);

        if isprop(host_vehicle, 'gamma_log') && ~isempty(host_vehicle.gamma_log)
            diagnostics.controller_gamma(row_idx, :) = pad_weight_log( ...
                double(host_vehicle.gamma_log(:))', time_steps);
        end

        diagnostics = collect_host_weight_traces(diagnostics, collected_car, ...
            host_id, row_idx, attacker_id, focus_target_id, time_steps);
        diagnostics.trusted_neighbor_count(row_idx, :) = trusted_neighbor_counts( ...
            host_vehicle, host_id, nb_vehicles, time_steps);
    end

    print_observer_weight_summary(diagnostics, case_number, ...
        attack_start_s, attack_end_s);
    plot_observer_weight_timeseries(diagnostics, case_number, ...
        attack_start_s, attack_end_s, trust_threshold);
    plot_focus_source_weight_heatmaps(diagnostics, case_number, ...
        attack_start_s, attack_end_s);
end

function time_steps = observer_weight_time_steps(collected_car, host_ids)
    time_steps = 0;
    for host_id = host_ids
        host_vehicle = collected_car(host_id);
        if isprop(host_vehicle, 'trust_log') && ~isempty(host_vehicle.trust_log)
            time_steps = max(time_steps, size(host_vehicle.trust_log, 2));
        end
        if isprop(host_vehicle, 'observer') && ~isempty(host_vehicle.observer) && ...
                isprop(host_vehicle.observer, 'target_weights_log') && ...
                ~isempty(host_vehicle.observer.target_weights_log)
            time_steps = max(time_steps, size(host_vehicle.observer.target_weights_log, 2));
        end
    end
end

function trace = read_trust_trace(host_vehicle, target_id, time_steps)
    trace = NaN(1, time_steps);
    if ~isprop(host_vehicle, 'trust_log') || isempty(host_vehicle.trust_log) || ...
            size(host_vehicle.trust_log, 3) < target_id
        return;
    end
    raw_trace = squeeze(host_vehicle.trust_log(1, :, target_id));
    trace = pad_weight_log(double(raw_trace(:))', time_steps);
end

function diagnostics = collect_host_weight_traces(diagnostics, collected_car, ...
        host_id, row_idx, attacker_id, focus_target_id, time_steps)
    host_vehicle = collected_car(host_id);
    if ~isprop(host_vehicle, 'observer') || isempty(host_vehicle.observer) || ...
            ~isprop(host_vehicle.observer, 'target_weights_log') || ...
            isempty(host_vehicle.observer.target_weights_log)
        return;
    end

    target_weights_log = host_vehicle.observer.target_weights_log;
    source_dim = size(target_weights_log, 1);
    max_time = min(time_steps, size(target_weights_log, 2));
    target_dim = size(target_weights_log, 3);
    nb_vehicles = length(collected_car);

    for time_idx = 1:max_time
        if focus_target_id <= target_dim
            focus_weights = double(target_weights_log(:, time_idx, focus_target_id));
            focus_weights = focus_weights(:)';
            diagnostics.focus_w0(row_idx, time_idx) = read_weight_channel(focus_weights, 1);
            diagnostics.focus_w_self(row_idx, time_idx) = ...
                read_weight_channel(focus_weights, host_id + 1);

            source_values = NaN(1, nb_vehicles);
            for source_id = 1:min(nb_vehicles, source_dim - 1)
                source_values(source_id) = read_weight_channel(focus_weights, source_id + 1);
                diagnostics.focus_source_weights(row_idx, time_idx, source_id) = ...
                    source_values(source_id);
            end

            external_sources = source_values;
            if host_id <= numel(external_sources)
                external_sources(host_id) = NaN;
            end
            diagnostics.focus_neighbor_sum(row_idx, time_idx) = finite_sum(external_sources);
            diagnostics.active_source_count(row_idx, time_idx) = ...
                sum(isfinite(external_sources) & external_sources > 1e-9);
        end

        if attacker_id <= target_dim
            attacker_target_weights = double(target_weights_log(:, time_idx, attacker_id));
            attacker_target_weights = attacker_target_weights(:)';
            diagnostics.attacker_direct_weight(row_idx, time_idx) = ...
                read_weight_channel(attacker_target_weights, 1);
        end

        attacker_source_values = NaN(1, target_dim);
        attacker_source_idx = attacker_id + 1;
        if attacker_source_idx <= source_dim
            for target_id = 1:target_dim
                if target_id == attacker_id
                    continue;
                end
                attacker_source_values(target_id) = ...
                    target_weights_log(attacker_source_idx, time_idx, target_id);
            end
        end

        finite_attacker_sources = attacker_source_values(isfinite(attacker_source_values));
        if ~isempty(finite_attacker_sources)
            diagnostics.attacker_as_source_max(row_idx, time_idx) = ...
                max(finite_attacker_sources);
            diagnostics.attacker_as_source_sum(row_idx, time_idx) = ...
                sum(finite_attacker_sources);
        end

        attacker_channels = [
            diagnostics.attacker_direct_weight(row_idx, time_idx), ...
            diagnostics.attacker_as_source_max(row_idx, time_idx)
        ];
        attacker_channels = attacker_channels(isfinite(attacker_channels));
        if ~isempty(attacker_channels)
            diagnostics.attacker_total_influence(row_idx, time_idx) = max(attacker_channels);
        end
    end
end

function value = read_weight_channel(weights, index)
    if index >= 1 && index <= numel(weights) && isfinite(weights(index))
        value = weights(index);
    else
        value = NaN;
    end
end

function counts = trusted_neighbor_counts(host_vehicle, host_id, nb_vehicles, time_steps)
    counts = NaN(1, time_steps);
    if ~isprop(host_vehicle, 'weight_module') || isempty(host_vehicle.weight_module) || ...
            ~isprop(host_vehicle, 'trust_log') || isempty(host_vehicle.trust_log)
        return;
    end
    if ~ismethod(host_vehicle.weight_module, 'get_trusted_neighbors')
        return;
    end

    for time_idx = 1:min(time_steps, size(host_vehicle.trust_log, 2))
        trust_scores_now = squeeze(host_vehicle.trust_log(1, time_idx, :));
        if numel(trust_scores_now) ~= nb_vehicles
            continue;
        end
        trust_scores_now = double(trust_scores_now(:))';
        counts(time_idx) = numel(host_vehicle.weight_module.get_trusted_neighbors( ...
            host_id, trust_scores_now));
    end
end

function print_observer_weight_summary(diagnostics, case_number, ...
        attack_start_s, attack_end_s)
    normal_mask = diagnostics.time < attack_start_s;
    attack_mask = diagnostics.time >= attack_start_s & diagnostics.time <= attack_end_s;
    if ~any(normal_mask)
        normal_mask = diagnostics.time <= attack_start_s;
    end
    if ~any(attack_mask)
        attack_mask = true(size(diagnostics.time));
    end

    fprintf('\n%s\n', repmat('=', 1, 104));
    fprintf('Observer weight diagnostics - Case %s, focus target V%d, attacker V%d, attack %.3f-%.3fs\n', ...
        case_label(case_number), diagnostics.focus_target_id, diagnostics.attacker_id, ...
        attack_start_s, attack_end_s);
    fprintf('%s\n', repmat('-', 1, 104));
    fprintf('%-8s | %8s %8s | %8s %8s | %8s %8s | %8s %8s | %8s %8s\n', ...
        'Host', 'NTrust', 'ATrust', 'N_w0', 'A_w0', 'NV1Src', 'AV1Src', ...
        'NTotal', 'ATotal', 'NCount', 'ACount');
    fprintf('%s\n', repmat('-', 1, 104));

    for row_idx = 1:numel(diagnostics.host_ids)
        fprintf('V%-7d | %8.3f %8.3f | %8.3f %8.3f | %8.3f %8.3f | %8.3f %8.3f | %8.2f %8.2f\n', ...
            diagnostics.host_ids(row_idx), ...
            finite_mean(diagnostics.attacker_trust(row_idx, normal_mask)), ...
            finite_mean(diagnostics.attacker_trust(row_idx, attack_mask)), ...
            finite_mean(diagnostics.attacker_direct_weight(row_idx, normal_mask)), ...
            finite_mean(diagnostics.attacker_direct_weight(row_idx, attack_mask)), ...
            finite_mean(diagnostics.attacker_as_source_max(row_idx, normal_mask)), ...
            finite_mean(diagnostics.attacker_as_source_max(row_idx, attack_mask)), ...
            finite_mean(diagnostics.attacker_total_influence(row_idx, normal_mask)), ...
            finite_mean(diagnostics.attacker_total_influence(row_idx, attack_mask)), ...
            finite_mean(diagnostics.trusted_neighbor_count(row_idx, normal_mask)), ...
            finite_mean(diagnostics.trusted_neighbor_count(row_idx, attack_mask)));
    end
    fprintf('%s\n\n', repmat('=', 1, 104));
end

function plot_observer_weight_timeseries(diagnostics, case_number, ...
        attack_start_s, attack_end_s, trust_threshold)
    figure("Name", sprintf("Case %s Observer Weight Diagnostics V%d", ...
            case_label(case_number), diagnostics.focus_target_id), ...
        "NumberTitle", "off", "Position", [80, 60, 1450, 850]);

    subplot(3, 2, 1);
    plot_host_matrix(diagnostics.time, diagnostics.attacker_trust, ...
        diagnostics.host_ids, 'Trust');
    yline(trust_threshold, 'r:', 'DisplayName', 'Threshold');
    ylim([0, 1.05]);
    title(sprintf('Trust in attacker V%d', diagnostics.attacker_id));
    add_attack_xlines(attack_start_s, attack_end_s);

    subplot(3, 2, 2);
    plot_named_mean_traces(diagnostics.time, {
        diagnostics.focus_w0, 'w0 direct', '-.', [0.0000, 0.4470, 0.7410];
        diagnostics.focus_w_self, 'w self', '--', [0.0000, 0.0000, 0.0000];
        diagnostics.focus_neighbor_sum, 'neighbor sum', '-', [0.8500, 0.3250, 0.0980]
    });
    ylim([0, 1.05]);
    ylabel('Weight');
    title(sprintf('Final weights for target V%d, mean over hosts', ...
        diagnostics.focus_target_id));
    add_attack_xlines(attack_start_s, attack_end_s);

    subplot(3, 2, 3);
    plot_focus_source_weight_means(diagnostics);
    ylim([0, 1.05]);
    title(sprintf('Per-source weights used to estimate V%d', ...
        diagnostics.focus_target_id));
    add_attack_xlines(attack_start_s, attack_end_s);

    subplot(3, 2, 4);
    plot_named_mean_traces(diagnostics.time, {
        diagnostics.attacker_direct_weight, 'w0 for attacker target', '-.', [0.0000, 0.4470, 0.7410];
        diagnostics.attacker_as_source_max, 'attacker source max', '-', [0.8500, 0.3250, 0.0980];
        diagnostics.attacker_as_source_sum, 'attacker source sum', ':', [0.9290, 0.6940, 0.1250];
        diagnostics.attacker_total_influence, 'total attacker influence', '-', [0.6350, 0.0780, 0.1840]
    });
    ylim([0, 1.05]);
    ylabel('Weight');
    title(sprintf('Attacker V%d influence split', diagnostics.attacker_id));
    add_attack_xlines(attack_start_s, attack_end_s);

    subplot(3, 2, 5);
    plot_named_mean_traces(diagnostics.time, {
        diagnostics.trusted_neighbor_count, 'trusted graph neighbors', '-', [0.4660, 0.6740, 0.1880];
        diagnostics.active_source_count, 'active source weights to focus', '--', [0.4940, 0.1840, 0.5560]
    });
    max_count = max(1, length(diagnostics.host_ids) + 1);
    ylim([-0.1, max_count + 0.5]);
    ylabel('Count');
    title('Trusted and active observer-source counts');
    add_attack_xlines(attack_start_s, attack_end_s);

    subplot(3, 2, 6);
    plot_attack_window_weight_heatmap(diagnostics, attack_start_s, attack_end_s);

    sgtitle(sprintf('Case %s - Python-style Observer Weight Diagnostics, Focus V%d', ...
        case_label(case_number), diagnostics.focus_target_id));
end

function plot_focus_source_weight_heatmaps(diagnostics, case_number, ...
        attack_start_s, attack_end_s)
    if ~any(isfinite(diagnostics.focus_source_weights(:)))
        return;
    end

    num_hosts = numel(diagnostics.host_ids);
    nb_vehicles = size(diagnostics.focus_source_weights, 3);
    num_cols = ceil(sqrt(num_hosts));
    num_rows = ceil(num_hosts / num_cols);

    figure("Name", sprintf("Case %s Source Weight Heatmaps V%d", ...
            case_label(case_number), diagnostics.focus_target_id), ...
        "NumberTitle", "off", "Position", [110, 90, 1300, max(420, 320 * num_rows)]);

    for row_idx = 1:num_hosts
        host_id = diagnostics.host_ids(row_idx);
        source_matrix = reshape(diagnostics.focus_source_weights(row_idx, :, :), ...
            numel(diagnostics.time), nb_vehicles)';

        subplot(num_rows, num_cols, row_idx);
        imagesc(diagnostics.time, 1:nb_vehicles, source_matrix, [0, 1]);
        set(gca, 'YTick', 1:nb_vehicles, ...
            'YTickLabel', arrayfun(@(v) sprintf('V%d', v), 1:nb_vehicles, ...
            'UniformOutput', false));
        xlabel('Time (s)');
        ylabel('Source');
        title(sprintf('Host V%d source weights to target V%d', ...
            host_id, diagnostics.focus_target_id));
        colorbar;
        add_attack_xlines(attack_start_s, attack_end_s);
    end

    sgtitle(sprintf('Case %s - Per-Host Source Weight Heatmaps for Target V%d', ...
        case_label(case_number), diagnostics.focus_target_id));
end

function plot_host_matrix(time_vector, data_matrix, host_ids, y_label)
    hold on;
    colors = lines(max(1, size(data_matrix, 1)));
    for row_idx = 1:size(data_matrix, 1)
        if any(isfinite(data_matrix(row_idx, :)))
            plot(time_vector, data_matrix(row_idx, :), ...
                'Color', colors(row_idx, :), ...
                'LineWidth', 1.1, ...
                'DisplayName', sprintf('V%d', host_ids(row_idx)));
        end
    end

    mean_trace = finite_column_mean(data_matrix);
    if any(isfinite(mean_trace))
        plot(time_vector, mean_trace, 'k', 'LineWidth', 2.0, ...
            'DisplayName', 'Mean');
    end

    hold off;
    ylabel(y_label);
    xlabel('Time (s)');
    grid on;
    if any(isfinite(data_matrix(:)))
        legend('show', 'Location', 'eastoutside');
    end
end

function plot_named_mean_traces(time_vector, trace_specs)
    hold on;
    for idx = 1:size(trace_specs, 1)
        data_matrix = trace_specs{idx, 1};
        label = trace_specs{idx, 2};
        line_style = trace_specs{idx, 3};
        color = trace_specs{idx, 4};
        mean_trace = finite_column_mean(data_matrix);
        if any(isfinite(mean_trace))
            plot(time_vector, mean_trace, line_style, 'Color', color, ...
                'LineWidth', 1.7, 'DisplayName', label);
        end
    end
    hold off;
    xlabel('Time (s)');
    grid on;
    legend('show', 'Location', 'best');
end

function plot_focus_source_weight_means(diagnostics)
    hold on;
    nb_vehicles = size(diagnostics.focus_source_weights, 3);
    colors = lines(max(1, nb_vehicles));
    for source_id = 1:nb_vehicles
        source_matrix = diagnostics.focus_source_weights(:, :, source_id);
        source_mean = finite_column_mean(source_matrix);
        if any(isfinite(source_mean))
            line_width = 1.2;
            line_style = '-';
            if source_id == diagnostics.attacker_id
                line_width = 2.2;
                line_style = '--';
            end
            plot(diagnostics.time, source_mean, line_style, ...
                'Color', colors(source_id, :), 'LineWidth', line_width, ...
                'DisplayName', sprintf('source V%d', source_id));
        end
    end
    hold off;
    xlabel('Time (s)');
    ylabel('Weight');
    grid on;
    legend('show', 'Location', 'eastoutside');
end

function plot_attack_window_weight_heatmap(diagnostics, attack_start_s, attack_end_s)
    attack_mask = diagnostics.time >= attack_start_s & diagnostics.time <= attack_end_s;
    if ~any(attack_mask)
        attack_mask = true(size(diagnostics.time));
    end

    metric_labels = {
        sprintf('trust V%d', diagnostics.attacker_id), ...
        'w0 attacker', ...
        sprintf('V%d source max', diagnostics.attacker_id), ...
        'total influence', ...
        sprintf('w self V%d', diagnostics.focus_target_id), ...
        sprintf('neighbor sum V%d', diagnostics.focus_target_id)
    };
    metric_data = [
        row_window_mean(diagnostics.attacker_trust, attack_mask), ...
        row_window_mean(diagnostics.attacker_direct_weight, attack_mask), ...
        row_window_mean(diagnostics.attacker_as_source_max, attack_mask), ...
        row_window_mean(diagnostics.attacker_total_influence, attack_mask), ...
        row_window_mean(diagnostics.focus_w_self, attack_mask), ...
        row_window_mean(diagnostics.focus_neighbor_sum, attack_mask)
    ];

    imagesc(metric_data, [0, 1]);
    colorbar;
    set(gca, 'XTick', 1:numel(metric_labels), 'XTickLabel', metric_labels, ...
        'YTick', 1:numel(diagnostics.host_ids), ...
        'YTickLabel', arrayfun(@(v) sprintf('V%d', v), diagnostics.host_ids, ...
        'UniformOutput', false), 'TickLength', [0, 0]);
    xtickangle(35);
    xlabel('Metric');
    ylabel('Host');
    title('Attack-window mean weights');
    grid on;

    for row_idx = 1:size(metric_data, 1)
        for col_idx = 1:size(metric_data, 2)
            value = metric_data(row_idx, col_idx);
            if isfinite(value)
                text(col_idx, row_idx, sprintf('%.2f', value), ...
                    'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'middle', ...
                    'Color', 'black', 'FontWeight', 'bold', ...
                    'FontSize', 9);
            end
        end
    end
end

function row_means = row_window_mean(data_matrix, mask)
    row_means = NaN(size(data_matrix, 1), 1);
    for row_idx = 1:size(data_matrix, 1)
        row_means(row_idx) = finite_mean(data_matrix(row_idx, mask));
    end
end

function mean_trace = finite_column_mean(data_matrix)
    if ~ismatrix(data_matrix)
        data_matrix = squeeze(data_matrix);
        if ~ismatrix(data_matrix)
            data_matrix = reshape(data_matrix, size(data_matrix, 1), []);
        end
    end
    mean_trace = NaN(1, size(data_matrix, 2));
    for col_idx = 1:size(data_matrix, 2)
        mean_trace(col_idx) = finite_mean(data_matrix(:, col_idx));
    end
end

function value = finite_mean(values)
    values = values(isfinite(values));
    if isempty(values)
        value = NaN;
    else
        value = mean(values);
    end
end

function value = finite_sum(values)
    values = values(isfinite(values));
    if isempty(values)
        value = NaN;
    else
        value = sum(values);
    end
end

function padded = pad_weight_log(values, n)
    padded = NaN(1, n);
    if isempty(values) || n == 0
        return;
    end
    m = min(numel(values), n);
    padded(1:m) = values(1:m);
end

function add_attack_xlines(attack_start_s, attack_end_s)
    if isfinite(attack_start_s)
        xline(attack_start_s, 'k--', 'HandleVisibility', 'off');
    end
    if isfinite(attack_end_s)
        xline(attack_end_s, 'k--', 'HandleVisibility', 'off');
    end
end

function label = case_label(case_number)
    if isnumeric(case_number) && isscalar(case_number) && isfinite(case_number)
        label = sprintf('%d', case_number);
    elseif isstring(case_number) || ischar(case_number)
        label = char(case_number);
    else
        label = 'n/a';
    end
end
