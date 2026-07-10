%% Plot clear trust-weight channels from saved simulation logs.
% This script reads logs/simulation_logs.mat directly and separates:
%   1) w0: direct/local anchor weight for the attacked vehicle target
%   2) w_self: observer host self/prediction weight for the attacked target
%   3) w_other_global: attacker-as-global-source weight used for other targets
%
% It avoids the older TrustWeightStats source-influence columns, which mix
% direct w0 and source influence through a max() operation.

clear;
close all;
clc;

script_dir = fileparts(mfilename('fullpath'));
results_root = fullfile(script_dir, 'results');

attacker_vehicle_id = 1;
attack_type = "Mix_test";
channels_to_plot = ["local", "global"];
t_star = 10;
t_end = 15;

% Plot in MATLAB by default. Set save_outputs = true when you want PNG/EPS/FIG
% and CSV files written into each run's figures folder.
show_figures = true;
save_outputs = false;
save_outputs = save_outputs || strcmpi(getenv('PLOT_SAVED_TRUST_WEIGHT_SAVE'), 'true');

for channel_idx = 1:numel(channels_to_plot)
    channel_name = channels_to_plot(channel_idx);
    run_dir = find_latest_run_dir(results_root, channel_name, attacker_vehicle_id, attack_type);
    log_file = fullfile(run_dir, 'logs', 'simulation_logs.mat');

    if ~exist(log_file, 'file')
        warning('Missing simulation log: %s', log_file);
        continue;
    end

    data = load(log_file, 'all_case_trust_logs', 'run_metadata');
    traces = extract_weight_channel_traces(data.all_case_trust_logs, ...
        data.run_metadata, attacker_vehicle_id, t_star, t_end);

    plot_weight_channels(traces, channel_name, attacker_vehicle_id, ...
        attack_type, t_star, t_end, show_figures, save_outputs, run_dir);
    plot_attack_window_channel_bars(traces, channel_name, attacker_vehicle_id, ...
        attack_type, show_figures, save_outputs, run_dir);

    if save_outputs
        out_base = output_base(run_dir, channel_name, attacker_vehicle_id);
        write_weight_channel_summary(traces, out_base);
        fprintf('Wrote trust-weight channel figure: %s.png\n', out_base);
        fprintf('Wrote attack-window channel bar figure: %s_attack_window_bars.png\n', out_base);
        fprintf('Wrote trust-weight channel summary: %s_summary.csv\n', out_base);
    else
        fprintf('Displayed MATLAB figures for %s attacker V%d from: %s\n', ...
            char(channel_name), attacker_vehicle_id, run_dir);
    end
end

function run_dir = find_latest_run_dir(results_root, channel_name, attacker_id, attack_type)
    group_dir = fullfile(results_root, char(channel_name), ...
        sprintf('attacker_V%d', attacker_id), char(attack_type));
    if ~exist(group_dir, 'dir')
        error('Result group folder does not exist: %s', group_dir);
    end

    entries = dir(group_dir);
    entries = entries([entries.isdir]);
    names = {entries.name};
    keep = ~ismember(names, {'.', '..'});
    entries = entries(keep);
    if isempty(entries)
        error('No run folders found under: %s', group_dir);
    end

    [~, order] = sort([entries.datenum], 'descend');
    run_dir = fullfile(group_dir, entries(order(1)).name);
end

function traces = extract_weight_channel_traces(all_case_trust_logs, run_metadata, attacker_id, t_star, t_end)
    num_cases = numel(all_case_trust_logs);
    if isfield(run_metadata, 'dt')
        dt = run_metadata.dt;
    else
        dt = 0.01;
    end

    if isfield(run_metadata, 'attack_case_numbers')
        case_numbers = run_metadata.attack_case_numbers;
    else
        case_numbers = 1:num_cases;
    end

    traces = struct();
    traces.dt = dt;
    traces.case_numbers = case_numbers;
    traces.time = cell(num_cases, 1);
    traces.w0 = cell(num_cases, 1);
    traces.w_self = cell(num_cases, 1);
    traces.w_other_global = cell(num_cases, 1);
    traces.summary = cell(num_cases + 1, 8);
    traces.summary(1, :) = {'Case', 'w0_Pre', 'w0_Attack', ...
        'wSelf_Pre', 'wSelf_Attack', 'wOtherGlobal_Pre', ...
        'wOtherGlobal_Attack', 'wOtherGlobal_ZeroRate_Attack'};

    for case_idx = 1:num_cases
        case_log = all_case_trust_logs{case_idx};
        observer_ids = detect_observer_ids(case_log);
        observer_ids = observer_ids(observer_ids ~= attacker_id);
        if isempty(observer_ids)
            continue;
        end

        max_steps = 0;
        for observer_id = observer_ids
            observer_field = sprintf('observer_v%d', observer_id);
            if isfield(case_log, observer_field) && isfield(case_log.(observer_field), 'target_weights')
                max_steps = max(max_steps, size(case_log.(observer_field).target_weights, 2));
            end
        end
        if max_steps == 0
            continue;
        end

        direct_w0 = NaN(numel(observer_ids), max_steps);
        self_weight = NaN(numel(observer_ids), max_steps);
        attacker_source_weight = NaN(numel(observer_ids), max_steps);

        for observer_idx = 1:numel(observer_ids)
            observer_id = observer_ids(observer_idx);
            observer_field = sprintf('observer_v%d', observer_id);
            if ~isfield(case_log, observer_field) || ...
                    ~isfield(case_log.(observer_field), 'target_weights')
                continue;
            end

            weights = case_log.(observer_field).target_weights;
            n_steps = size(weights, 2);
            n_sources = size(weights, 1);
            n_targets = size(weights, 3);

            if attacker_id <= n_targets
                direct_w0(observer_idx, 1:n_steps) = squeeze(weights(1, :, attacker_id));
                self_source_idx = observer_id + 1;
                if self_source_idx <= n_sources
                    self_weight(observer_idx, 1:n_steps) = squeeze(weights(self_source_idx, :, attacker_id));
                end
            end

            attacker_source_idx = attacker_id + 1;
            if attacker_source_idx <= n_sources
                source_matrix = squeeze(weights(attacker_source_idx, :, :));
                source_matrix = normalize_source_matrix(source_matrix, n_steps);
                if attacker_id <= size(source_matrix, 2)
                    source_matrix(:, attacker_id) = NaN;
                end
                source_trace = row_nanmax(source_matrix);
                attacker_source_weight(observer_idx, 1:n_steps) = source_trace(:).';
            end
        end

        traces.time{case_idx} = (1:max_steps) * dt;
        traces.w0{case_idx} = mean(direct_w0, 1, 'omitnan');
        traces.w_self{case_idx} = mean(self_weight, 1, 'omitnan');
        traces.w_other_global{case_idx} = mean(attacker_source_weight, 1, 'omitnan');

        pre_idx = 1:max(1, round(t_star / dt) - 1);
        attack_idx = max(1, round(t_star / dt)):min(max_steps, round(t_end / dt));
        attack_values = attacker_source_weight(:, attack_idx);
        attack_values = attack_values(isfinite(attack_values));
        if isempty(attack_values)
            zero_rate = NaN;
        else
            zero_rate = mean(attack_values <= 1e-9);
        end

        traces.summary(case_idx + 1, :) = { ...
            sprintf('Case %d', case_numbers(case_idx)), ...
            mean(traces.w0{case_idx}(pre_idx), 'omitnan'), ...
            mean(traces.w0{case_idx}(attack_idx), 'omitnan'), ...
            mean(traces.w_self{case_idx}(pre_idx), 'omitnan'), ...
            mean(traces.w_self{case_idx}(attack_idx), 'omitnan'), ...
            mean(traces.w_other_global{case_idx}(pre_idx), 'omitnan'), ...
            mean(traces.w_other_global{case_idx}(attack_idx), 'omitnan'), ...
            zero_rate};
    end
end

function observer_ids = detect_observer_ids(case_log)
    fields = fieldnames(case_log);
    observer_ids = [];
    for idx = 1:numel(fields)
        tokens = regexp(fields{idx}, '^observer_v(\d+)$', 'tokens', 'once');
        if ~isempty(tokens)
            observer_ids(end + 1) = str2double(tokens{1}); %#ok<AGROW>
        end
    end
    observer_ids = unique(observer_ids);
end

function source_matrix = normalize_source_matrix(source_matrix, n_steps)
    if isvector(source_matrix)
        source_matrix = source_matrix(:);
    end
    if size(source_matrix, 1) ~= n_steps && size(source_matrix, 2) == n_steps
        source_matrix = source_matrix.';
    end
    if size(source_matrix, 1) ~= n_steps
        source_matrix = reshape(source_matrix, n_steps, []);
    end
end

function row_max = row_nanmax(values)
    row_max = NaN(size(values, 1), 1);
    for row_idx = 1:size(values, 1)
        row_values = values(row_idx, :);
        row_values = row_values(isfinite(row_values));
        if ~isempty(row_values)
            row_max(row_idx) = max(row_values);
        end
    end
end

function out_base = output_base(run_dir, channel_name, attacker_id)
    figures_dir = fullfile(run_dir, 'figures');
    if ~exist(figures_dir, 'dir')
        mkdir(figures_dir);
    end
    out_base = fullfile(figures_dir, sprintf('trust_weight_channels_%s_attacker_V%d', ...
        char(channel_name), attacker_id));
end

function plot_weight_channels(traces, channel_name, attacker_id, attack_type, ...
        t_star, t_end, show_figures, save_outputs, run_dir)
    num_cases = numel(traces.case_numbers);
    colors = lines(num_cases);
    case_labels = arrayfun(@(case_id) sprintf('Case %d', case_id), ...
        traces.case_numbers, 'UniformOutput', false);

    if show_figures
        visible_mode = 'on';
    else
        visible_mode = 'off';
    end
    fig = figure('Name', sprintf('trust_weight_channels_%s_attacker_V%d', ...
        char(channel_name), attacker_id), ...
        'Color', 'w', 'Position', [100, 80, 1200, 900], 'Visible', visible_mode);

    channel_data = {traces.w0, traces.w_self, traces.w_other_global};
    ylabels = { ...
        sprintf('w0 direct for V%d', attacker_id), ...
        sprintf('w self for V%d target', attacker_id), ...
        sprintf('w other global from V%d', attacker_id)};
    titles = { ...
        'Direct local anchor weight for attacked target', ...
        'Host self/prediction weight while estimating attacked target', ...
        'Attacker global-source weight used for other targets'};

    for plot_idx = 1:3
        ax = subplot(3, 1, plot_idx);
        hold(ax, 'on');
        patch(ax, [t_star, t_end, t_end, t_star], [0, 0, 1.05, 1.05], ...
            [0.92, 0.92, 0.92], 'EdgeColor', 'none', ...
            'FaceAlpha', 0.65, 'HandleVisibility', 'off');

        for case_idx = 1:num_cases
            trace = channel_data{plot_idx}{case_idx};
            if isempty(trace) || isempty(traces.time{case_idx})
                continue;
            end
            plot(ax, traces.time{case_idx}, trace, 'LineWidth', 1.7, ...
                'Color', colors(case_idx, :));
        end

        if plot_idx == 1
            yline(ax, 0.4, ':', 'nominal w0=0.4', ...
                'Color', [0.25, 0.25, 0.25], 'HandleVisibility', 'off');
        end
        xlim(ax, [0, max(cellfun(@(x) max([x(:); 0]), traces.time))]);
        ylim(ax, [0, 1.05]);
        ylabel(ax, ylabels{plot_idx});
        title(ax, titles{plot_idx});
        grid(ax, 'on');
        box(ax, 'on');
        if plot_idx == 3
            xlabel(ax, 'Time (s)');
        else
            set(ax, 'XTickLabel', []);
        end
        if plot_idx == 1
            legend(ax, case_labels, 'Location', 'eastoutside');
        end
    end

    sgtitle(sprintf('%s attack, attacker V%d: separated trust-weight channels', ...
        char(channel_name), attacker_id), 'FontWeight', 'bold');

    if save_outputs
        out_base = output_base(run_dir, channel_name, attacker_id);
        savefig(fig, [out_base, '.fig']);
        try
            exportgraphics(fig, [out_base, '.png'], 'Resolution', 300);
        catch
            print(fig, [out_base, '.png'], '-dpng', '-r300');
        end
        print(fig, [out_base, '.eps'], '-depsc', '-vector', '-r300');
    end
    if ~show_figures
        close(fig);
    end

    fprintf('Channel meaning for %s/%s:\n', char(channel_name), char(attack_type));
    fprintf('  w0: direct local anchor for attacked vehicle V%d.\n', attacker_id);
    fprintf('  w_self: host observer self/prediction weight for target V%d.\n', attacker_id);
    fprintf('  w_other_global: attacker V%d as global source for other target rows.\n', attacker_id);
end

function write_weight_channel_summary(traces, out_base)
    writecell(traces.summary, [out_base, '_summary.csv']);
end

function plot_attack_window_channel_bars(traces, channel_name, attacker_id, ...
        attack_type, show_figures, save_outputs, run_dir)
    num_cases = numel(traces.case_numbers);
    case_labels = arrayfun(@(case_id) sprintf('Case %d', case_id), ...
        traces.case_numbers, 'UniformOutput', false);

    values = NaN(num_cases, 3);
    for case_idx = 1:num_cases
        row_idx = case_idx + 1;
        if row_idx <= size(traces.summary, 1)
            values(case_idx, :) = cell2mat(traces.summary(row_idx, [3, 5, 7]));
        end
    end

    if show_figures
        visible_mode = 'on';
    else
        visible_mode = 'off';
    end
    fig = figure('Name', sprintf('trust_weight_channels_%s_attacker_V%d_attack_window_bars', ...
        char(channel_name), attacker_id), ...
        'Color', 'w', 'Position', [160, 120, 1000, 520], 'Visible', visible_mode);
    ax = axes(fig);
    bar_handle = bar(ax, values, 'grouped');
    bar_handle(1).FaceColor = [0.0000, 0.4470, 0.7410];
    bar_handle(2).FaceColor = [0.8500, 0.3250, 0.0980];
    bar_handle(3).FaceColor = [0.4660, 0.6740, 0.1880];

    finite_values = values(isfinite(values));
    if isempty(finite_values)
        y_max = 1.05;
    else
        y_max = min(1.05, max(0.35, max(finite_values) + 0.15));
    end
    ylim(ax, [0, y_max]);
    xticks(ax, 1:num_cases);
    xticklabels(ax, case_labels);
    xlabel(ax, 'Attack case');
    ylabel(ax, 'Mean weight during attack window');
    title(ax, sprintf('%s/%s attacker V%d: separated attack-window weights', ...
        char(channel_name), char(attack_type), attacker_id), 'Interpreter', 'none');
    legend(ax, {'w0 direct', 'w self', 'w other global from attacker'}, ...
        'Location', 'northoutside', 'Orientation', 'horizontal');
    grid(ax, 'on');
    box(ax, 'on');

    for channel_idx = 1:size(values, 2)
        x_endpoints = bar_handle(channel_idx).XEndPoints;
        y_endpoints = bar_handle(channel_idx).YEndPoints;
        labels = arrayfun(@(v) sprintf('%.3f', v), values(:, channel_idx), ...
            'UniformOutput', false);
        text(ax, x_endpoints, y_endpoints + 0.025, labels, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
            'FontSize', 8);
    end

    if save_outputs
        out_base = output_base(run_dir, channel_name, attacker_id);
        bar_base = [out_base, '_attack_window_bars'];
        savefig(fig, [bar_base, '.fig']);
        try
            exportgraphics(fig, [bar_base, '.png'], 'Resolution', 300);
        catch
            print(fig, [bar_base, '.png'], '-dpng', '-r300');
        end
        print(fig, [bar_base, '.eps'], '-depsc', '-vector', '-r300');
    end
    if ~show_figures
        close(fig);
    end
end
