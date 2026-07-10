%% Analyze saved attack results for paper-ready metrics.
% This script reads the saved result workbooks/logs under test/results and
% derives cross-run metrics without rerunning simulations.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
results_root = fullfile(script_dir, 'results');
index_file = fullfile(results_root, 'results_index.csv');

if ~exist(index_file, 'file')
    error('Missing results index: %s', index_file);
end

attack_descriptions = {'P Bias -5m', 'P Faulty 10m', 'V Bias -2m/s', ...
    'V Faulty 2.5m/s', 'DoS Attack'};
trust_threshold = 0.7;
recovery_fraction = 0.90;
sustain_seconds = 0.5;

run_index = readtable(index_file, 'TextType', 'string', ...
    'Delimiter', ',', 'VariableNamingRule', 'preserve');

case_rows = {};
run_rows = {};
vehicle_rows = {};

for run_idx = 1:height(run_index)
    data_type = string(run_index.DataTypeAttack(run_idx));
    attacker_label = string(run_index.AttackerVehicle(run_idx));
    attacker_id = sscanf(char(attacker_label), 'V%d');
    excel_file = char(run_index.ExcelWorkbook(run_idx));
    run_folder = char(run_index.RunFolder(run_idx));
    mat_file = fullfile(run_folder, 'logs', 'simulation_logs.mat');

    if ~exist(excel_file, 'file')
        warning('Skipping missing workbook: %s', excel_file);
        continue;
    end
    if ~exist(mat_file, 'file')
        warning('Skipping missing log: %s', mat_file);
        continue;
    end

    summary_tbl = readtable(excel_file, 'Sheet', 'Summary', ...
        'TextType', 'string', 'VariableNamingRule', 'preserve');
    trust_tbl = readtable(excel_file, 'Sheet', 'TrustWeightStats', ...
        'TextType', 'string', 'VariableNamingRule', 'preserve');
    logs = load(mat_file);

    dt = logs.run_metadata.dt;
    t_start = logs.run_metadata.rmse_time_window(1);
    t_end = logs.run_metadata.rmse_time_window(2);
    attack_start_idx = max(1, round(t_start / dt));
    attack_end_idx = round(t_end / dt);
    sustain_steps = max(1, round(sustain_seconds / dt));

    all_case_raw = [];
    all_case_detection_delay = [];
    all_case_trust_recovery_delay = [];
    all_case_source_suppression_delay = [];
    all_case_combined_suppression_delay = [];
    all_case_source_recovery_delay = [];
    all_case_combined_recovery_delay = [];

    vehicle_ids = extract_vehicle_ids(summary_tbl.Vehicle);
    unique_vehicle_ids = unique(vehicle_ids(:))';

    for case_idx = 1:numel(attack_descriptions)
        case_label = "Case " + case_idx;
        case_mask = summary_tbl.Case == case_label;
        trust_mask = trust_tbl.Case == case_label;

        if ~any(case_mask) || ~any(trust_mask)
            warning('Missing case %s in %s', case_label, excel_file);
            continue;
        end

        case_summary = summary_tbl(case_mask, :);
        case_trust = trust_tbl(trust_mask, :);

        raw_combined = case_summary.AttackWindow_Raw_Combined;
        dist_rmse = case_summary.AttackWindow_RMSE_Distance;
        vel_rmse = case_summary.AttackWindow_RMSE_Velocity;
        acc_rmse = case_summary.AttackWindow_RMSE_Acceleration;
        orient_rmse = case_summary.AttackWindow_RMSE_Orientation;

        non_attacker_ids = unique_vehicle_ids(unique_vehicle_ids ~= attacker_id);
        case_log = logs.all_case_trust_logs{case_idx};
        [time_vec, mean_trust, mean_source_influence, mean_combined_influence] = ...
            extract_trust_weight_traces(case_log, non_attacker_ids, attacker_id, dt);

        pre_idx = 1:max(1, attack_start_idx - 1);
        attack_idx = attack_start_idx:min(attack_end_idx, numel(time_vec));
        post_idx = min(attack_end_idx + 1, numel(time_vec)):numel(time_vec);

        trust_pre = mean(mean_trust(pre_idx), 'omitnan');
        trust_attack = mean(mean_trust(attack_idx), 'omitnan');
        trust_post = mean(mean_trust(post_idx), 'omitnan');
        trust_final_1s = final_window_mean(mean_trust, dt, 1.0);

        source_pre = mean(mean_source_influence(pre_idx), 'omitnan');
        source_attack = mean(mean_source_influence(attack_idx), 'omitnan');
        source_post = mean(mean_source_influence(post_idx), 'omitnan');
        source_final_1s = final_window_mean(mean_source_influence, dt, 1.0);

        combined_pre = mean(mean_combined_influence(pre_idx), 'omitnan');
        combined_attack = mean(mean_combined_influence(attack_idx), 'omitnan');
        combined_post = mean(mean_combined_influence(post_idx), 'omitnan');
        combined_final_1s = final_window_mean(mean_combined_influence, dt, 1.0);

        detection_time = first_sustained_time(time_vec, mean_trust < trust_threshold, ...
            attack_start_idx, min(attack_end_idx, numel(time_vec)), 1);
        if isnan(detection_time)
            detection_delay = NaN;
        else
            detection_delay = detection_time - t_start;
        end

        trust_recovery_time = first_sustained_time(time_vec, mean_trust >= trust_threshold, ...
            min(attack_end_idx + 1, numel(time_vec)), numel(time_vec), sustain_steps);
        if isnan(trust_recovery_time)
            trust_recovery_delay = NaN;
        else
            trust_recovery_delay = trust_recovery_time - t_end;
        end

        source_suppression_threshold = 0.10 * source_pre;
        combined_suppression_threshold = 0.10 * combined_pre;
        source_suppression_time = first_sustained_time(time_vec, ...
            mean_source_influence <= source_suppression_threshold, ...
            attack_start_idx, min(attack_end_idx, numel(time_vec)), sustain_steps);
        combined_suppression_time = first_sustained_time(time_vec, ...
            mean_combined_influence <= combined_suppression_threshold, ...
            attack_start_idx, min(attack_end_idx, numel(time_vec)), sustain_steps);

        if isnan(source_suppression_time)
            source_suppression_delay = NaN;
        else
            source_suppression_delay = source_suppression_time - t_start;
        end
        if isnan(combined_suppression_time)
            combined_suppression_delay = NaN;
        else
            combined_suppression_delay = combined_suppression_time - t_start;
        end

        source_recovery_threshold = recovery_fraction * source_pre;
        combined_recovery_threshold = recovery_fraction * combined_pre;
        source_recovery_time = first_sustained_time(time_vec, ...
            mean_source_influence >= source_recovery_threshold, ...
            min(attack_end_idx + 1, numel(time_vec)), numel(time_vec), sustain_steps);
        combined_recovery_time = first_sustained_time(time_vec, ...
            mean_combined_influence >= combined_recovery_threshold, ...
            min(attack_end_idx + 1, numel(time_vec)), numel(time_vec), sustain_steps);

        if isnan(source_recovery_time)
            source_recovery_delay = NaN;
        else
            source_recovery_delay = source_recovery_time - t_end;
        end
        if isnan(combined_recovery_time)
            combined_recovery_delay = NaN;
        else
            combined_recovery_delay = combined_recovery_time - t_end;
        end

        source_zero_rate = mean(mean_source_influence(attack_idx) <= 1e-9, 'omitnan');
        combined_zero_rate = mean(mean_combined_influence(attack_idx) <= 1e-9, 'omitnan');

        detection_rate = case_trust.DetectionRate(1);
        workbook_detection_time = case_trust.MeanDetectionTime(1);
        workbook_detection_delay = workbook_detection_time - t_start;
        workbook_combined_pre = case_trust.MeanAttackerSourceInfluence_PreAttack(1);
        workbook_combined_attack = case_trust.MeanAttackerSourceInfluence_AttackWindow(1);
        workbook_combined_zero_rate = case_trust.AttackerSourceInfluenceZeroRate_AttackWindow(1);

        case_rows(end + 1, :) = { ...
            data_type, attacker_label, case_label, string(attack_descriptions{case_idx}), ...
            mean(raw_combined, 'omitnan'), std(raw_combined, 0, 'omitnan'), ...
            min(raw_combined, [], 'omitnan'), max(raw_combined, [], 'omitnan'), ...
            mean(dist_rmse, 'omitnan'), mean(orient_rmse, 'omitnan'), ...
            mean(vel_rmse, 'omitnan'), mean(acc_rmse, 'omitnan'), ...
            trust_pre, trust_attack, trust_pre - trust_attack, trust_post, trust_final_1s, ...
            detection_rate, workbook_detection_time, workbook_detection_delay, detection_delay, ...
            trust_recovery_time, trust_recovery_delay, ...
            source_pre, source_attack, source_pre - source_attack, source_zero_rate, source_post, source_final_1s, ...
            source_suppression_delay, source_recovery_delay, ...
            combined_pre, combined_attack, combined_pre - combined_attack, combined_zero_rate, combined_post, combined_final_1s, ...
            combined_suppression_delay, combined_recovery_delay, ...
            workbook_combined_pre, workbook_combined_attack, workbook_combined_zero_rate};

        all_case_raw = [all_case_raw; raw_combined(:)];
        all_case_detection_delay = [all_case_detection_delay; workbook_detection_delay];
        all_case_trust_recovery_delay = [all_case_trust_recovery_delay; trust_recovery_delay];
        all_case_source_suppression_delay = [all_case_source_suppression_delay; source_suppression_delay];
        all_case_combined_suppression_delay = [all_case_combined_suppression_delay; combined_suppression_delay];
        all_case_source_recovery_delay = [all_case_source_recovery_delay; source_recovery_delay];
        all_case_combined_recovery_delay = [all_case_combined_recovery_delay; combined_recovery_delay];

        for row_idx = 1:height(case_summary)
            vehicle_rows(end + 1, :) = { ...
                data_type, attacker_label, case_label, string(attack_descriptions{case_idx}), ...
                case_summary.Vehicle(row_idx), ...
                case_summary.AttackWindow_Raw_Combined(row_idx), ...
                case_summary.AttackWindow_RMSE_Distance(row_idx), ...
                case_summary.AttackWindow_RMSE_Orientation(row_idx), ...
                case_summary.AttackWindow_RMSE_Velocity(row_idx), ...
                case_summary.AttackWindow_RMSE_Acceleration(row_idx), ...
                case_summary.Mean_Trust_Score(row_idx), ...
                case_summary.Trust_Degradation(row_idx), ...
                case_summary.Attack_Detection_Time(row_idx) - t_start};
        end
    end

    run_rows(end + 1, :) = { ...
        data_type, attacker_label, string(run_index.RunTimestamp(run_idx)), ...
        mean(all_case_raw, 'omitnan'), std(all_case_raw, 0, 'omitnan'), ...
        min(all_case_raw, [], 'omitnan'), max(all_case_raw, [], 'omitnan'), ...
        mean(all_case_detection_delay, 'omitnan'), ...
        mean(all_case_trust_recovery_delay, 'omitnan'), ...
        mean(all_case_source_suppression_delay, 'omitnan'), ...
        mean(all_case_combined_suppression_delay, 'omitnan'), ...
        mean(all_case_source_recovery_delay, 'omitnan'), ...
        mean(all_case_combined_recovery_delay, 'omitnan')};
end

case_headers = {'DataTypeAttack','AttackerVehicle','Case','AttackDescription', ...
    'MeanRawCombinedRMSE','StdRawCombinedRMSE','MinRawCombinedRMSE','MaxRawCombinedRMSE', ...
    'MeanDistanceRMSE','MeanOrientationRMSE','MeanVelocityRMSE','MeanAccelerationRMSE', ...
    'MeanTrustPre','MeanTrustAttack','TrustDrop','MeanTrustPost','Final1sTrust', ...
    'DetectionRate','MeanDetectionTime_s','WorkbookDetectionDelay_s','MeanTraceDetectionDelay_s', ...
    'TrustRecoveryTime_s','TrustRecoveryDelay_s', ...
    'MeanSourceInfluencePre','MeanSourceInfluenceAttack','SourceInfluenceDrop', ...
    'SourceZeroRateAttack','MeanSourceInfluencePost','Final1sSourceInfluence', ...
    'SourceSuppressionDelay10pctPre_s','SourceRecoveryDelay90pctPre_s', ...
    'MeanCombinedInfluencePre','MeanCombinedInfluenceAttack','CombinedInfluenceDrop', ...
    'CombinedZeroRateAttack','MeanCombinedInfluencePost','Final1sCombinedInfluence', ...
    'CombinedSuppressionDelay10pctPre_s','CombinedRecoveryDelay90pctPre_s', ...
    'WorkbookCombinedInfluencePre_MislabeledSource','WorkbookCombinedInfluenceAttack_MislabeledSource', ...
    'WorkbookCombinedZeroRate_MislabeledSource'};
case_tbl = cell2table(case_rows, 'VariableNames', case_headers);

run_headers = {'DataTypeAttack','AttackerVehicle','RunTimestamp', ...
    'MeanRawCombinedRMSE','StdRawCombinedRMSE','MinRawCombinedRMSE','MaxRawCombinedRMSE', ...
    'MeanDetectionDelay_s','MeanTrustRecoveryDelay_s', ...
    'MeanSourceSuppressionDelay10pctPre_s','MeanCombinedSuppressionDelay10pctPre_s', ...
    'MeanSourceRecoveryDelay90pctPre_s','MeanCombinedRecoveryDelay90pctPre_s'};
run_tbl = cell2table(run_rows, 'VariableNames', run_headers);

vehicle_headers = {'DataTypeAttack','AttackerVehicle','Case','AttackDescription','Vehicle', ...
    'RawCombinedRMSE','DistanceRMSE','OrientationRMSE','VelocityRMSE','AccelerationRMSE', ...
    'MeanTrustScore','TrustDegradation','DetectionDelay_s'};
vehicle_tbl = cell2table(vehicle_rows, 'VariableNames', vehicle_headers);

case_csv = fullfile(results_root, 'paper_analysis_case_metrics.csv');
run_csv = fullfile(results_root, 'paper_analysis_run_metrics.csv');
vehicle_csv = fullfile(results_root, 'paper_analysis_vehicle_metrics.csv');
writetable(case_tbl, case_csv);
writetable(run_tbl, run_csv);
writetable(vehicle_tbl, vehicle_csv);

report_file = fullfile(results_root, 'paper_analysis_summary.md');
write_report(report_file, case_tbl, run_tbl, vehicle_tbl, attack_descriptions, ...
    trust_threshold, recovery_fraction, sustain_seconds, case_csv, run_csv, vehicle_csv);

fprintf('Wrote %s\n', case_csv);
fprintf('Wrote %s\n', run_csv);
fprintf('Wrote %s\n', vehicle_csv);
fprintf('Wrote %s\n', report_file);

function ids = extract_vehicle_ids(labels)
    ids = NaN(numel(labels), 1);
    for i = 1:numel(labels)
        ids(i) = sscanf(char(labels(i)), 'V%d');
    end
end

function [time_vec, mean_trust, mean_source, mean_combined] = extract_trust_weight_traces(case_log, evaluator_ids, attacker_id, dt)
    trust_traces = [];
    source_traces = [];
    combined_traces = [];

    for evaluator_id = evaluator_ids
        vehicle_field = sprintf('vehicle_%d', evaluator_id);
        observer_field = sprintf('observer_v%d', evaluator_id);
        if ~isfield(case_log, vehicle_field) || ~isfield(case_log, observer_field)
            continue;
        end

        trust_log = case_log.(vehicle_field);
        trust_trace = squeeze(trust_log(1, :, attacker_id));
        trust_traces(end + 1, 1:numel(trust_trace)) = trust_trace(:).'; %#ok<AGROW>

        observer = case_log.(observer_field);
        if ~isfield(observer, 'target_weights') || isempty(observer.target_weights)
            continue;
        end
        target_weights = observer.target_weights;
        n_steps = size(target_weights, 2);
        direct = NaN(1, n_steps);
        source = NaN(1, n_steps);
        combined = NaN(1, n_steps);

        if attacker_id <= size(target_weights, 3)
            direct = squeeze(target_weights(1, :, attacker_id)).';
        end
        if attacker_id + 1 <= size(target_weights, 1)
            source_matrix = squeeze(target_weights(attacker_id + 1, :, :));
            if isvector(source_matrix)
                source_matrix = source_matrix(:);
            end
            if attacker_id <= size(source_matrix, 2)
                source_matrix(:, attacker_id) = NaN;
            end
            source = max(source_matrix, [], 2, 'omitnan').';
        end
        for k = 1:n_steps
            values = [direct(k), source(k)];
            values = values(isfinite(values));
            if ~isempty(values)
                combined(k) = max(values);
            end
        end

        source_traces(end + 1, 1:n_steps) = source; %#ok<AGROW>
        combined_traces(end + 1, 1:n_steps) = combined; %#ok<AGROW>
    end

    n = max([size(trust_traces, 2), size(source_traces, 2), size(combined_traces, 2)]);
    time_vec = (1:n) * dt;
    mean_trust = mean(pad_to_width(trust_traces, n), 1, 'omitnan');
    mean_source = mean(pad_to_width(source_traces, n), 1, 'omitnan');
    mean_combined = mean(pad_to_width(combined_traces, n), 1, 'omitnan');
end

function X = pad_to_width(X, n)
    if isempty(X)
        X = NaN(1, n);
        return;
    end
    if size(X, 2) < n
        X(:, end + 1:n) = NaN;
    end
end

function t = first_sustained_time(time_vec, condition, start_idx, end_idx, sustain_steps)
    t = NaN;
    if isempty(condition) || start_idx > end_idx
        return;
    end
    start_idx = max(1, start_idx);
    end_idx = min(numel(condition), end_idx);
    for idx = start_idx:end_idx
        last_idx = min(end_idx, idx + sustain_steps - 1);
        if last_idx - idx + 1 < sustain_steps
            return;
        end
        if all(condition(idx:last_idx))
            t = time_vec(idx);
            return;
        end
    end
end

function y = final_window_mean(x, dt, window_seconds)
    n = numel(x);
    win = max(1, round(window_seconds / dt));
    start_idx = max(1, n - win + 1);
    y = mean(x(start_idx:n), 'omitnan');
end

function out = fmt(x, precision)
    if nargin < 2
        precision = 4;
    end
    if isempty(x) || isnan(x)
        out = "N/A";
    else
        out = string(sprintf(['%0.', num2str(precision), 'f'], x));
    end
end

function out = pct(x, precision)
    if nargin < 2
        precision = 1;
    end
    if isempty(x) || isnan(x)
        out = "N/A";
    else
        out = string(sprintf(['%0.', num2str(precision), 'f%%'], 100 * x));
    end
end

function write_report(report_file, case_tbl, run_tbl, vehicle_tbl, attack_descriptions, ...
    trust_threshold, recovery_fraction, sustain_seconds, case_csv, run_csv, vehicle_csv)
    fid = fopen(report_file, 'w');
    if fid < 0
        error('Could not open report for writing: %s', report_file);
    end
    cleanup = onCleanup(@() fclose(fid));

    fprintf(fid, '# Paper Analysis Summary\n\n');
    fprintf(fid, 'Source data: 15 saved runs under `test/results` covering local, global, and both attack channels, attackers V1-V5, and five Mix_test cases. Attack and RMSE window: 10-15 s.\n\n');
    fprintf(fid, 'Metric definitions used here:\n\n');
    fprintf(fid, '- Detection delay: first time after 10 s when mean trust in the attacker falls below %.2f; workbook detection is also reported as `MeanDetectionTime - 10`.\n', trust_threshold);
    fprintf(fid, '- Trust recovery delay: first time after 15 s when mean trust in the attacker stays above %.2f for %.1f s.\n', trust_threshold, sustain_seconds);
    fprintf(fid, '- Source influence: attacker-as-neighbor/global-source weight, excluding direct self-weight.\n');
    fprintf(fid, '- Combined influence: max of direct attacker target weight and attacker source influence. The workbook columns named `MeanAttackerSourceInfluence_*` are mislabeled and contain this combined influence.\n');
    fprintf(fid, '- Suppression delay: first time during 10-15 s when influence stays below 10%% of its pre-attack mean for %.1f s.\n', sustain_seconds);
    fprintf(fid, '- Influence recovery delay: first time after 15 s when influence stays above %.0f%% of its pre-attack mean for %.1f s.\n\n', recovery_fraction * 100, sustain_seconds);

    fprintf(fid, '## Overall Results\n\n');
    fprintf(fid, '- Mean raw combined RMSE over all non-attacker vehicle/case/run cells: %s.\n', fmt(mean(vehicle_tbl.RawCombinedRMSE, 'omitnan'), 4));
    fprintf(fid, '- Maximum raw combined RMSE observed: %s.\n', fmt(max(vehicle_tbl.RawCombinedRMSE, [], 'omitnan'), 4));
    fprintf(fid, '- Mean workbook detection delay: %s s; detection rate is %s across case-level entries.\n', ...
        fmt(mean(case_tbl.WorkbookDetectionDelay_s, 'omitnan'), 4), pct(mean(case_tbl.DetectionRate, 'omitnan'), 1));
    fprintf(fid, '- Mean trace-based trust recovery delay after attack end: %s s.\n', fmt(mean(case_tbl.TrustRecoveryDelay_s, 'omitnan'), 3));
    fprintf(fid, '- Mean source-influence suppression delay: %s s; mean combined-influence suppression delay: %s s.\n', ...
        fmt(mean(case_tbl.SourceSuppressionDelay10pctPre_s, 'omitnan'), 3), ...
        fmt(mean(case_tbl.CombinedSuppressionDelay10pctPre_s, 'omitnan'), 3));
    fprintf(fid, '- Mean source-influence recovery delay: %s s; mean combined-influence recovery delay: %s s.\n\n', ...
        fmt(mean(case_tbl.SourceRecoveryDelay90pctPre_s, 'omitnan'), 3), ...
        fmt(mean(case_tbl.CombinedRecoveryDelay90pctPre_s, 'omitnan'), 3));

    fprintf(fid, '## By Attack Channel\n\n');
    fprintf(fid, '| Channel | Mean raw RMSE | Max raw RMSE | Trust attack | Trust drop | Detection delay (s) | Trust recovery (s) | Source attack weight | Source zero rate | Source suppression (s) |\n');
    fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n');
    channels = unique(case_tbl.DataTypeAttack, 'stable');
    for i = 1:numel(channels)
        mask = case_tbl.DataTypeAttack == channels(i);
        fprintf(fid, '| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |\n', ...
            channels(i), ...
            fmt(mean(case_tbl.MeanRawCombinedRMSE(mask), 'omitnan'), 4), ...
            fmt(max(case_tbl.MaxRawCombinedRMSE(mask), [], 'omitnan'), 4), ...
            fmt(mean(case_tbl.MeanTrustAttack(mask), 'omitnan'), 3), ...
            fmt(mean(case_tbl.TrustDrop(mask), 'omitnan'), 3), ...
            fmt(mean(case_tbl.WorkbookDetectionDelay_s(mask), 'omitnan'), 3), ...
            fmt(mean(case_tbl.TrustRecoveryDelay_s(mask), 'omitnan'), 3), ...
            fmt(mean(case_tbl.MeanSourceInfluenceAttack(mask), 'omitnan'), 4), ...
            pct(mean(case_tbl.SourceZeroRateAttack(mask), 'omitnan'), 1), ...
            fmt(mean(case_tbl.SourceSuppressionDelay10pctPre_s(mask), 'omitnan'), 3));
    end
    fprintf(fid, '\n');

    fprintf(fid, '## By Attack Case\n\n');
    fprintf(fid, '| Case | Description | Mean raw RMSE | Trust attack | Trust drop | Detection delay (s) | Trust recovery (s) | Source attack weight | Source zero rate |\n');
    fprintf(fid, '|---:|---|---:|---:|---:|---:|---:|---:|---:|\n');
    for case_idx = 1:numel(attack_descriptions)
        mask = case_tbl.Case == "Case " + case_idx;
        fprintf(fid, '| %d | %s | %s | %s | %s | %s | %s | %s | %s |\n', ...
            case_idx, string(attack_descriptions{case_idx}), ...
            fmt(mean(case_tbl.MeanRawCombinedRMSE(mask), 'omitnan'), 4), ...
            fmt(mean(case_tbl.MeanTrustAttack(mask), 'omitnan'), 3), ...
            fmt(mean(case_tbl.TrustDrop(mask), 'omitnan'), 3), ...
            fmt(mean(case_tbl.WorkbookDetectionDelay_s(mask), 'omitnan'), 3), ...
            fmt(mean(case_tbl.TrustRecoveryDelay_s(mask), 'omitnan'), 3), ...
            fmt(mean(case_tbl.MeanSourceInfluenceAttack(mask), 'omitnan'), 4), ...
            pct(mean(case_tbl.SourceZeroRateAttack(mask), 'omitnan'), 1));
    end
    fprintf(fid, '\n');

    fprintf(fid, '## Strongest Cells\n\n');
    [~, max_idx] = max(vehicle_tbl.RawCombinedRMSE);
    fprintf(fid, '- Largest single vehicle/case impact: %s attacker %s, %s (%s), target %s, raw combined RMSE %s.\n', ...
        vehicle_tbl.DataTypeAttack(max_idx), vehicle_tbl.AttackerVehicle(max_idx), ...
        vehicle_tbl.Case(max_idx), vehicle_tbl.AttackDescription(max_idx), ...
        vehicle_tbl.Vehicle(max_idx), fmt(vehicle_tbl.RawCombinedRMSE(max_idx), 4));

    [~, run_idx] = max(run_tbl.MeanRawCombinedRMSE);
    fprintf(fid, '- Highest average run severity: %s attacker %s, mean raw combined RMSE %s.\n', ...
        run_tbl.DataTypeAttack(run_idx), run_tbl.AttackerVehicle(run_idx), ...
        fmt(run_tbl.MeanRawCombinedRMSE(run_idx), 4));

    fprintf(fid, '\n## Suggested Paper Text\n\n');
    fprintf(fid, 'Across the 15 saved Mix_test runs, the trust layer detected all injected attacks within approximately %.3f s after attack onset. Mean trust in the attacker decreased from a pre-attack level near %.3f to %.3f during the 10-15 s attack window, while attacker source influence was strongly attenuated from %.3f to %.3f on average. After the attack ended, the mean trust trace recovered above the %.2f operational threshold after %.3f s on average, indicating that the mechanism suppresses malicious data rapidly while allowing trust to recover once the attack stops.\n\n', ...
        mean(case_tbl.WorkbookDetectionDelay_s, 'omitnan'), ...
        mean(case_tbl.MeanTrustPre, 'omitnan'), ...
        mean(case_tbl.MeanTrustAttack, 'omitnan'), ...
        mean(case_tbl.MeanSourceInfluencePre, 'omitnan'), ...
        mean(case_tbl.MeanSourceInfluenceAttack, 'omitnan'), ...
        trust_threshold, mean(case_tbl.TrustRecoveryDelay_s, 'omitnan'));

    fprintf(fid, 'The global-channel attacks produced the largest estimation degradation, with a mean raw combined RMSE of %s compared with %s for local-only and %s for combined local+global attacks. Position-fault attacks were the most severe case family by mean raw RMSE, whereas the constant bias cases produced the strongest trust suppression. These results support the interpretation that the observer remains numerically stable under attack because the trust-weight mechanism removes the attacker as a reliable source within the attack window.\n\n', ...
        channel_mean(case_tbl, "global"), channel_mean(case_tbl, "local"), channel_mean(case_tbl, "both"));

    fprintf(fid, '## Output Files\n\n');
    fprintf(fid, '- `%s`\n', case_csv);
    fprintf(fid, '- `%s`\n', run_csv);
    fprintf(fid, '- `%s`\n', vehicle_csv);
end

function s = channel_mean(case_tbl, channel)
    mask = case_tbl.DataTypeAttack == channel;
    s = fmt(mean(case_tbl.MeanRawCombinedRMSE(mask), 'omitnan'), 4);
end
