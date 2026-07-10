%%%%%%%%%
%%%%%%%%%
%%%%%%%%%
%%%%%%%%%
%% Here plot mean for all scenarios with the cyberattack ( Bogus type ).

Config

% Log and Debug related
IsShowAnimation = false;
debug_mode = false;
if (debug_mode )
    dbstop if error;
    % dbstop in Observer at 75 if instant_index>=1000;
end


%% Attack and communication modules are reset inside each attack case.



t_star = 10;
t_end = 15;
rmse_time_window = [t_star, t_end];
attacker_vehicle_id = 1;
victim_id = -1;
data_type_attack = "local"; % "local" , "global","both"
attack_type = "Mix_test"; % "DoS" , "faulty" , "scaling" , "Collusion" ,"Bogus" , "POS" , "VEL" , "ACC" , "Mix_test"


% ──────────────────────────────────────────────────────────────────────────────
% 3) Run output, Excel logging, and command-window diary setup.
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end

run_timestamp = datestr(now, 'yyyymmdd_HHMMSS_FFF');
data_type_folder = sanitize_result_filename(char(data_type_attack));
attacker_folder = sprintf('attacker_V%d', attacker_vehicle_id);
attack_type_folder = sanitize_result_filename(char(attack_type));
run_label = sanitize_result_filename(sprintf('%s_window_%gs_%gs', ...
    run_timestamp, rmse_time_window(1), rmse_time_window(2)));
results_root = fullfile(script_dir, 'results');
results_group_dir = fullfile(results_root, data_type_folder, attacker_folder, attack_type_folder);
run_results_dir = fullfile(results_group_dir, run_label);
figures_dir = fullfile(run_results_dir, 'figures');
logs_dir = fullfile(run_results_dir, 'logs');

if ~exist(results_root, 'dir')
    mkdir(results_root);
end
if ~exist(results_group_dir, 'dir')
    mkdir(results_group_dir);
end
if ~exist(run_results_dir, 'dir')
    mkdir(run_results_dir);
end
if ~exist(figures_dir, 'dir')
    mkdir(figures_dir);
end
if ~exist(logs_dir, 'dir')
    mkdir(logs_dir);
end

run_log_filename = fullfile(logs_dir, 'command_window.log');
diary(run_log_filename);
diary on;
fprintf('Run output folder: %s\n', run_results_dir);
fprintf('Command-window log: %s\n', run_log_filename);

excel_filename = fullfile(run_results_dir, sprintf('Results_%s_FullAndAttackWindow_%gs_%gs_Attacker_V%d.xlsx', ...
    attack_type, rmse_time_window(1), rmse_time_window(2), attacker_vehicle_id));
sheet_name     = 'Summary';
stats_sheet_name = 'MetricStats';
trust_weight_sheet_name = 'TrustWeightStats';
headers = {'AttackType','AttackerVehicle','Case','Vehicle', ...
    'Full_RMSE_Distance','Full_RMSE_Orientation','Full_RMSE_Velocity','Full_RMSE_Acceleration', ...
    'AttackWindow_RMSE_Distance','AttackWindow_RMSE_Orientation','AttackWindow_RMSE_Velocity','AttackWindow_RMSE_Acceleration', ...
    'Full_Raw_Combined','AttackWindow_Raw_Combined','RMSE_Window_Start','RMSE_Window_End', ...
    'Mean_Trust_Score','Trust_Degradation','Attack_Detection_Time'};

% Refresh this run's summary from the top to avoid mixing stale partial rows.
writecell(headers, excel_filename, 'Sheet', sheet_name, 'Range', 'A1');
row_index = 2;



% Vehicles are initialized in the same lane with fixed platoon spacing.
initial_lane_id = 1;
direction_flag = 0; % 1 stands for changing to the left adjacent lane, 0 stands for keeping the current lane, -1 stands for changing

% Fixed lead scenario for all attack cases
Scenarios_config.set_Lead_Senarios("constant");




start_attack_senarios_index = 1;
last_attack_senarios_index = 5;  % All Mix_test cases (1-5) - matches Atk_Scenarios.m

attack_case_numbers = start_attack_senarios_index:last_attack_senarios_index;
total_num_attack_cases = numel(attack_case_numbers);
all_case_state_logs = cell(total_num_attack_cases, 1);
all_case_input_logs = cell(total_num_attack_cases, 1);

num_vehicles = 5;
all_vehicle_ids = 1:num_vehicles;
non_attacker_ids = all_vehicle_ids(all_vehicle_ids ~= attacker_vehicle_id);

% Override the 4-vehicle graph from Config for this 5-vehicle workflow.
graph = ones(num_vehicles) - eye(num_vehicles);
clear('Weight_Trust_module');
weight_trust_module = Weight_Trust_module(graph, trust_threshold, kappa);

vehicle_labels = arrayfun(@(x) sprintf('V%d', x), all_vehicle_ids, 'UniformOutput', false);
scenario_labels = arrayfun(@(x) sprintf("Case %d", x), attack_case_numbers, 'UniformOutput', false);
metric_labels = {'Distance Error (m)', 'Orientation Error (rad)', 'Velocity Error (m/s)','Acc Error (m/s)'};

mean_errors = zeros(num_vehicles, length(metric_labels), total_num_attack_cases); % vehicles x metrics x attack cases
mean_errors_full = zeros(num_vehicles, length(metric_labels), total_num_attack_cases);

% Trust and trust-weight diagnostics for showing the defense mechanism.
case_time_vectors = cell(total_num_attack_cases, 1);
mean_attacker_trust_time_by_case = cell(total_num_attack_cases, 1);
mean_attacker_weight_time_by_case = cell(total_num_attack_cases, 1);
mean_attacker_direct_weight_time_by_case = cell(total_num_attack_cases, 1);
mean_attacker_source_weight_time_by_case = cell(total_num_attack_cases, 1);
mean_trusted_neighbor_count_time_by_case = cell(total_num_attack_cases, 1);

mean_attacker_trust_full_by_case = NaN(1, total_num_attack_cases);
mean_attacker_trust_pre_by_case = NaN(1, total_num_attack_cases);
mean_attacker_trust_attack_by_case = NaN(1, total_num_attack_cases);
min_attacker_trust_attack_by_case = NaN(1, total_num_attack_cases);
mean_trust_degradation_by_case = NaN(1, total_num_attack_cases);
detection_rate_by_case = NaN(1, total_num_attack_cases);
mean_detection_time_by_case = NaN(1, total_num_attack_cases);
false_positive_rejection_full_by_case = NaN(1, total_num_attack_cases);
false_positive_rejection_pre_by_case = NaN(1, total_num_attack_cases);
false_positive_rejection_attack_by_case = NaN(1, total_num_attack_cases);

mean_attacker_weight_pre_by_case = NaN(1, total_num_attack_cases);
mean_attacker_weight_attack_by_case = NaN(1, total_num_attack_cases);
attacker_weight_reduction_by_case = NaN(1, total_num_attack_cases);
attacker_weight_zero_rate_attack_by_case = NaN(1, total_num_attack_cases);
mean_attacker_direct_weight_pre_by_case = NaN(1, total_num_attack_cases);
mean_attacker_direct_weight_attack_by_case = NaN(1, total_num_attack_cases);
mean_attacker_source_weight_pre_by_case = NaN(1, total_num_attack_cases);
mean_attacker_source_weight_attack_by_case = NaN(1, total_num_attack_cases);
attacker_source_weight_reduction_by_case = NaN(1, total_num_attack_cases);
attacker_source_weight_zero_rate_attack_by_case = NaN(1, total_num_attack_cases);
mean_trusted_neighbor_count_attack_by_case = NaN(1, total_num_attack_cases);

attacker_trust_attack_by_vehicle = NaN(length(non_attacker_ids), total_num_attack_cases);
attacker_weight_attack_by_vehicle = NaN(length(non_attacker_ids), total_num_attack_cases);


is_plot_each_case = false;
plot_weight_diagnostics_each_case = false; % Python-style per-case observer weight figures
hybrid_attack_x_label_mode = "case"; % "case" = Case1..CaseN, "description" = attack type/value labels

initial_x_positions = 80:-20:(80 - 20 * (num_vehicles - 1));
initial_speeds = [23, repmat(26, 1, num_vehicles - 1)];
controller_types = ["None", repmat("IDM", 1, num_vehicles - 1)];
controller2_types = ["None", repmat("CACC", 1, num_vehicles - 1)];

% Give each vehicle a small fixed parameter mismatch, reused for all cases.
rng_state = rng;
rng(10);
param_spread = 0.03;
vehicle_params = cell(num_vehicles, 1);
for vehicle_id = all_vehicle_ids
    veh_param = param_sys;
    scale = @(spread) 1 + spread * (2 * rand - 1);

    veh_param.l_f = param_sys.l_f * scale(param_spread);
    veh_param.l_r = param_sys.l_r * scale(param_spread);
    veh_param.l_fc = param_sys.l_fc * scale(param_spread);
    veh_param.l_rc = param_sys.l_rc * scale(param_spread);
    veh_param.width = param_sys.width * scale(0.01);
    veh_param.tau = param_sys.tau * scale(param_spread);
    veh_param.tau_v = param_sys.tau_v * scale(param_spread);
    veh_param.mass = param_sys.mass * scale(0.05);
    veh_param.C1 = param_sys.C1 * scale(0.05);
    veh_param.C2 = param_sys.C2 * scale(0.05);

    veh_param.max_acceleration = param_sys.max_acceleration * scale(0.05);
    veh_param.min_acceleration = param_sys.min_acceleration * scale(0.05);

    max_steer = param_sys.max_steering_angle * scale(0.02);
    veh_param.max_steering_angle = max_steer;
    veh_param.min_steering_angle = -max_steer;

    vehicle_params{vehicle_id} = veh_param;
end
rng(rng_state);

for case_idx = 1:total_num_attack_cases
    case_nb_attack = attack_case_numbers(case_idx);
    fprintf('Running %s attack by V%d, Case %d/%d...\n', ...
        attack_type, attacker_vehicle_id, case_idx, total_num_attack_cases);

    attack_module = Attack_module(Scenarios_config.dt);
    attack_module = Atk_Scenarios(attack_module , attack_type ,data_type_attack,case_nb_attack , t_star, t_end, attacker_vehicle_id,victim_id );
    center_communication = CenterCommunication(attack_module);


    platton_vehicles = Vehicle.empty;
    for vehicle_id = all_vehicle_ids
        initial_state = [initial_x_positions(vehicle_id); 0.5 * lane_width; 0; initial_speeds(vehicle_id); 0];
        new_vehicle = Vehicle(vehicle_id, controller_types(vehicle_id), vehicle_params{vehicle_id}, initial_state, initial_lane_id, straightLanes, direction_flag, 0, Scenarios_config, weight_trust_module);
        platton_vehicles = [platton_vehicles; new_vehicle];
    end

    for vehicle_id = all_vehicle_ids
        platton_vehicles(vehicle_id).assign_neighbor_vehicle(platton_vehicles, [], controller2_types(vehicle_id), center_communication, graph);
    end


    %% define a simulator and start simulation
    simulator0 = Simulator(straightLanes, [] , platton_vehicles, Scenarios_config.dt , IsShowAnimation );
    [state_log, input_log] = simulator0.startSimulation(Scenarios_config.simulation_time,t_star, t_end, attacker_vehicle_id);
    all_case_state_logs{case_idx} = state_log;
    all_case_input_logs{case_idx} = input_log;

    % plot
    if (is_plot_each_case)
        for vehicle_id = all_vehicle_ids
            platton_vehicles(vehicle_id).plot_ground_error_global_est(platton_vehicles);
        end
    end

    if plot_weight_diagnostics_each_case
        plot_observer_weight_diagnostics(platton_vehicles, attacker_vehicle_id, ...
            t_star, t_end, trust_threshold, case_nb_attack, attacker_vehicle_id);
    end
    % After the simulation, calculate errors for each vehicle
    % Select vehicles to evaluate based on IDs
    vehicles_to_evaluate = platton_vehicles(non_attacker_ids);

    all_full_dist_errors = [];
    all_full_theta_errors = [];
    all_full_vel_errors = [];
    all_full_acc_errors = [];

    all_attack_dist_errors = [];
    all_attack_theta_errors = [];
    all_attack_vel_errors = [];
    all_attack_acc_errors = [];
    
    % Trust-related metrics
    all_trust_scores = [];
    trust_degradation = [];
    attack_detection_times = [];

    for k = 1:length(vehicles_to_evaluate)
        v = vehicles_to_evaluate(k);
        [dist_full, theta_full, vel_full, acc_full] = v.observer.calculate_global_errors();
        [dist_attack, theta_attack, vel_attack, acc_attack] = v.observer.calculate_global_errors(rmse_time_window);

        all_full_dist_errors = [all_full_dist_errors, dist_full];
        all_full_theta_errors = [all_full_theta_errors, theta_full];
        all_full_vel_errors = [all_full_vel_errors, vel_full];
        all_full_acc_errors = [all_full_acc_errors, acc_full];

        all_attack_dist_errors = [all_attack_dist_errors, dist_attack];
        all_attack_theta_errors = [all_attack_theta_errors, theta_attack];
        all_attack_vel_errors = [all_attack_vel_errors, vel_attack];
        all_attack_acc_errors = [all_attack_acc_errors, acc_attack];
        
        % Extract each evaluator's trust in the attacker from Vehicle.trust_log.
        trust_trace = squeeze(v.trust_log(1, :, attacker_vehicle_id));
        attack_start_idx = max(1, round(t_star / Scenarios_config.dt));
        attack_end_idx = min(length(trust_trace), round(t_end / Scenarios_config.dt));

        if attack_start_idx <= length(trust_trace) && attack_start_idx <= attack_end_idx
            pre_attack_end_idx = max(1, attack_start_idx - 1);
            pre_attack_trust = mean(trust_trace(1:pre_attack_end_idx), 'omitnan');
            during_attack_trust = mean(trust_trace(attack_start_idx:attack_end_idx), 'omitnan');

            all_trust_scores = [all_trust_scores; mean(trust_trace, 'omitnan')];
            trust_degradation = [trust_degradation; pre_attack_trust - during_attack_trust];

            trust_threshold_detection = 0.7;
            detection_idx = find(trust_trace(attack_start_idx:attack_end_idx) < trust_threshold_detection, 1);
            if ~isempty(detection_idx)
                detection_time = (attack_start_idx + detection_idx - 1) * Scenarios_config.dt;
                attack_detection_times = [attack_detection_times; detection_time];
            else
                attack_detection_times = [attack_detection_times; NaN];
            end
        else
            all_trust_scores = [all_trust_scores; NaN];
            trust_degradation = [trust_degradation; NaN];
            attack_detection_times = [attack_detection_times; NaN];
        end
    end

    % Aggregate trust in the attacker and the actual attacker source influence.
    % With per-target weights, this scalar is the maximum attacker contribution
    % across target rows: direct w0 when the attacker is the target, or
    % attacker-as-neighbor/source weight for other targets.
    if ~isempty(vehicles_to_evaluate)
        num_trust_steps = size(vehicles_to_evaluate(1).trust_log, 2);
        case_time_vectors{case_idx} = (1:num_trust_steps) * Scenarios_config.dt;

        attack_start_idx = max(1, round(t_star / Scenarios_config.dt));
        attack_end_idx = min(num_trust_steps, round(t_end / Scenarios_config.dt));
        pre_attack_indices = 1:max(1, attack_start_idx - 1);
        attack_indices = attack_start_idx:attack_end_idx;

        attacker_trust_traces = NaN(length(vehicles_to_evaluate), num_trust_steps);
        attacker_weight_traces = NaN(length(vehicles_to_evaluate), num_trust_steps);
        attacker_direct_weight_traces = NaN(length(vehicles_to_evaluate), num_trust_steps);
        attacker_source_weight_traces = NaN(length(vehicles_to_evaluate), num_trust_steps);
        trusted_neighbor_count_traces = NaN(length(vehicles_to_evaluate), num_trust_steps);
        benign_trust_traces = NaN(0, num_trust_steps);

        for ev_idx = 1:length(vehicles_to_evaluate)
            evaluator = vehicles_to_evaluate(ev_idx);
            evaluator_id = evaluator.vehicle_number;
            attacker_trust_trace = squeeze(evaluator.trust_log(1, :, attacker_vehicle_id));
            attacker_trust_traces(ev_idx, :) = attacker_trust_trace(:).';

            benign_vehicle_ids = setdiff(all_vehicle_ids, [attacker_vehicle_id, evaluator_id]);
            for benign_id = benign_vehicle_ids
                benign_trust_trace = squeeze(evaluator.trust_log(1, :, benign_id));
                benign_trust_trace = benign_trust_trace(:).';
                trace_len = min(numel(benign_trust_trace), num_trust_steps);
                if trace_len > 0
                    benign_trust_traces(end + 1, 1:trace_len) = benign_trust_trace(1:trace_len); %#ok<AGROW>
                end
            end

            for time_idx = 1:num_trust_steps
                trust_scores_now = squeeze(evaluator.trust_log(1, time_idx, :));
                if numel(trust_scores_now) == num_vehicles
                    trust_scores_now = trust_scores_now(:).';
                    if isprop(evaluator.observer, 'target_weights_log') && ...
                            ~isempty(evaluator.observer.target_weights_log) && ...
                            time_idx <= size(evaluator.observer.target_weights_log, 2)
                        target_weights_now = evaluator.observer.target_weights_log(:, time_idx, :);

                        direct_attacker_weight = NaN;
                        if attacker_vehicle_id <= size(target_weights_now, 3)
                            direct_attacker_weight = target_weights_now(1, 1, attacker_vehicle_id);
                            if isfinite(direct_attacker_weight)
                                attacker_direct_weight_traces(ev_idx, time_idx) = direct_attacker_weight;
                            end
                        end

                        source_attacker_weights = NaN(1, size(target_weights_now, 3));
                        if attacker_vehicle_id + 1 <= size(target_weights_now, 1)
                            source_attacker_weights = squeeze(target_weights_now(attacker_vehicle_id + 1, 1, :)).';
                            if attacker_vehicle_id <= numel(source_attacker_weights)
                                source_attacker_weights(attacker_vehicle_id) = NaN;
                            end
                        end
                        finite_source_attacker_weights = source_attacker_weights(isfinite(source_attacker_weights));
                        if ~isempty(finite_source_attacker_weights)
                            attacker_source_weight_traces(ev_idx, time_idx) = max(finite_source_attacker_weights);
                        end

                        attacker_source_values = [direct_attacker_weight, finite_source_attacker_weights];
                        attacker_source_values = attacker_source_values(isfinite(attacker_source_values));
                        if ~isempty(attacker_source_values)
                            attacker_weight_traces(ev_idx, time_idx) = max(attacker_source_values);
                        end
                    end
                    trusted_neighbor_count_traces(ev_idx, time_idx) = ...
                        length(weight_trust_module.get_trusted_neighbors(evaluator_id, trust_scores_now));
                end
            end
        end

        mean_attacker_trust_time = mean(attacker_trust_traces, 1, 'omitnan');
        mean_attacker_weight_time = mean(attacker_weight_traces, 1, 'omitnan');
        mean_attacker_direct_weight_time = mean(attacker_direct_weight_traces, 1, 'omitnan');
        mean_attacker_source_weight_time = mean(attacker_source_weight_traces, 1, 'omitnan');
        mean_trusted_neighbor_count_time = mean(trusted_neighbor_count_traces, 1, 'omitnan');

        mean_attacker_trust_time_by_case{case_idx} = mean_attacker_trust_time;
        mean_attacker_weight_time_by_case{case_idx} = mean_attacker_weight_time;
        mean_attacker_direct_weight_time_by_case{case_idx} = mean_attacker_direct_weight_time;
        mean_attacker_source_weight_time_by_case{case_idx} = mean_attacker_source_weight_time;
        mean_trusted_neighbor_count_time_by_case{case_idx} = mean_trusted_neighbor_count_time;

        mean_attacker_trust_full_by_case(case_idx) = mean(mean_attacker_trust_time, 'omitnan');
        mean_attacker_trust_pre_by_case(case_idx) = mean(mean_attacker_trust_time(pre_attack_indices), 'omitnan');
        mean_attacker_trust_attack_by_case(case_idx) = mean(mean_attacker_trust_time(attack_indices), 'omitnan');
        mean_trust_degradation_by_case(case_idx) = mean(trust_degradation, 'omitnan');
        false_positive_rejection_full_by_case(case_idx) = ...
            finite_fraction_below(benign_trust_traces(:), trust_threshold);
        false_positive_rejection_pre_by_case(case_idx) = ...
            finite_fraction_below(benign_trust_traces(:, pre_attack_indices), trust_threshold);
        false_positive_rejection_attack_by_case(case_idx) = ...
            finite_fraction_below(benign_trust_traces(:, attack_indices), trust_threshold);

        attack_mean_trust = mean_attacker_trust_time(attack_indices);
        attack_mean_trust = attack_mean_trust(~isnan(attack_mean_trust));
        if ~isempty(attack_mean_trust)
            min_attacker_trust_attack_by_case(case_idx) = min(attack_mean_trust);
        end

        valid_detection_times = attack_detection_times(~isnan(attack_detection_times));
        detection_rate_by_case(case_idx) = numel(valid_detection_times) / length(vehicles_to_evaluate);
        if ~isempty(valid_detection_times)
            mean_detection_time_by_case(case_idx) = mean(valid_detection_times);
        end

        mean_attacker_weight_pre_by_case(case_idx) = mean(mean_attacker_weight_time(pre_attack_indices), 'omitnan');
        mean_attacker_weight_attack_by_case(case_idx) = mean(mean_attacker_weight_time(attack_indices), 'omitnan');
        attacker_weight_reduction_by_case(case_idx) = ...
            mean_attacker_weight_pre_by_case(case_idx) - mean_attacker_weight_attack_by_case(case_idx);
        mean_attacker_direct_weight_pre_by_case(case_idx) = ...
            mean(mean_attacker_direct_weight_time(pre_attack_indices), 'omitnan');
        mean_attacker_direct_weight_attack_by_case(case_idx) = ...
            mean(mean_attacker_direct_weight_time(attack_indices), 'omitnan');
        mean_attacker_source_weight_pre_by_case(case_idx) = ...
            mean(mean_attacker_source_weight_time(pre_attack_indices), 'omitnan');
        mean_attacker_source_weight_attack_by_case(case_idx) = ...
            mean(mean_attacker_source_weight_time(attack_indices), 'omitnan');
        attacker_source_weight_reduction_by_case(case_idx) = ...
            mean_attacker_source_weight_pre_by_case(case_idx) - mean_attacker_source_weight_attack_by_case(case_idx);
        mean_trusted_neighbor_count_attack_by_case(case_idx) = ...
            mean(mean_trusted_neighbor_count_time(attack_indices), 'omitnan');

        attack_weight_values = attacker_weight_traces(:, attack_indices);
        attack_weight_values = attack_weight_values(~isnan(attack_weight_values));
        if ~isempty(attack_weight_values)
            attacker_weight_zero_rate_attack_by_case(case_idx) = mean(attack_weight_values <= 1e-9);
        end
        attack_source_weight_values = attacker_source_weight_traces(:, attack_indices);
        attack_source_weight_values = attack_source_weight_values(~isnan(attack_source_weight_values));
        if ~isempty(attack_source_weight_values)
            attacker_source_weight_zero_rate_attack_by_case(case_idx) = mean(attack_source_weight_values <= 1e-9);
        end

        for ev_idx = 1:length(vehicles_to_evaluate)
            attacker_trust_attack_by_vehicle(ev_idx, case_idx) = ...
                mean(attacker_trust_traces(ev_idx, attack_indices), 'omitnan');
            attacker_weight_attack_by_vehicle(ev_idx, case_idx) = ...
                mean(attacker_weight_traces(ev_idx, attack_indices), 'omitnan');
        end
    end




    % Calculate consensus RMSE across all observer vehicles for each target vehicle.
    % Each observer vehicle estimates all other vehicles -> average their RMSE estimates
    % Result: mean_attack_dist(i) = average RMSE for vehicle i across all observers.
    mean_full_dist = mean(all_full_dist_errors,2);
    mean_full_theta = mean(all_full_theta_errors,2);
    mean_full_vel = mean(all_full_vel_errors,2);
    mean_full_acc = mean(all_full_acc_errors,2);

    mean_attack_dist = mean(all_attack_dist_errors,2);
    mean_attack_theta = mean(all_attack_theta_errors,2);
    mean_attack_vel = mean(all_attack_vel_errors,2);
    mean_attack_acc = mean(all_attack_acc_errors,2);

    % write one row per evaluating vehicle
    for vi = 1:length(non_attacker_ids)
        target_id = non_attacker_ids(vi);
        % Handle trust metrics safely
        if isempty(all_trust_scores) || vi > length(all_trust_scores)
            trust_score = NaN;
        else
            trust_score = all_trust_scores(vi);
        end
        
        if isempty(trust_degradation) || vi > length(trust_degradation)
            trust_deg = NaN;
        else
            trust_deg = trust_degradation(vi);
        end
        
        if isempty(attack_detection_times) || vi > length(attack_detection_times)
            detection_time = NaN;
        else
            detection_time = attack_detection_times(vi);
        end

        full_raw_combined = mean_full_dist(target_id) + mean_full_vel(target_id) + mean_full_acc(target_id);
        attack_raw_combined = mean_attack_dist(target_id) + mean_attack_vel(target_id) + mean_attack_acc(target_id);
        
        row_data = {
            attack_type, ...
            sprintf('V%d', attacker_vehicle_id), ...
            sprintf('Case %d', case_nb_attack), ...
            sprintf('V%d', target_id), ...
            mean_full_dist(target_id), ...
            mean_full_theta(target_id), ...
            mean_full_vel(target_id), ...
            mean_full_acc(target_id), ...
            mean_attack_dist(target_id), ...
            mean_attack_theta(target_id), ...
            mean_attack_vel(target_id), ...
            mean_attack_acc(target_id), ...
            full_raw_combined, ...
            attack_raw_combined, ...
            rmse_time_window(1), ...
            rmse_time_window(2), ...
            trust_score, ...
            trust_deg, ...
            detection_time
        };
        writecell(row_data, excel_filename, 'Sheet', sheet_name, ...
                  'Range', sprintf('A%d', row_index));
        row_index = row_index + 1;
    end



    % Store the errors for plotting
    mean_errors(:, :, case_idx) = [mean_attack_dist, mean_attack_theta, mean_attack_vel, mean_attack_acc];
    mean_errors_full(:, :, case_idx) = [mean_full_dist, mean_full_theta, mean_full_vel, mean_full_acc];
    
    % Store trust data for direct analysis - COLLECT ALL CASES
    if ~exist('all_case_trust_logs', 'var')
        all_case_trust_logs = cell(total_num_attack_cases, 1);
        all_case_vehicles = cell(total_num_attack_cases, 1);
        all_case_scenarios = cell(total_num_attack_cases, 1);
    end
    
    % Store complete trust data for each case
    all_case_trust_logs{case_idx} = struct();
    all_case_vehicles{case_idx} = platton_vehicles;
    all_case_scenarios{case_idx} = struct('t_start', t_star, 't_end', t_end, 'dt', Scenarios_config.dt, 'case_number', case_nb_attack);
    
    % Collect trust logs from all vehicles for this case
    for v_idx = 1:length(platton_vehicles)
        vehicle = platton_vehicles(v_idx);
        if isprop(vehicle, 'trust_log')
            all_case_trust_logs{case_idx}.(sprintf('vehicle_%d', v_idx)) = vehicle.trust_log;
        end

        if isprop(vehicle, 'observer') && ~isempty(vehicle.observer)
            observer_log_field = sprintf('observer_v%d', v_idx);
            observer_log = struct();
            if isprop(vehicle.observer, 'target_weights_log')
                observer_log.target_weights = vehicle.observer.target_weights_log;
                observer_log.target_weights_current = vehicle.observer.target_weights_current;
            end
            if isprop(vehicle.observer, 'rollback_stats')
                observer_log.rollback_stats = vehicle.observer.rollback_stats;
            end
            all_case_trust_logs{case_idx}.(observer_log_field) = observer_log;
        end
        
        % Also store trip model data
        if isprop(vehicle, 'trip_models')
            for tm_idx = 1:length(vehicle.trip_models)
                if ~isempty(vehicle.trip_models{tm_idx})
                    trust_model = vehicle.trip_models{tm_idx};
                    field_name = sprintf('trust_model_v%d_to_v%d', v_idx, tm_idx);
                    all_case_trust_logs{case_idx}.(field_name) = struct( ...
                        'trust_samples', trust_model.trust_sample_log, ...
                        'final_scores', trust_model.final_score_log, ...
                        'gamma_cross', trust_model.gamma_cross_log, ...
                        'gamma_local', trust_model.gamma_local_log, ...
                        'gamma_self', trust_model.gamma_local_our_self_log, ...
                        'flag_target_attack', trust_model.flag_taget_attk_log, ...
                        'flag_global_est_check', trust_model.flag_glob_est_check_log, ...
                        'flag_local_est_check', trust_model.flag_local_est_check_log, ...
                        'beacon_local', trust_model.beacon_score_local_log, ...
                        'beacon_global', trust_model.beacon_score_global_log, ...
                        'v_score', trust_model.v_score_log, ...
                        'd_score', trust_model.d_score_log, ...
                        'a_score', trust_model.a_score_log);
                end
            end
        end
    end
end

%% ===================================================================
%% PAPER-QUALITY PLOTTING SECTION
%% ===================================================================

% Define Mix_test attack case descriptions for better labeling (complete list)
all_attack_descriptions = {
    'X Bias -5m', 'X Faulty 10m', 'V Bias -2.5m/s', 'V Faulty 2.5m/s', ...
    'DoS Attack'
};

% Select attack descriptions based on the scenario range
attack_descriptions = all_attack_descriptions(start_attack_senarios_index:last_attack_senarios_index);
case_only_attack_labels = arrayfun(@(x) sprintf('Case%d', x), attack_case_numbers, 'UniformOutput', false);

switch char(lower(hybrid_attack_x_label_mode))
    case 'case'
        hybrid_attack_x_labels = case_only_attack_labels;
    case 'description'
        hybrid_attack_x_labels = attack_descriptions;
    otherwise
        warning('Unknown hybrid_attack_x_label_mode "%s"; using detailed attack descriptions.', char(hybrid_attack_x_label_mode));
        hybrid_attack_x_labels = attack_descriptions;
end

%% 1. COMPREHENSIVE ESTIMATION ERROR ANALYSIS (WITHOUT ORIENTATION)
figure('Name', '01_estimation_error_bars', 'Position', [100, 100, 1200, 600]);

% Subplot 1: Distance Estimation Error
subplot(1,3,1);
data = squeeze(mean_errors(non_attacker_ids, 1, :))';  % Distance errors for non-attackers
bar(data);
xticklabels(attack_descriptions(1:size(data,1)));
xtickangle(45);
set(gca, 'FontSize', 9);  % Make text smaller
ylabel('Distance Error (m)');
title('(a) Distance Estimation Error');
legend(cellstr("V" + string(non_attacker_ids)), 'Location', 'best');
grid on;

% Subplot 2: Velocity Estimation Error  
subplot(1,3,2);
data = squeeze(mean_errors(non_attacker_ids, 3, :))';  % Velocity errors
bar(data);
xticklabels(attack_descriptions(1:size(data,1)));
xtickangle(45);
set(gca, 'FontSize', 9);  % Make text smaller
ylabel('Velocity Error (m/s)');
title('(b) Velocity Estimation Error');
legend(cellstr("V" + string(non_attacker_ids)), 'Location', 'best');
grid on;

% Subplot 3: Acceleration Estimation Error
subplot(1,3,3);
data = squeeze(mean_errors(non_attacker_ids, 4, :))';  % Acceleration errors
bar(data);
xticklabels(attack_descriptions(1:size(data,1)));
xtickangle(45);
set(gca, 'FontSize', 9);  % Make text smaller
ylabel('Acceleration Error (m/s²)');
title('(c) Acceleration Estimation Error');
legend(cellstr("V" + string(non_attacker_ids)), 'Location', 'best');
grid on;

sgtitle(sprintf('Distributed Estimation Performance Under %s Attacks (Attacker: V%d, RMSE %.1f-%.1fs)', ...
    attack_type, attacker_vehicle_id, rmse_time_window(1), rmse_time_window(2)), ...
    'FontSize', 14, 'FontWeight', 'bold');

%% 2. MEANINGFUL ATTACK IMPACT HEATMAPS (WITHOUT ORIENTATION)
% Option 1: Separate heatmaps for each error type (RECOMMENDED)
figure('Name', '02_metric_error_heatmaps', 'Position', [200, 200, 1200, 600]);

% Select only non-orientation metrics: Distance(1), Velocity(3), Acceleration(4)
selected_metrics = [1, 3, 4];
selected_labels = {'Distance Error (m)', 'Velocity Error (m/s)', 'Acc Error (m/s²)'};

for plot_idx = 1:3
    metric_idx = selected_metrics(plot_idx);
    subplot(1, 3, plot_idx);
    
    % Extract data for this specific metric across all vehicles and cases
    metric_data = squeeze(mean_errors(non_attacker_ids, metric_idx, :));  % vehicles × cases
    
    % Create heatmap
    imagesc(metric_data);
    colorbar;
    
    % Customize labels and title
    xlabel('Attack Cases');
    ylabel('Vehicles');
    title(sprintf('%s Impact', selected_labels{plot_idx}));
    
    % Set tick labels
    if size(metric_data, 2) <= length(attack_descriptions)
        xticklabels(attack_descriptions(1:size(metric_data, 2)));
    end
    % Center the y-tick labels properly
    num_vehicles = length(non_attacker_ids);
    yticks(1:num_vehicles);
    vehicle_labels = arrayfun(@(x) sprintf('V%d', x), non_attacker_ids, 'UniformOutput', false);
    yticklabels(vehicle_labels);
    xtickangle(45);
    set(gca, 'FontSize', 9);  % Make text smaller for heatmaps
    % Force y-tick labels to be centered
    set(gca, 'TickLength', [0 0]);  % Remove tick marks
    ylim([0.5, num_vehicles + 0.5]);  % Set proper y-axis limits
    
    % Add text annotations with simple black text
    for i = 1:size(metric_data, 1)
        for j = 1:size(metric_data, 2)
            text(j, i, sprintf('%.3f', metric_data(i,j)), ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                 'Color', 'black', 'FontWeight', 'bold', 'FontSize', 11, ...
                 'EdgeColor', 'none');
        end
    end
end
sgtitle(sprintf('Attack Impact Analysis by Error Type - %s Attacks by V%d', attack_type, attacker_vehicle_id), ...
        'FontSize', 14, 'FontWeight', 'bold');

% Option 2: Balanced component-normalized impact heatmap.
% This score is bounded in [0, 1], but it is not the raw RMSE. Each
% component is normalized over the displayed non-attacker vehicles, then
% distance, velocity, and acceleration receive equal weight.
figure('Name', '03_balanced_component_score_heatmap', 'Position', [300, 100, 1000, 600]);

impact_weights = [1/3, 0.0, 1/3, 1/3];  % [distance, orientation, velocity, acceleration]
normalized_errors = NaN(size(mean_errors));
normalized_errors_display = zeros(length(non_attacker_ids), 4, size(mean_errors, 3));
for metric_idx = 1:4
    metric_slice = mean_errors(non_attacker_ids, metric_idx, :);
    min_val = min(metric_slice(:));
    max_val = max(metric_slice(:));
    if max_val > min_val
        normalized_errors_display(:, metric_idx, :) = (metric_slice - min_val) / (max_val - min_val);
    end
    normalized_errors(non_attacker_ids, metric_idx, :) = normalized_errors_display(:, metric_idx, :);
end

balanced_component_scores = squeeze(sum(normalized_errors_display .* reshape(impact_weights, 1, 4, 1), 2));
impact_scores = NaN(size(mean_errors, 1), size(mean_errors, 3));
impact_scores(non_attacker_ids, :) = balanced_component_scores;

imagesc(balanced_component_scores);
caxis([0, 1]);
c = colorbar;
c.Label.String = 'Balanced component score (0-1)';
c.Label.FontSize = 10;
c.Label.FontWeight = 'bold';
xlabel('Attack Cases');
ylabel('Vehicles');
title(sprintf('Balanced Component-Normalized Impact Score - %s Attacks by V%d', attack_type, attacker_vehicle_id));
xticks(1:size(balanced_component_scores, 2));
xticklabels(attack_descriptions(1:size(balanced_component_scores, 2)));
% Center the y-tick labels properly
num_vehicles = length(non_attacker_ids);
yticks(1:num_vehicles);
vehicle_labels = arrayfun(@(x) sprintf('V%d', x), non_attacker_ids, 'UniformOutput', false);
yticklabels(vehicle_labels);
xtickangle(45);
set(gca, 'FontSize', 8);  % Make text smaller for heatmaps
% Force y-tick labels to be centered
set(gca, 'TickLength', [0 0]);  % Remove tick marks
ylim([0.5, num_vehicles + 0.5]);  % Set proper y-axis limits

% Add text annotations with simple black text
for i = 1:length(non_attacker_ids)
    for j = 1:size(balanced_component_scores, 2)
        score = balanced_component_scores(i, j);
        
        text(j, i, sprintf('%.3f', score), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
             'Color', 'black', 'FontWeight', 'bold', 'FontSize', 11, ...
             'EdgeColor', 'none');
    end
end

% raw_errors_selected = mean_errors(non_attacker_ids, raw_selected_metrics, :); % 3 vehicles × 3 metrics × 6 cases

% selected_labels = {'Distance Error (m)', 'Velocity Error (m/s)', 'Acc Error (m/s²)'};

% Raw combined attack-window RMSE used by the hybrid heatmap and summary.
raw_selected_metrics = [1, 3, 4]; % Distance, velocity, acceleration
raw_errors_selected = mean_errors(non_attacker_ids, raw_selected_metrics, :);
raw_combined_errors = zeros(length(non_attacker_ids), size(mean_errors, 3));
raw_errors_full_selected = mean_errors_full(non_attacker_ids, raw_selected_metrics, :);
raw_combined_errors_full = zeros(length(non_attacker_ids), size(mean_errors_full, 3));
for i = 1:length(non_attacker_ids)
    for j = 1:size(mean_errors, 3)
        vehicle_errors = squeeze(raw_errors_selected(i, :, j));
        raw_combined_errors(i, j) = sum(vehicle_errors);

        vehicle_full_errors = squeeze(raw_errors_full_selected(i, :, j));
        raw_combined_errors_full(i, j) = sum(vehicle_full_errors);
    end
end

valid_raw_combined = raw_combined_errors(~isnan(raw_combined_errors));
if isempty(valid_raw_combined) || max(valid_raw_combined) <= 0
    raw_normalized_impact = zeros(size(raw_combined_errors));
else
    raw_normalized_impact = raw_combined_errors / max(valid_raw_combined);
end

%% 3. RAW COMBINED RMSE HEATMAP
figure('Name', '04_raw_combined_rmse_heatmap', 'Position', [400, 180, 1000, 600]);
imagesc(raw_combined_errors);
c = colorbar;
c.Label.String = 'Raw combined RMSE';
c.Label.FontSize = 10;
c.Label.FontWeight = 'bold';
xlabel('Attack Cases');
ylabel('Vehicles');
title(sprintf('Raw Combined RMSE: Distance + Velocity + Acceleration - %s Attacks by V%d', ...
    attack_type, attacker_vehicle_id));
xticks(1:size(raw_combined_errors, 2));
xticklabels(attack_descriptions(1:size(raw_combined_errors, 2)));
yticks(1:length(non_attacker_ids));
yticklabels(arrayfun(@(x) sprintf('V%d', x), non_attacker_ids, 'UniformOutput', false));
xtickangle(45);
set(gca, 'FontSize', 8, 'TickLength', [0 0]);
ylim([0.5, length(non_attacker_ids) + 0.5]);

for i = 1:length(non_attacker_ids)
    for j = 1:size(raw_combined_errors, 2)
        text(j, i, sprintf('%.3f', raw_combined_errors(i, j)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'Color', 'black', 'FontWeight', 'bold', 'FontSize', 11, ...
            'EdgeColor', 'none');
    end
end

%% 4. RAW-NORMALIZED IMPACT HEATMAP
figure('Name', '05_raw_normalized_impact_heatmap', 'Position', [430, 210, 1000, 600]);
imagesc(raw_normalized_impact);
caxis([0, 1]);
c = colorbar;
c.Label.String = 'Raw combined RMSE / max RMSE';
c.Label.FontSize = 10;
c.Label.FontWeight = 'bold';
xlabel('Attack Cases');
ylabel('Vehicles');
title(sprintf('Raw-Normalized Impact Score - %s Attacks by V%d', attack_type, attacker_vehicle_id));
xticks(1:size(raw_normalized_impact, 2));
xticklabels(attack_descriptions(1:size(raw_normalized_impact, 2)));
yticks(1:length(non_attacker_ids));
yticklabels(arrayfun(@(x) sprintf('V%d', x), non_attacker_ids, 'UniformOutput', false));
xtickangle(45);
set(gca, 'FontSize', 8, 'TickLength', [0 0]);
ylim([0.5, length(non_attacker_ids) + 0.5]);

for i = 1:length(non_attacker_ids)
    for j = 1:size(raw_normalized_impact, 2)
        text_color = 'black';
        if raw_normalized_impact(i, j) < 0.35
            text_color = 'white';
        end
        text(j, i, sprintf('%.3f', raw_normalized_impact(i, j)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'Color', text_color, 'FontWeight', 'bold', 'FontSize', 11, ...
            'EdgeColor', 'none');
    end
end

% Write compact statistics for both full-run and attack-window RMSE to Excel.
metric_stats_headers = {'AttackType','AttackerVehicle','Case','Window','Window_Start','Window_End', ...
    'Metric','Mean','Std','Min','Max'};
metric_stats_rows = metric_stats_headers;
metric_names = {'Distance','Orientation','Velocity','Acceleration','Raw_Combined'};

for case_idx = 1:total_num_attack_cases
    case_nb_attack = attack_case_numbers(case_idx);
    for window_idx = 1:2
        if window_idx == 1
            window_label = 'Full';
            window_start = 0;
            window_end = Scenarios_config.simulation_time;
            data_cube = mean_errors_full;
            combined_data = raw_combined_errors_full;
        else
            window_label = 'AttackWindow';
            window_start = rmse_time_window(1);
            window_end = rmse_time_window(2);
            data_cube = mean_errors;
            combined_data = raw_combined_errors;
        end

        for metric_idx = 1:length(metric_names)
            if metric_idx <= 4
                metric_values = squeeze(data_cube(non_attacker_ids, metric_idx, case_idx));
            else
                metric_values = combined_data(:, case_idx);
            end

            metric_values = metric_values(:);
            valid_values = metric_values(~isnan(metric_values));
            if isempty(valid_values)
                mean_value = NaN;
                std_value = NaN;
                min_value = NaN;
                max_value = NaN;
            else
                mean_value = mean(valid_values);
                std_value = std(valid_values, 0);
                min_value = min(valid_values);
                max_value = max(valid_values);
            end

            metric_stats_rows(end+1, :) = { ...
                attack_type, ...
                sprintf('V%d', attacker_vehicle_id), ...
                sprintf('Case %d', case_nb_attack), ...
                window_label, ...
                window_start, ...
                window_end, ...
                metric_names{metric_idx}, ...
                mean_value, ...
                std_value, ...
                min_value, ...
                max_value};
        end
    end
end

writecell(metric_stats_rows, excel_filename, 'Sheet', stats_sheet_name, 'Range', 'A1');

% Write one row per attack case for trust and trust-weight diagnostics.
trust_weight_stats_headers = {'AttackType','AttackerVehicle','Case', ...
    'MeanTrust_Full','MeanTrust_PreAttack','MeanTrust_AttackWindow','MinTrust_AttackWindow', ...
    'TrustDrop_PreMinusAttack','DetectionRate','MeanDetectionTime', ...
    'MeanAttackerSourceInfluence_PreAttack','MeanAttackerSourceInfluence_AttackWindow','AttackerSourceInfluenceDrop_PreMinusAttack', ...
    'AttackerSourceInfluenceZeroRate_AttackWindow','MeanTrustedNeighborCount_AttackWindow', ...
    'FalsePositiveTrustRejection_Full','FalsePositiveTrustRejection_PreAttack','FalsePositiveTrustRejection_AttackWindow', ...
    'RMSE_Window_Start','RMSE_Window_End'};
trust_weight_stats_rows = trust_weight_stats_headers;

for case_idx = 1:total_num_attack_cases
    case_nb_attack = attack_case_numbers(case_idx);
    trust_weight_stats_rows(end+1, :) = { ...
        attack_type, ...
        sprintf('V%d', attacker_vehicle_id), ...
        sprintf('Case %d', case_nb_attack), ...
        mean_attacker_trust_full_by_case(case_idx), ...
        mean_attacker_trust_pre_by_case(case_idx), ...
        mean_attacker_trust_attack_by_case(case_idx), ...
        min_attacker_trust_attack_by_case(case_idx), ...
        mean_trust_degradation_by_case(case_idx), ...
        detection_rate_by_case(case_idx), ...
        mean_detection_time_by_case(case_idx), ...
        mean_attacker_weight_pre_by_case(case_idx), ...
        mean_attacker_weight_attack_by_case(case_idx), ...
        attacker_weight_reduction_by_case(case_idx), ...
        attacker_weight_zero_rate_attack_by_case(case_idx), ...
        mean_trusted_neighbor_count_attack_by_case(case_idx), ...
        false_positive_rejection_full_by_case(case_idx), ...
        false_positive_rejection_pre_by_case(case_idx), ...
        false_positive_rejection_attack_by_case(case_idx), ...
        rmse_time_window(1), ...
        rmse_time_window(2)};
end

writecell(trust_weight_stats_rows, excel_filename, 'Sheet', trust_weight_sheet_name, 'Range', 'A1');

%% COPY-READY PAPER MATRICES AND INTERPRETATION
paper_matrix_sheet_name = 'PaperMatrices';
paper_text_sheet_name = 'PaperText';
paper_vehicle_labels = arrayfun(@(x) sprintf('V%d', x), non_attacker_ids, 'UniformOutput', false)';
paper_matrix_width = total_num_attack_cases + 1;
blank_matrix_row = cell(1, paper_matrix_width);
matrix_header_row = [{'Vehicle'}, attack_descriptions(:)'];

paper_matrix_rows = cell(0, paper_matrix_width);
paper_matrix_rows(end + 1, :) = blank_matrix_row;
paper_matrix_rows{end, 1} = sprintf('Raw combined RMSE: distance + velocity + acceleration, attack window %.1f-%.1fs', ...
    rmse_time_window(1), rmse_time_window(2));
paper_matrix_rows(end + 1, :) = matrix_header_row;
paper_matrix_rows = [paper_matrix_rows; [paper_vehicle_labels, num2cell(raw_combined_errors)]];

paper_matrix_rows(end + 1, :) = blank_matrix_row;
paper_matrix_rows{end, 1} = 'Raw-normalized impact: raw combined RMSE divided by maximum raw combined RMSE';
paper_matrix_rows(end + 1, :) = matrix_header_row;
paper_matrix_rows = [paper_matrix_rows; [paper_vehicle_labels, num2cell(raw_normalized_impact)]];

paper_matrix_rows(end + 1, :) = blank_matrix_row;
paper_matrix_rows{end, 1} = 'Balanced component score: mean of separately normalized distance, velocity, and acceleration errors';
paper_matrix_rows(end + 1, :) = matrix_header_row;
paper_matrix_rows = [paper_matrix_rows; [paper_vehicle_labels, num2cell(balanced_component_scores)]];

writecell(paper_matrix_rows, excel_filename, 'Sheet', paper_matrix_sheet_name, 'Range', 'A1');

paper_avg_raw_per_case = mean(raw_combined_errors, 1);
paper_avg_raw_per_vehicle = mean(raw_combined_errors, 2);
paper_avg_raw_norm_per_case = mean(raw_normalized_impact, 1);
paper_avg_balanced_per_case = mean(balanced_component_scores, 1);
[paper_max_raw, paper_max_raw_linear_idx] = max(raw_combined_errors(:));
[paper_max_raw_vehicle_idx, paper_max_raw_case_idx] = ind2sub(size(raw_combined_errors), paper_max_raw_linear_idx);
[~, paper_strongest_raw_case_idx] = max(paper_avg_raw_per_case);
[~, paper_lowest_raw_case_idx] = min(paper_avg_raw_per_case);
[~, paper_most_affected_avg_idx] = max(paper_avg_raw_per_vehicle);
[~, paper_most_resilient_avg_idx] = min(paper_avg_raw_per_vehicle);
paper_raw_norm_case_parts = cell(1, total_num_attack_cases);
paper_balanced_case_parts = cell(1, total_num_attack_cases);
for case_idx = 1:total_num_attack_cases
    paper_raw_norm_case_parts{case_idx} = sprintf('Case %d %.3f', ...
        case_idx, paper_avg_raw_norm_per_case(case_idx));
    paper_balanced_case_parts{case_idx} = sprintf('Case %d %.3f', ...
        case_idx, paper_avg_balanced_per_case(case_idx));
end

paper_text_rows = {
    'Copy-ready metric definitions';
    sprintf('Raw combined RMSE is computed over %.1f-%.1fs as distance RMSE + velocity RMSE + acceleration RMSE. Orientation is excluded from this combined metric.', rmse_time_window(1), rmse_time_window(2));
    'Raw-normalized impact is raw combined RMSE divided by the largest raw combined RMSE in the displayed non-attacker matrix. It is bounded between 0 and 1 and preserves the raw severity ordering.';
    'Balanced component score first normalizes distance, velocity, and acceleration separately over the displayed non-attacker vehicles and then averages them with equal weights. It is bounded between 0 and 1, but it is not a physical-error magnitude.';
    'Copy-ready interpretation';
    sprintf('The largest single raw combined RMSE is %.3f for V%d under Case %d (%s).', paper_max_raw, non_attacker_ids(paper_max_raw_vehicle_idx), paper_max_raw_case_idx, attack_descriptions{paper_max_raw_case_idx});
    sprintf('By average raw combined RMSE, the strongest attack is Case %d (%s), with mean %.3f across non-attacker vehicles.', paper_strongest_raw_case_idx, attack_descriptions{paper_strongest_raw_case_idx}, paper_avg_raw_per_case(paper_strongest_raw_case_idx));
    sprintf('By average raw combined RMSE, the weakest attack is Case %d (%s), with mean %.3f across non-attacker vehicles.', paper_lowest_raw_case_idx, attack_descriptions{paper_lowest_raw_case_idx}, paper_avg_raw_per_case(paper_lowest_raw_case_idx));
    sprintf('By average raw combined RMSE, V%d is the most affected vehicle and V%d is the most resilient vehicle.', non_attacker_ids(paper_most_affected_avg_idx), non_attacker_ids(paper_most_resilient_avg_idx));
    sprintf('The average raw-normalized impact by case is: %s.', strjoin(paper_raw_norm_case_parts, '; '));
    sprintf('The average balanced component score by case is: %s.', strjoin(paper_balanced_case_parts, '; '));
    'Recommended figure usage';
    'Use the raw combined RMSE heatmap to report physical error magnitude.';
    'Use the raw-normalized impact heatmap or hybrid heatmap when a bounded 0-1 color scale is needed.';
    'Use the balanced component score heatmap only when the paper needs equal visual weight for distance, velocity, and acceleration components.';
    };
writecell(paper_text_rows, excel_filename, 'Sheet', paper_text_sheet_name, 'Range', 'A1');

%% 5. HYBRID HEATMAP: RAW-NORMALIZED COLORS WITH RAW ERROR VALUES
figure('Name', '06_hybrid_raw_normalized_heatmap', 'Position', [500, 300, 1000, 600]);

% Colors and text now come from the same raw-combined RMSE metric:
% color = raw combined RMSE normalized by the worst displayed cell,
% text = raw combined RMSE in physical units.
imagesc(raw_normalized_impact);
caxis([0, 1]);
c = colorbar;
c.Label.String = 'Raw combined RMSE / max RMSE';
c.Label.FontSize = 10;
c.Label.FontWeight = 'bold';
xlabel('Attack Cases');
ylabel('Vehicles');
title(sprintf('Hybrid Analysis: Raw-Normalized Color + Raw RMSE Text - %s Attacks by V%d', attack_type, attacker_vehicle_id));

% Set axis labels and formatting
xticks(1:size(raw_normalized_impact, 2));
xticklabels(hybrid_attack_x_labels(1:size(raw_normalized_impact, 2)));
num_vehicles = length(non_attacker_ids);
yticks(1:num_vehicles);
vehicle_labels = arrayfun(@(x) sprintf('V%d', x), non_attacker_ids, 'UniformOutput', false);
yticklabels(vehicle_labels);
if strcmpi(char(hybrid_attack_x_label_mode), 'case')
    xtickangle(0);
else
    xtickangle(45);
end
set(gca, 'FontSize', 8);
set(gca, 'TickLength', [0 0]);
ylim([0.5, num_vehicles + 0.5]);

% Add text annotations showing RAW COMBINED ERROR values
for i = 1:length(non_attacker_ids)
    for j = 1:size(raw_combined_errors, 2)
        raw_error = raw_combined_errors(i, j);
        text_color = 'black';
        if raw_normalized_impact(i, j) < 0.35
            text_color = 'white';
        end
        
        % Display raw error value as text
        text(j, i, sprintf('%.3f', raw_error), ...
             'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
             'Color', text_color, 'FontWeight', 'bold', 'FontSize', 11, ...
             'EdgeColor', 'none');
    end
end

% Add legend/explanation
annotation('textbox', [0.02, 0.95, 0.3, 0.05], ...
    'String', 'Colors: Raw RMSE / Max | Numbers: Raw Combined RMSE', ...
    'FontSize', 10, 'FontWeight', 'bold', ...
    'BackgroundColor', 'white', 'EdgeColor', 'black', ...
    'HorizontalAlignment', 'left');

%% 6. TRUST AND CONSENSUS WEIGHT RESPONSE OVER TIME
figure('Name', '07_trust_weight_response_over_time', 'Position', [600, 80, 1200, 850]);
case_colors = lines(total_num_attack_cases);

subplot(3, 1, 1);
hold on;
for case_idx = 1:total_num_attack_cases
    if ~isempty(case_time_vectors{case_idx}) && ~isempty(mean_attacker_trust_time_by_case{case_idx})
        plot(case_time_vectors{case_idx}, mean_attacker_trust_time_by_case{case_idx}, ...
            'LineWidth', 1.6, 'Color', case_colors(case_idx, :));
    end
end
xlim([0, Scenarios_config.simulation_time]);
ylim([0, 1.05]);
plot([t_star, t_star], [0, 1.05], 'k--', 'HandleVisibility', 'off');
plot([t_end, t_end], [0, 1.05], 'k--', 'HandleVisibility', 'off');
plot([0, Scenarios_config.simulation_time], [trust_threshold, trust_threshold], 'r:', ...
    'LineWidth', 1.4, 'HandleVisibility', 'off');
xlabel('Time (s)');
ylabel(sprintf('Mean Trust in V%d', attacker_vehicle_id));
title(sprintf('Mean Trust Response to Attacker V%d', attacker_vehicle_id));
legend(attack_descriptions, 'Location', 'bestoutside');
grid on;

subplot(3, 1, 2);
hold on;
max_direct_weight_trace = 0;
for case_idx = 1:total_num_attack_cases
    weight_trace = mean_attacker_direct_weight_time_by_case{case_idx};
    if ~isempty(case_time_vectors{case_idx}) && ~isempty(weight_trace)
        valid_weight_trace = weight_trace(~isnan(weight_trace));
        if ~isempty(valid_weight_trace)
            max_direct_weight_trace = max(max_direct_weight_trace, max(valid_weight_trace));
        end
        plot(case_time_vectors{case_idx}, weight_trace, ...
            'LineWidth', 1.6, 'Color', case_colors(case_idx, :));
    end
end
weight_ylim_max = max(0.25, max_direct_weight_trace + 0.05);
xlim([0, Scenarios_config.simulation_time]);
ylim([0, weight_ylim_max]);
plot([t_star, t_star], [0, weight_ylim_max], 'k--', 'HandleVisibility', 'off');
plot([t_end, t_end], [0, weight_ylim_max], 'k--', 'HandleVisibility', 'off');
xlabel('Time (s)');
ylabel(sprintf('Mean direct w0 for V%d', attacker_vehicle_id));
title(sprintf('Direct Attacker V%d Anchor Weight (w0)', attacker_vehicle_id));
legend(attack_descriptions, 'Location', 'bestoutside');
grid on;

subplot(3, 1, 3);
hold on;
max_source_weight_trace = 0;
for case_idx = 1:total_num_attack_cases
    weight_trace = mean_attacker_source_weight_time_by_case{case_idx};
    if ~isempty(case_time_vectors{case_idx}) && ~isempty(weight_trace)
        valid_weight_trace = weight_trace(~isnan(weight_trace));
        if ~isempty(valid_weight_trace)
            max_source_weight_trace = max(max_source_weight_trace, max(valid_weight_trace));
        end
        plot(case_time_vectors{case_idx}, weight_trace, ...
            'LineWidth', 1.6, 'Color', case_colors(case_idx, :));
    end
end
weight_ylim_max = max(0.25, max_source_weight_trace + 0.05);
xlim([0, Scenarios_config.simulation_time]);
ylim([0, weight_ylim_max]);
plot([t_star, t_star], [0, weight_ylim_max], 'k--', 'HandleVisibility', 'off');
plot([t_end, t_end], [0, weight_ylim_max], 'k--', 'HandleVisibility', 'off');
xlabel('Time (s)');
ylabel(sprintf('Mean V%d global-source weight', attacker_vehicle_id));
title(sprintf('Attacker V%d as Global Source to Other Targets', attacker_vehicle_id));
legend(attack_descriptions, 'Location', 'bestoutside');
grid on;

sgtitle(sprintf('Trust, Direct Anchor, and Global-Source Response During %.1f-%.1fs Attack Window', t_star, t_end), ...
    'FontSize', 14, 'FontWeight', 'bold');

%% 7. TRUST AND WEIGHT SUMMARY BY ATTACK CASE
figure('Name', '08_trust_weight_summary_by_case', 'Position', [650, 180, 1200, 700]);

subplot(2, 2, 1);
bar([mean_attacker_trust_pre_by_case(:), mean_attacker_trust_attack_by_case(:)]);
xticks(1:total_num_attack_cases);
xticklabels(attack_descriptions(1:total_num_attack_cases));
xtickangle(45);
ylim([0, 1.05]);
hold on;
plot([0.5, total_num_attack_cases + 0.5], [trust_threshold, trust_threshold], 'r:', 'LineWidth', 1.4);
ylabel(sprintf('Mean Trust in V%d', attacker_vehicle_id));
title('(a) Trust Drop During Attack Window');
legend({'Pre-attack', 'Attack window', 'Trust threshold'}, 'Location', 'best');
grid on;

subplot(2, 2, 2);
bar([ ...
    mean_attacker_direct_weight_pre_by_case(:), ...
    mean_attacker_direct_weight_attack_by_case(:), ...
    mean_attacker_source_weight_pre_by_case(:), ...
    mean_attacker_source_weight_attack_by_case(:)]);
xticks(1:total_num_attack_cases);
xticklabels(attack_descriptions(1:total_num_attack_cases));
xtickangle(45);
ylabel(sprintf('Weight involving V%d', attacker_vehicle_id));
title('(b) Direct Anchor vs Global-Source Suppression');
legend({'Direct pre', 'Direct attack', 'Global source pre', 'Global source attack'}, ...
    'Location', 'best');
grid on;

subplot(2, 2, 3);
bar(mean_trust_degradation_by_case);
xticks(1:total_num_attack_cases);
xticklabels(attack_descriptions(1:total_num_attack_cases));
xtickangle(45);
ylabel('Pre-attack Trust - Attack Trust');
title('(c) Mean Trust Degradation');
grid on;

subplot(2, 2, 4);
yyaxis left;
bar(100 * detection_rate_by_case);
ylim([0, 100]);
ylabel('Detection Rate (%)');
yyaxis right;
plot(1:total_num_attack_cases, mean_detection_time_by_case, '-o', 'LineWidth', 1.6);
ylabel('Mean Detection Time (s)');
xticks(1:total_num_attack_cases);
xticklabels(attack_descriptions(1:total_num_attack_cases));
xtickangle(45);
title('(d) Trust-Based Attack Detection');
grid on;

sgtitle(sprintf('Trust Mechanism Summary - %s Attacks by V%d', attack_type, attacker_vehicle_id), ...
    'FontSize', 14, 'FontWeight', 'bold');

%% 8. ATTACK-WINDOW TRUST AND WEIGHT HEATMAPS
figure('Name', '09_attack_window_trust_weight_heatmaps', 'Position', [700, 220, 1200, 500]);

subplot(1, 2, 1);
imagesc(attacker_trust_attack_by_vehicle);
caxis([0, 1]);
colorbar;
xlabel('Attack Cases');
ylabel('Evaluator Vehicles');
title(sprintf('Mean Trust in Attacker V%d During Attack', attacker_vehicle_id));
xticks(1:total_num_attack_cases);
xticklabels(attack_descriptions(1:total_num_attack_cases));
yticks(1:length(non_attacker_ids));
yticklabels(arrayfun(@(x) sprintf('V%d', x), non_attacker_ids, 'UniformOutput', false));
xtickangle(45);
set(gca, 'FontSize', 8, 'TickLength', [0 0]);
ylim([0.5, length(non_attacker_ids) + 0.5]);
for i = 1:length(non_attacker_ids)
    for j = 1:total_num_attack_cases
        text(j, i, sprintf('%.2f', attacker_trust_attack_by_vehicle(i, j)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'Color', 'black', 'FontWeight', 'bold', 'FontSize', 10);
    end
end

subplot(1, 2, 2);
imagesc(attacker_weight_attack_by_vehicle);
colorbar;
xlabel('Attack Cases');
ylabel('Evaluator Vehicles');
title(sprintf('Mean Attacker V%d Max Direct/Global-Source Influence During Attack', attacker_vehicle_id));
xticks(1:total_num_attack_cases);
xticklabels(attack_descriptions(1:total_num_attack_cases));
yticks(1:length(non_attacker_ids));
yticklabels(arrayfun(@(x) sprintf('V%d', x), non_attacker_ids, 'UniformOutput', false));
xtickangle(45);
set(gca, 'FontSize', 8, 'TickLength', [0 0]);
ylim([0.5, length(non_attacker_ids) + 0.5]);
for i = 1:length(non_attacker_ids)
    for j = 1:total_num_attack_cases
        text(j, i, sprintf('%.2f', attacker_weight_attack_by_vehicle(i, j)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'Color', 'black', 'FontWeight', 'bold', 'FontSize', 10);
    end
end

sgtitle(sprintf('Attack-Window Trust and Combined Influence Detail (%.1f-%.1fs)', t_star, t_end), ...
    'FontSize', 14, 'FontWeight', 'bold');

%% 9. SUMMARY STATISTICS FOR PAPER
fprintf('\n=== SUMMARY STATISTICS FOR PAPER ===\n');
fprintf('Attack Type: %s, Attacker: V%d\n', attack_type, attacker_vehicle_id);
fprintf('Total Attack Cases Tested: %d\n', total_num_attack_cases);
fprintf('Attack Duration: %.1f seconds (t=%.1fs to %.1fs)\n', t_end-t_star, t_star, t_end);
fprintf('RMSE Window: %.1f seconds to %.1f seconds\n', rmse_time_window(1), rmse_time_window(2));

fprintf('\n=== TRUST AND WEIGHT RESPONSE SUMMARY ===\n');
for case_idx = 1:total_num_attack_cases
    fprintf(['Case %d (%s): Trust pre %.3f -> attack %.3f (drop %.3f), ', ...
        'direct w0 pre %.3f -> attack %.3f, ', ...
        'global-source pre %.3f -> attack %.3f (drop %.3f), ', ...
        'global-source zero-weight %.1f%%, benign false rejection %.1f%%, ', ...
        'trusted neighbors %.2f, detection %.1f%%'], ...
        case_idx, attack_descriptions{case_idx}, ...
        mean_attacker_trust_pre_by_case(case_idx), ...
        mean_attacker_trust_attack_by_case(case_idx), ...
        mean_trust_degradation_by_case(case_idx), ...
        mean_attacker_direct_weight_pre_by_case(case_idx), ...
        mean_attacker_direct_weight_attack_by_case(case_idx), ...
        mean_attacker_source_weight_pre_by_case(case_idx), ...
        mean_attacker_source_weight_attack_by_case(case_idx), ...
        attacker_source_weight_reduction_by_case(case_idx), ...
        100 * attacker_source_weight_zero_rate_attack_by_case(case_idx), ...
        100 * false_positive_rejection_attack_by_case(case_idx), ...
        mean_trusted_neighbor_count_attack_by_case(case_idx), ...
        100 * detection_rate_by_case(case_idx));
    if isnan(mean_detection_time_by_case(case_idx))
        fprintf(', no threshold crossing\n');
    else
        fprintf(', mean detection %.2fs\n', mean_detection_time_by_case(case_idx));
    end
end

%% RAW COMBINED ERROR ANALYSIS
fprintf('\n=== RAW AVERAGE COMBINED ERROR ANALYSIS ===\n');

% Get the raw combined error data for analysis
raw_error_data = raw_combined_errors;

% Find attack case with highest raw error
[max_raw_per_case, ~] = max(raw_error_data, [], 1);  % Max raw error across vehicles for each case
[overall_max_raw, most_severe_raw_case] = max(max_raw_per_case);
fprintf('Highest Raw Error Attack: Case %d (%s) - Max Error: %.4f\n', ...
    most_severe_raw_case, attack_descriptions{most_severe_raw_case}, overall_max_raw);

% Find attack case with the lowest single raw error cell
[min_raw_per_case, min_raw_vehicle_idx_by_case] = min(raw_error_data, [], 1);  % Min raw error across vehicles for each case
[overall_min_raw, least_severe_raw_case] = min(min_raw_per_case);
least_raw_vehicle = non_attacker_ids(min_raw_vehicle_idx_by_case(least_severe_raw_case));
fprintf('Lowest Single Raw Error Cell: Case %d (%s), V%d - Min Error: %.4f\n', ...
    least_severe_raw_case, attack_descriptions{least_severe_raw_case}, least_raw_vehicle, overall_min_raw);

% Find vehicle with highest raw error across all attacks
[max_raw_per_vehicle, ~] = max(raw_error_data, [], 2);  % Max raw error across cases for each vehicle
[vehicle_max_raw, most_affected_vehicle_idx] = max(max_raw_per_vehicle);
most_affected_vehicle = non_attacker_ids(most_affected_vehicle_idx);
fprintf('Most Affected Vehicle: V%d - Max Raw Error: %.4f\n', ...
    most_affected_vehicle, vehicle_max_raw);

% Find vehicle with lowest single raw error across all attacks
[min_raw_per_vehicle, ~] = min(raw_error_data, [], 2);  % Min raw error across cases for each vehicle
[vehicle_min_raw, least_affected_vehicle_idx] = min(min_raw_per_vehicle);
least_affected_vehicle = non_attacker_ids(least_affected_vehicle_idx);
fprintf('Lowest Single Raw Error Vehicle: V%d - Min Raw Error: %.4f\n', ...
    least_affected_vehicle, vehicle_min_raw);

% Average raw error per attack case
avg_raw_per_case = mean(raw_error_data, 1);
fprintf('\nAverage Raw Combined Error by Attack Case:\n');
for i = 1:length(attack_descriptions)
    fprintf('  Case %d (%s): %.4f\n', i, attack_descriptions{i}, avg_raw_per_case(i));
end

% Average raw error per vehicle
avg_raw_per_vehicle = mean(raw_error_data, 2);
fprintf('\nAverage Raw Combined Error by Vehicle:\n');
for i = 1:length(non_attacker_ids)
    fprintf('  V%d: %.4f\n', non_attacker_ids(i), avg_raw_per_vehicle(i));
end

[~, most_affected_avg_vehicle_idx] = max(avg_raw_per_vehicle);
[~, most_resilient_avg_vehicle_idx] = min(avg_raw_per_vehicle);
fprintf('Most affected vehicle by average raw error: V%d\n', non_attacker_ids(most_affected_avg_vehicle_idx));
fprintf('Most resilient vehicle by average raw error: V%d\n', non_attacker_ids(most_resilient_avg_vehicle_idx));

% Raw error severity classification based on actual error magnitudes
critical_error_threshold = 2.0;   % Combined error > 2.0 is critical
high_error_threshold = 1.0;       % Combined error > 1.0 is high
moderate_error_threshold = 0.5;   % Combined error > 0.5 is moderate

fprintf('\n=== RAW ERROR SEVERITY CLASSIFICATION ===\n');
fprintf('Critical Error (>%.1f): System performance severely compromised\n', critical_error_threshold);
fprintf('High Error (%.1f-%.1f): Significant performance degradation\n', high_error_threshold, critical_error_threshold);
fprintf('Moderate Error (%.1f-%.1f): Noticeable but manageable impact\n', moderate_error_threshold, high_error_threshold);
fprintf('Low Error (<%.1f): Minimal impact on system performance\n', moderate_error_threshold);

critical_cases = find(max_raw_per_case > critical_error_threshold);
high_cases = find(max_raw_per_case > high_error_threshold & max_raw_per_case <= critical_error_threshold);
moderate_cases = find(max_raw_per_case > moderate_error_threshold & max_raw_per_case <= high_error_threshold);
low_cases = find(max_raw_per_case <= moderate_error_threshold);

if ~isempty(critical_cases)
    fprintf('\nCRITICAL ERROR CASES: ');
    for i = 1:length(critical_cases)
        fprintf('Case %d (%s) ', critical_cases(i), attack_descriptions{critical_cases(i)});
    end
    fprintf('\n');
end

if ~isempty(high_cases)
    fprintf('HIGH ERROR CASES: ');
    for i = 1:length(high_cases)
        fprintf('Case %d (%s) ', high_cases(i), attack_descriptions{high_cases(i)});
    end
    fprintf('\n');
end

if ~isempty(moderate_cases)
    fprintf('MODERATE ERROR CASES: ');
    for i = 1:length(moderate_cases)
        fprintf('Case %d (%s) ', moderate_cases(i), attack_descriptions{moderate_cases(i)});
    end
    fprintf('\n');
end

if ~isempty(low_cases)
    fprintf('LOW ERROR CASES: ');
    for i = 1:length(low_cases)
        fprintf('Case %d (%s) ', low_cases(i), attack_descriptions{low_cases(i)});
    end
    fprintf('\n');
end

% Error magnitude distribution analysis
fprintf('\n=== ERROR MAGNITUDE DISTRIBUTION ===\n');
overall_avg_raw = mean(raw_error_data(:));
overall_std_raw = std(raw_error_data(:));
overall_max_raw_all = max(raw_error_data(:));
overall_min_raw_all = min(raw_error_data(:));

fprintf('Overall Statistics:\n');
fprintf('  Mean Combined Error: %.4f\n', overall_avg_raw);
fprintf('  Standard Deviation: %.4f\n', overall_std_raw);
fprintf('  Maximum Error: %.4f\n', overall_max_raw_all);
fprintf('  Minimum Error: %.4f\n', overall_min_raw_all);
fprintf('  Error Range: %.4f\n', overall_max_raw_all - overall_min_raw_all);

%% ERROR COMPONENT CONTRIBUTION ANALYSIS
fprintf('\n=== ERROR COMPONENT CONTRIBUTION ANALYSIS ===\n');

% Calculate average contribution of each error type to total combined error
% Extract individual error components for non-attacker vehicles
dist_errors_only = squeeze(mean_errors(non_attacker_ids, 1, :));  % Distance errors
vel_errors_only = squeeze(mean_errors(non_attacker_ids, 3, :));   % Velocity errors  
acc_errors_only = squeeze(mean_errors(non_attacker_ids, 4, :));   % Acceleration errors

% Calculate average contribution percentages
avg_dist_error = mean(dist_errors_only(:));
avg_vel_error = mean(vel_errors_only(:));
avg_acc_error = mean(acc_errors_only(:));
total_avg_error = avg_dist_error + avg_vel_error + avg_acc_error;

% Calculate percentage contributions
dist_contribution = (avg_dist_error / total_avg_error) * 100;
vel_contribution = (avg_vel_error / total_avg_error) * 100;
acc_contribution = (avg_acc_error / total_avg_error) * 100;

fprintf('Average Error Component Contributions to RAW COMBINED ERROR:\n');
fprintf('  Distance Error: %.4f (%.1f%%)\n', avg_dist_error, dist_contribution);
fprintf('  Velocity Error: %.4f (%.1f%%)\n', avg_vel_error, vel_contribution);
fprintf('  Acceleration Error: %.4f (%.1f%%)\n', avg_acc_error, acc_contribution);
fprintf('  Total Combined: %.4f (100.0%%)\n', total_avg_error);

% Find dominant error component
[max_contribution, max_idx] = max([dist_contribution, vel_contribution, acc_contribution]);
component_names = {'Distance', 'Velocity', 'Acceleration'};
fprintf('\nDominant Error Component: %s (%.1f%% of total error)\n', component_names{max_idx}, max_contribution);

% Analyze contribution variability across cases
fprintf('\nContribution Variability Across Attack Cases:\n');
for case_idx = 1:size(dist_errors_only, 2)
    case_dist = mean(dist_errors_only(:, case_idx));
    case_vel = mean(vel_errors_only(:, case_idx));
    case_acc = mean(acc_errors_only(:, case_idx));
    case_total = case_dist + case_vel + case_acc;
    
    if case_total > 0  % Avoid division by zero
        case_dist_pct = (case_dist / case_total) * 100;
        case_vel_pct = (case_vel / case_total) * 100;
        case_acc_pct = (case_acc / case_total) * 100;
        
        fprintf('  %s: Dist %.1f%%, Vel %.1f%%, Acc %.1f%%\n', ...
            attack_descriptions{case_idx}, case_dist_pct, case_vel_pct, case_acc_pct);
    end
end

%% NORMALIZED HEATMAP ANALYSIS
fprintf('\n=== RAW-NORMALIZED IMPACT ANALYSIS ===\n');
fprintf('Raw-normalized impact = raw combined RMSE / max(raw combined RMSE).\n');
fprintf('This score is bounded in [0, 1] and preserves raw RMSE ordering.\n');

raw_norm_data = raw_normalized_impact;
avg_raw_norm_per_case = mean(raw_norm_data, 1);
avg_raw_norm_per_vehicle = mean(raw_norm_data, 2);
[max_raw_norm_per_case, ~] = max(raw_norm_data, [], 1);
[overall_max_raw_norm, most_severe_raw_norm_case] = max(max_raw_norm_per_case);
[~, most_impacted_raw_norm_vehicle_idx] = max(avg_raw_norm_per_vehicle);
[~, most_resilient_raw_norm_vehicle_idx] = min(avg_raw_norm_per_vehicle);

fprintf('Highest Raw-Normalized Attack: Case %d (%s) - Max Score: %.3f\n', ...
    most_severe_raw_norm_case, attack_descriptions{most_severe_raw_norm_case}, overall_max_raw_norm);
fprintf('Average Raw-Normalized Impact by Attack Case:\n');
for i = 1:length(attack_descriptions)
    fprintf('  Case %d (%s): %.3f\n', i, attack_descriptions{i}, avg_raw_norm_per_case(i));
end
fprintf('Most impacted vehicle by average raw-normalized impact: V%d\n', ...
    non_attacker_ids(most_impacted_raw_norm_vehicle_idx));
fprintf('Most resilient vehicle by average raw-normalized impact: V%d\n', ...
    non_attacker_ids(most_resilient_raw_norm_vehicle_idx));

fprintf('\n=== BALANCED COMPONENT SCORE ANALYSIS ===\n');
fprintf('Balanced component score = mean of separately normalized distance, velocity, and acceleration errors.\n');
fprintf('This score is bounded in [0, 1], but it is not a physical error magnitude.\n');

% Get the balanced component scores for analysis
impact_data = balanced_component_scores;

% Find most severe attack case
[max_impact_per_case, ~] = max(impact_data, [], 1);  % Max impact across vehicles for each case
[overall_max_impact, most_severe_case] = max(max_impact_per_case);
fprintf('Highest Balanced-Score Attack: Case %d (%s) - Max Score: %.3f\n', ...
    most_severe_case, attack_descriptions{most_severe_case}, overall_max_impact);

% Find least severe attack case  
[min_impact_per_case, ~] = min(impact_data, [], 1);  % Min impact across vehicles for each case
[overall_min_impact, least_severe_case] = min(min_impact_per_case);
fprintf('Lowest Single Balanced-Score Cell: Case %d (%s) - Min Score: %.3f\n', ...
    least_severe_case, attack_descriptions{least_severe_case}, overall_min_impact);

% Find most impacted vehicle across all attacks
[max_impact_per_vehicle, ~] = max(impact_data, [], 2);  % Max impact across cases for each vehicle
[vehicle_max_impact, most_impacted_vehicle_idx] = max(max_impact_per_vehicle);
most_impacted_vehicle = non_attacker_ids(most_impacted_vehicle_idx);
fprintf('Most Impacted Vehicle by Single Balanced-Score Cell: V%d - Max Score: %.3f\n', ...
    most_impacted_vehicle, vehicle_max_impact);

% Find least impacted vehicle across all attacks by its single smallest cell
[min_impact_per_vehicle, ~] = min(impact_data, [], 2);  % Min impact across cases for each vehicle  
[vehicle_min_impact, least_impacted_vehicle_idx] = min(min_impact_per_vehicle);
least_impacted_vehicle = non_attacker_ids(least_impacted_vehicle_idx);
fprintf('Lowest Single Balanced-Score Vehicle: V%d - Min Score: %.3f\n', ...
    least_impacted_vehicle, vehicle_min_impact);

% Average impact per attack case
avg_impact_per_case = mean(impact_data, 1);
fprintf('\nAverage Impact Score by Attack Case:\n');
for i = 1:length(attack_descriptions)
    fprintf('  Case %d (%s): %.3f\n', i, attack_descriptions{i}, avg_impact_per_case(i));
end

% Average impact per vehicle
avg_impact_per_vehicle = mean(impact_data, 2);
fprintf('\nAverage Impact Score by Vehicle:\n');
for i = 1:length(non_attacker_ids)
    fprintf('  V%d: %.3f\n', non_attacker_ids(i), avg_impact_per_vehicle(i));
end

[~, most_impacted_avg_balanced_idx] = max(avg_impact_per_vehicle);
[~, most_resilient_avg_balanced_idx] = min(avg_impact_per_vehicle);
fprintf('Most impacted vehicle by average balanced score: V%d\n', non_attacker_ids(most_impacted_avg_balanced_idx));
fprintf('Most resilient vehicle by average balanced score: V%d\n', non_attacker_ids(most_resilient_avg_balanced_idx));

fprintf('\n=== SCORE SCALE NOTE ===\n');
fprintf('Use raw combined RMSE for physical severity thresholds.\n');
fprintf('Use raw-normalized impact for bounded 0-1 visualization of the same raw severity ordering.\n');
fprintf('Use balanced component score only to compare distance, velocity, and acceleration on equal normalized scales.\n');

% Find most/least vulnerable cases using the physical raw combined RMSE.
overall_errors = avg_raw_per_case;
[max_error, max_case] = max(overall_errors);
[min_error, min_case] = min(overall_errors);

fprintf('\nStrongest Raw Attack: Case %d (%s) - Average Raw Combined RMSE: %.4f\n', max_case, attack_descriptions{max_case}, max_error);
fprintf('Weakest Raw Attack: Case %d (%s) - Average Raw Combined RMSE: %.4f\n', min_case, attack_descriptions{min_case}, min_error);

% Vehicle resilience ranking based on physical raw combined RMSE.
vehicle_resilience = avg_raw_per_vehicle;
[~, resilience_rank] = sort(vehicle_resilience);
fprintf('\nVehicle Resilience Ranking (most to least resilient):\n');
for i = 1:length(resilience_rank)
    v_id = non_attacker_ids(resilience_rank(i));
    fprintf('  %d. V%d (Average Raw Combined RMSE: %.4f)\n', i, v_id, vehicle_resilience(resilience_rank(i)));
end

fprintf('\n=== COPY-READY PAPER OUTPUT ===\n');
fprintf('Matrices written to Excel sheet: %s\n', paper_matrix_sheet_name);
fprintf('Interpretation text written to Excel sheet: %s\n', paper_text_sheet_name);
fprintf('Recommended main paper figure: Hybrid heatmap with raw-normalized color and raw RMSE text.\n');
fprintf('Recommended supporting figure: Balanced component-normalized heatmap for component-balanced comparison.\n');

%% 10. SAVE RUN LOGS, SUMMARY REPORT, AND FIGURE ARTIFACTS
summary_filename = fullfile(run_results_dir, 'summary_report.txt');
summary_fid = fopen(summary_filename, 'w');
if summary_fid < 0
    warning('Could not open summary report for writing: %s', summary_filename);
else
    fprintf(summary_fid, 'RUN SUMMARY\n');
    fprintf(summary_fid, '===========\n');
    fprintf(summary_fid, 'Run folder: %s\n', run_results_dir);
    fprintf(summary_fid, 'Grouped under: %s\n', results_group_dir);
    fprintf(summary_fid, 'Excel workbook: %s\n', excel_filename);
    fprintf(summary_fid, 'Attack type: %s\n', attack_type);
    fprintf(summary_fid, 'Data type attack: %s\n', data_type_attack);
    fprintf(summary_fid, 'Attacker vehicle: V%d\n', attacker_vehicle_id);
    fprintf(summary_fid, 'Attack window: %.3f s to %.3f s\n', t_star, t_end);
    fprintf(summary_fid, 'RMSE window: %.3f s to %.3f s\n', rmse_time_window(1), rmse_time_window(2));
    fprintf(summary_fid, 'Cases tested: %d\n\n', total_num_attack_cases);

    fprintf(summary_fid, 'RAW COMBINED RMSE SUMMARY\n');
    fprintf(summary_fid, 'Strongest raw attack: Case %d (%s), average raw combined RMSE %.6f\n', ...
        max_case, attack_descriptions{max_case}, max_error);
    fprintf(summary_fid, 'Weakest raw attack: Case %d (%s), average raw combined RMSE %.6f\n', ...
        min_case, attack_descriptions{min_case}, min_error);
    fprintf(summary_fid, 'Largest single raw combined RMSE: %.6f\n', overall_max_raw_all);
    fprintf(summary_fid, 'Overall mean raw combined RMSE: %.6f\n', overall_avg_raw);
    fprintf(summary_fid, 'Overall standard deviation: %.6f\n\n', overall_std_raw);

    fprintf(summary_fid, 'AVERAGE RAW COMBINED RMSE BY CASE\n');
    for case_idx = 1:total_num_attack_cases
        fprintf(summary_fid, 'Case %d (%s): %.6f\n', ...
            case_idx, attack_descriptions{case_idx}, avg_raw_per_case(case_idx));
    end
    fprintf(summary_fid, '\nAVERAGE RAW COMBINED RMSE BY VEHICLE\n');
    for vehicle_idx = 1:length(non_attacker_ids)
        fprintf(summary_fid, 'V%d: %.6f\n', non_attacker_ids(vehicle_idx), avg_raw_per_vehicle(vehicle_idx));
    end

    fprintf(summary_fid, '\nTRUST AND WEIGHT RESPONSE BY CASE\n');
    for case_idx = 1:total_num_attack_cases
        fprintf(summary_fid, ['Case %d (%s): trust pre %.6f, trust attack %.6f, ', ...
            'drop %.6f, global-source pre %.6f, global-source attack %.6f, ', ...
            'source drop %.6f, source zero-rate %.2f%%, detection %.2f%%'], ...
            case_idx, attack_descriptions{case_idx}, ...
            mean_attacker_trust_pre_by_case(case_idx), ...
            mean_attacker_trust_attack_by_case(case_idx), ...
            mean_trust_degradation_by_case(case_idx), ...
            mean_attacker_source_weight_pre_by_case(case_idx), ...
            mean_attacker_source_weight_attack_by_case(case_idx), ...
            attacker_source_weight_reduction_by_case(case_idx), ...
            100 * attacker_source_weight_zero_rate_attack_by_case(case_idx), ...
            100 * detection_rate_by_case(case_idx));
        if isnan(mean_detection_time_by_case(case_idx))
            fprintf(summary_fid, ', no threshold crossing\n');
        else
            fprintf(summary_fid, ', mean detection %.6f s\n', mean_detection_time_by_case(case_idx));
        end
    end

    fprintf(summary_fid, '\nARTIFACTS\n');
    fprintf(summary_fid, 'Excel workbook: %s\n', excel_filename);
    fprintf(summary_fid, 'Command-window log: %s\n', run_log_filename);
    fprintf(summary_fid, 'Figures folder: %s\n', figures_dir);
    fprintf(summary_fid, 'Grouped results folder: %s\n', results_group_dir);
    fclose(summary_fid);
end

if ~exist('all_case_trust_logs', 'var')
    all_case_trust_logs = {};
end
if ~exist('all_case_scenarios', 'var')
    all_case_scenarios = {};
end
run_metadata = struct( ...
    'run_timestamp', run_timestamp, ...
    'run_results_dir', run_results_dir, ...
    'attack_type', attack_type, ...
    'data_type_attack', data_type_attack, ...
    'attacker_vehicle_id', attacker_vehicle_id, ...
    'victim_id', victim_id, ...
    'rmse_time_window', rmse_time_window, ...
    'attack_case_numbers', attack_case_numbers, ...
    'simulation_time', Scenarios_config.simulation_time, ...
    'dt', Scenarios_config.dt);
simulation_log_filename = fullfile(logs_dir, 'simulation_logs.mat');
save(simulation_log_filename, 'run_metadata', 'all_case_state_logs', ...
    'all_case_input_logs', 'all_case_trust_logs', 'all_case_scenarios', '-v7.3');

figure_manifest_filename = save_all_open_figures_to_results(figures_dir);

latest_run_text = sprintf(['Latest run folder:\n%s\n\n', ...
    'Data type attack: %s\nAttacker vehicle: V%d\nAttack type: %s\n', ...
    'Excel workbook:\n%s\nSummary report:\n%s\nFigures folder:\n%s\n'], ...
    run_results_dir, data_type_attack, attacker_vehicle_id, attack_type, ...
    excel_filename, summary_filename, figures_dir);
write_text_file(fullfile(results_root, 'latest_run_folder.txt'), latest_run_text);
write_text_file(fullfile(results_group_dir, 'latest_run_folder.txt'), latest_run_text);
write_text_file(fullfile(run_results_dir, 'open_this_results_folder.bat'), ...
    sprintf('@echo off\r\nexplorer "%s"\r\n', run_results_dir));
write_text_file(fullfile(results_root, 'open_latest_results.bat'), ...
    sprintf('@echo off\r\nexplorer "%s"\r\n', run_results_dir));
write_text_file(fullfile(results_group_dir, 'open_latest_results.bat'), ...
    sprintf('@echo off\r\nexplorer "%s"\r\n', run_results_dir));

append_run_index(fullfile(results_root, 'results_index.csv'), run_metadata, ...
    results_group_dir, run_results_dir, excel_filename, summary_filename, figures_dir);
append_run_index(fullfile(results_group_dir, 'results_index.csv'), run_metadata, ...
    results_group_dir, run_results_dir, excel_filename, summary_filename, figures_dir);

fprintf('\nResults saved to run folder: %s\n', run_results_dir);
fprintf('Grouped result folder: %s\n', results_group_dir);
fprintf('Latest-run pointer: %s\n', fullfile(results_group_dir, 'latest_run_folder.txt'));
fprintf('Open latest shortcut: %s\n', fullfile(results_group_dir, 'open_latest_results.bat'));
fprintf('Excel workbook: %s\n', excel_filename);
fprintf('Summary report: %s\n', summary_filename);
fprintf('Simulation logs MAT: %s\n', simulation_log_filename);
fprintf('Figures saved to: %s\n', figures_dir);
fprintf('Figure manifest: %s\n', figure_manifest_filename);
fprintf('Command-window log: %s\n', run_log_filename);
fprintf('Use these figures and statistics in your paper!\n');
diary off;

function manifest_filename = save_all_open_figures_to_results(figures_dir)
    if ~exist(figures_dir, 'dir')
        mkdir(figures_dir);
    end

    figures = findall(groot, 'Type', 'figure');
    manifest_filename = fullfile(figures_dir, 'figure_manifest.csv');
    manifest_rows = {'Index', 'FigureNumber', 'FigureName', 'FIG_File', 'EPS_File', 'ESP_File'};

    if isempty(figures)
        writecell(manifest_rows, manifest_filename);
        return;
    end

    figure_numbers = zeros(numel(figures), 1);
    for idx = 1:numel(figures)
        if isprop(figures(idx), 'Number')
            figure_numbers(idx) = figures(idx).Number;
        else
            figure_numbers(idx) = idx;
        end
    end
    [figure_numbers, order] = sort(figure_numbers);
    figures = figures(order);

    for idx = 1:numel(figures)
        figure_handle = figures(idx);
        figure_name = get(figure_handle, 'Name');
        if isempty(figure_name)
            figure_name = sprintf('figure_%02d', idx);
        end
        base_filename = sanitize_result_filename(sprintf('%02d_%s', idx, figure_name));
        fig_filename = fullfile(figures_dir, [base_filename, '.fig']);
        eps_filename = fullfile(figures_dir, [base_filename, '.eps']);
        esp_filename = fullfile(figures_dir, [base_filename, '.esp']);

        savefig(figure_handle, fig_filename);
        set(figure_handle, 'PaperPositionMode', 'auto');
        print(figure_handle, eps_filename, '-depsc', '-painters', '-r300');
        copyfile(eps_filename, esp_filename);

        manifest_rows(end + 1, :) = {idx, figure_numbers(idx), figure_name, fig_filename, eps_filename, esp_filename};
    end

    writecell(manifest_rows, manifest_filename);
end

function write_text_file(filename, text_content)
    fid = fopen(filename, 'w');
    if fid < 0
        warning('Could not open text file for writing: %s', filename);
        return;
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '%s', text_content);
end

function append_run_index(index_filename, run_metadata, results_group_dir, run_results_dir, excel_filename, summary_filename, figures_dir)
    is_new_file = ~exist(index_filename, 'file');
    if ~is_new_file
        file_info = dir(index_filename);
        is_new_file = isempty(file_info) || file_info.bytes == 0;
    end

    fid = fopen(index_filename, 'a');
    if fid < 0
        warning('Could not open results index for writing: %s', index_filename);
        return;
    end
    cleanup = onCleanup(@() fclose(fid));

    if is_new_file
        header = {'RunTimestamp','DataTypeAttack','AttackerVehicle','AttackType', ...
            'RMSEWindowStart','RMSEWindowEnd','GroupFolder','RunFolder', ...
            'ExcelWorkbook','SummaryReport','FiguresFolder'};
        fprintf(fid, '%s\n', strjoin(header, ','));
    end

    row = { ...
        run_metadata.run_timestamp, ...
        run_metadata.data_type_attack, ...
        sprintf('V%d', run_metadata.attacker_vehicle_id), ...
        run_metadata.attack_type, ...
        run_metadata.rmse_time_window(1), ...
        run_metadata.rmse_time_window(2), ...
        results_group_dir, ...
        run_results_dir, ...
        excel_filename, ...
        summary_filename, ...
        figures_dir};
    row_text = cellfun(@csv_quote, row, 'UniformOutput', false);
    fprintf(fid, '%s\n', strjoin(row_text, ','));
end

function out = csv_quote(value)
    if isnumeric(value)
        if isscalar(value)
            value_text = sprintf('%.15g', value);
        else
            value_text = mat2str(value);
        end
    elseif isstring(value)
        value_text = char(value);
    elseif ischar(value)
        value_text = value;
    else
        value_text = char(string(value));
    end
    out = ['"', strrep(value_text, '"', '""'), '"'];
end

function rate = finite_fraction_below(values, threshold)
    values = values(isfinite(values));
    if isempty(values)
        rate = NaN;
    else
        rate = mean(values < threshold);
    end
end

function safe_name = sanitize_result_filename(raw_name)
    safe_name = regexprep(char(raw_name), '[^A-Za-z0-9_.-]', '_');
    safe_name = regexprep(safe_name, '_+', '_');
    safe_name = regexprep(safe_name, '^_|_$', '');
    if isempty(safe_name)
        safe_name = 'figure';
    end
end
