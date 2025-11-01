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


%% Set Attack senario
attack_module = Attack_module(Scenarios_config.dt);
%% Comunication Module
center_communication = CenterCommunication(attack_module);



t_star = 10;
t_end = 15;
attacker_vehicle_id = 2;
victim_id = -1;
data_type_attack = "local"; % "local" , "global",
attack_type = "Bogus"; % "DoS" , "faulty" , "scaling" , "Collusion" ,"Bogus" , "POS" , "VEL" , "ACC"


% ──────────────────────────────────────────────────────────────────────────────
% 3) Excel logging setup
excel_filename = sprintf('Results_%s_Attacker_V%d.xlsx', attack_type, attacker_vehicle_id);
sheet_name     = 'Summary';
headers = {'AttackType','AttackerVehicle','Case','Vehicle', ...
    'Mean_Distance','Mean_Orientation','Mean_Velocity','Mean_Acceleration', ...
    'Mean_Trust_Score','Trust_Degradation','Attack_Detection_Time'};

% Write headers if file does not exist
if ~isfile(excel_filename)
    writecell(headers, excel_filename, 'Sheet', sheet_name, 'Range', 'A1');
end

% Read existing rows to find next empty row
raw = readcell(excel_filename, 'Sheet', sheet_name);
row_index = size(raw,1) + 1;



% car1 is in the same lane with ego vehicle , lane lowest
initial_lane_id = 1;
direction_flag = 0; % 1 stands for changing to the left adjacent lane, 0 stands for keeping the current lane, -1 stands for changing

% Fixed lead scenario for all attack cases
Scenarios_config.set_Lead_Senarios("constant");




start_attack_senarios_index = 1;
last_attack_senarios_index = 3;  % All Bogus cases (1-10)

total_num_attack_cases = last_attack_senarios_index - start_attack_senarios_index ;

vehicle_labels = {'V1', 'V2', 'V3', 'V4'};
scenario_labels = arrayfun(@(x) sprintf("Case %d", x), start_attack_senarios_index:total_num_attack_cases, 'UniformOutput', false);
metric_labels = {'Distance Error (m)', 'Orientation Error (rad)', 'Velocity Error (m/s)','Acc Error (m/s)'};

mean_errors = zeros(length(vehicle_labels), length(metric_labels),length(scenario_labels)); % 4 vehicles × 4 metrics × nb  scenarios


is_plot_each_case = false;

% Get vehicle IDs
all_vehicle_ids = [1, 2, 3, 4];

% Get IDs excluding attacker
non_attacker_ids = all_vehicle_ids(all_vehicle_ids ~= attacker_vehicle_id);

for case_nb_attack = start_attack_senarios_index:total_num_attack_cases
    fprintf('Running %s attack by V%d, Case %d/%d...\n', ...
        attack_type, attacker_vehicle_id, case_nb_attack, total_num_attack_cases);

    attack_module = Atk_Scenarios(attack_module , attack_type ,data_type_attack,case_nb_attack , t_star, t_end, attacker_vehicle_id,victim_id );


    car1 = Vehicle(1, "None", param_sys, [80; 0.5 * lane_width; 0; 23 ; 0], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

    % car2 , car3 is in the middle lane, lane middle
    car2 = Vehicle(2, "IDM", param_sys, [60; 0.5 * lane_width; 0; 26; 0], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

    car3 = Vehicle(3, "IDM", param_sys, [40; 0.5 * lane_width; 0; 26; 0], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

    % car4 is in the lane highest
    car4 = Vehicle(4, "IDM", param_sys, [20; 0.5 * lane_width; 0; 26; 0], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);




    platton_vehicles = [car1; car2; car3; car4];

    car1.assign_neighbor_vehicle(platton_vehicles, [],"None", center_communication, graph);
    car2.assign_neighbor_vehicle(platton_vehicles, [],"CACC", center_communication, graph);
    car3.assign_neighbor_vehicle(platton_vehicles, [],"CACC", center_communication, graph );
    car4.assign_neighbor_vehicle(platton_vehicles, [],"CACC", center_communication, graph);


    %% define a simulator and start simulation
    simulator0 = Simulator(straightLanes, [] , platton_vehicles, Scenarios_config.dt , IsShowAnimation );
    [state_log, input_log] = simulator0.startSimulation(Scenarios_config.simulation_time,t_star, t_end, attacker_vehicle_id);

    % plot
    if (is_plot_each_case)
        car1.plot_ground_error_global_est(platton_vehicles);
        car2.plot_ground_error_global_est(platton_vehicles);
        car3.plot_ground_error_global_est(platton_vehicles);
        car4.plot_ground_error_global_est(platton_vehicles);
    end
    % After the simulation, calculate errors for each vehicle
    % vehicles_to_evaluate = [car2, car3, car4];  % Update based on which vehicles to evaluate

    % Select vehicles to evaluate based on IDs
    vehicles_to_evaluate = platton_vehicles(non_attacker_ids);

    all_global_dist_errors = []; % Initialize arrays to store global errors
    all_global_theta_errors = []; % global orientation errors of vehicles to evaluate
    all_global_vel_errors = [];
    all_global_acc_errors = [];
    
    % Trust-related metrics
    all_trust_scores = [];
    trust_degradation = [];
    attack_detection_times = [];

    for k = 1:length(vehicles_to_evaluate)
        v = vehicles_to_evaluate(k);
        [dist_err, theta_err, vel_err,global_acc_err] = v.observer.calculate_global_errors();  % Call global error calculation
        all_global_dist_errors = [all_global_dist_errors, dist_err];
        all_global_theta_errors = [all_global_theta_errors, theta_err];
        all_global_vel_errors = [all_global_vel_errors, vel_err];
        all_global_acc_errors = [all_global_acc_errors, global_acc_err];
        
        % Extract trust scores (assuming trust model exists)
        try
            if isfield(v, 'trust_model') && ~isempty(v.trust_model)
                trust_scores = v.trust_model.trust_sample_log;
                all_trust_scores = [all_trust_scores; mean(trust_scores, 2)];
                
                % Calculate trust degradation during attack
                attack_start_idx = round(t_star / Scenarios_config.dt);
                attack_end_idx = round(t_end / Scenarios_config.dt);
                if length(trust_scores) > attack_end_idx
                    pre_attack_trust = mean(trust_scores(1:attack_start_idx, attacker_vehicle_id));
                    during_attack_trust = mean(trust_scores(attack_start_idx:attack_end_idx, attacker_vehicle_id));
                    trust_degradation = [trust_degradation; pre_attack_trust - during_attack_trust];
                    
                    % Detect when trust drops below threshold
                    trust_threshold_detection = 0.7; % Adjust based on your system
                    detection_idx = find(trust_scores(attack_start_idx:end, attacker_vehicle_id) < trust_threshold_detection, 1);
                    if ~isempty(detection_idx)
                        attack_detection_times = [attack_detection_times; detection_idx * Scenarios_config.dt];
                    else
                        attack_detection_times = [attack_detection_times; NaN];
                    end
                else
                    trust_degradation = [trust_degradation; NaN];
                    attack_detection_times = [attack_detection_times; NaN];
                end
            else
                all_trust_scores = [all_trust_scores; NaN];
                trust_degradation = [trust_degradation; NaN];
                attack_detection_times = [attack_detection_times; NaN];
            end
        catch
            all_trust_scores = [all_trust_scores; NaN];
            trust_degradation = [trust_degradation; NaN];
            attack_detection_times = [attack_detection_times; NaN];
        end
    end

    % n_vehicles_eval = length(vehicles_to_evaluate);
    %
    % n_steps =  size(vehicles_to_evaluate(1).observer.est_global_state_log, 2);  % assuming dist_err is a time series
    % all_global_dist_errors = zeros(n_steps, n_vehicles_eval);
    % all_global_theta_errors = zeros(n_steps, n_vehicles_eval);
    % all_global_vel_errors   = zeros(n_steps, n_vehicles_eval);
    % all_global_acc_errors   = zeros(n_steps, n_vehicles_eval);
    %
    % for k = 1:n_vehicles_eval
    %     v = vehicles_to_evaluate(k);
    %     [dist_err, theta_err, vel_err, acc_err] = v.observer.calculate_global_errors();
    %     all_global_dist_errors(:,k) = dist_err;
    %     all_global_theta_errors(:,k) = theta_err;
    %     all_global_vel_errors(:,k)   = vel_err;
    %     all_global_acc_errors(:,k)   = acc_err;
    % end





    % Calculate overall mean of global errors for all vehicles in this scenario
    mean_dist = mean(all_global_dist_errors,2);
    mean_theta = mean(all_global_theta_errors,2);
    mean_vel = mean(all_global_vel_errors,2);
    mean_acc = mean(all_global_acc_errors,2);

    % write one row per evaluating vehicle
    for vi = 1:length(non_attacker_ids)
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
        
        row_data = {
            attack_type, ...
            sprintf('V%d', attacker_vehicle_id), ...
            sprintf('Case %d', case_nb_attack), ...
            sprintf('V%d', non_attacker_ids(vi)), ...
            mean_dist(vi), ...
            mean_theta(vi), ...
            mean_vel(vi), ...
            mean_acc(vi), ...
            trust_score, ...
            trust_deg, ...
            detection_time
        };
        writecell(row_data, excel_filename, 'Sheet', sheet_name, ...
                  'Range', sprintf('A%d', row_index));
        row_index = row_index + 1;
    end



    % Store the errors for plotting
    mean_errors(:, :, case_nb_attack) = [mean_dist, mean_theta, mean_vel,mean_acc];
    
    % Store trust data for direct analysis - COLLECT ALL CASES
    if ~exist('all_case_trust_logs', 'var')
        all_case_trust_logs = cell(total_num_attack_cases, 1);
        all_case_vehicles = cell(total_num_attack_cases, 1);
        all_case_scenarios = cell(total_num_attack_cases, 1);
    end
    
    % Store complete trust data for each case
    all_case_trust_logs{case_nb_attack} = struct();
    all_case_vehicles{case_nb_attack} = platton_vehicles;
    all_case_scenarios{case_nb_attack} = struct('t_start', t_star, 't_end', t_end, 'dt', Scenarios_config.dt);
    
    % Collect trust logs from all vehicles for this case
    for v_idx = 1:length(platton_vehicles)
        vehicle = platton_vehicles(v_idx);
        if isfield(vehicle, 'trust_log')
            all_case_trust_logs{case_nb_attack}.(sprintf('vehicle_%d', v_idx)) = vehicle.trust_log;
        end
        
        % Also store trip model data
        if isfield(vehicle, 'trip_models')
            for tm_idx = 1:length(vehicle.trip_models)
                if ~isempty(vehicle.trip_models{tm_idx})
                    trust_model = vehicle.trip_models{tm_idx};
                    field_name = sprintf('trust_model_v%d_to_v%d', v_idx, tm_idx);
                    all_case_trust_logs{case_nb_attack}.(field_name) = struct( ...
                        'trust_samples', trust_model.trust_sample_log, ...
                        'final_scores', trust_model.final_score_log, ...
                        'gamma_cross', trust_model.gamma_cross_log, ...
                        'gamma_local', trust_model.gamma_local_log, ...
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

% Define Bogus attack case descriptions for better labeling
attack_descriptions = {
    'Vel Scale -50%', 'Vel Scale +50%', 'Pos Bias -15m', 'Pos Bias +20m', ...
    'Vel Linear +0.1', 'Pos Sinusoidal', 'Pos Fault 10m', 'Vel Fault 2.5', ...
    'Acc Fault 0.5', 'Vel Bias +2'
};

%% 1. COMPREHENSIVE ESTIMATION ERROR ANALYSIS
figure('Position', [100, 100, 1200, 800]);

% Subplot 1: Distance Estimation Error
subplot(2,2,1);
data = squeeze(mean_errors(non_attacker_ids, 1, :))';  % Distance errors for non-attackers
bar(data);
xticklabels(attack_descriptions(1:size(data,1)));
xtickangle(45);
ylabel('Distance Error (m)');
title('(a) Distance Estimation Error');
legend(cellstr("V" + string(non_attacker_ids)), 'Location', 'best');
grid on;

% Subplot 2: Velocity Estimation Error  
subplot(2,2,2);
data = squeeze(mean_errors(non_attacker_ids, 3, :))';  % Velocity errors
bar(data);
xticklabels(attack_descriptions(1:size(data,1)));
xtickangle(45);
ylabel('Velocity Error (m/s)');
title('(b) Velocity Estimation Error');
legend(cellstr("V" + string(non_attacker_ids)), 'Location', 'best');
grid on;

% Subplot 3: Orientation Estimation Error
subplot(2,2,3);
data = squeeze(mean_errors(non_attacker_ids, 2, :))';  % Orientation errors
bar(data);
xticklabels(attack_descriptions(1:size(data,1)));
xtickangle(45);
ylabel('Orientation Error (rad)');
title('(c) Orientation Estimation Error');
legend(cellstr("V" + string(non_attacker_ids)), 'Location', 'best');
grid on;

% Subplot 4: Acceleration Estimation Error
subplot(2,2,4);
data = squeeze(mean_errors(non_attacker_ids, 4, :))';  % Acceleration errors
bar(data);
xticklabels(attack_descriptions(1:size(data,1)));
xtickangle(45);
ylabel('Acceleration Error (m/s²)');
title('(d) Acceleration Estimation Error');
legend(cellstr("V" + string(non_attacker_ids)), 'Location', 'best');
grid on;

sgtitle(sprintf('Distributed Estimation Performance Under %s Attacks (Attacker: V%d)', attack_type, attacker_vehicle_id), 'FontSize', 14, 'FontWeight', 'bold');

%% 2. MEANINGFUL ATTACK IMPACT HEATMAPS
% Option 1: Separate heatmaps for each error type (RECOMMENDED)
figure('Position', [200, 200, 1400, 1000]);

for metric_idx = 1:4
    subplot(2, 2, metric_idx);
    
    % Extract data for this specific metric across all vehicles and cases
    metric_data = squeeze(mean_errors(non_attacker_ids, metric_idx, :));  % vehicles × cases
    
    % Create heatmap
    imagesc(metric_data);
    colorbar;
    
    % Customize labels and title
    xlabel('Attack Cases');
    ylabel('Vehicles');
    title(sprintf('%s Impact', metric_labels{metric_idx}));
    
    % Set tick labels
    if size(metric_data, 2) <= length(attack_descriptions)
        xticklabels(attack_descriptions(1:size(metric_data, 2)));
    end
    yticklabels(cellstr("V" + string(non_attacker_ids)));
    xtickangle(45);
    
    % Add text annotations for better readability
    for i = 1:size(metric_data, 1)
        for j = 1:size(metric_data, 2)
            if metric_data(i,j) > mean(metric_data(:))
                text_color = 'white';
            else
                text_color = 'black';
            end
            text(j, i, sprintf('%.3f', metric_data(i,j)), ...
                 'HorizontalAlignment', 'center', ...
                 'Color', text_color);
        end
    end
end
sgtitle(sprintf('Attack Impact Analysis by Error Type - %s Attacks by V%d', attack_type, attacker_vehicle_id), ...
        'FontSize', 14, 'FontWeight', 'bold');

% Option 2: Normalized Combined Impact Heatmap
figure('Position', [300, 100, 1000, 600]);

% Normalize each metric to [0,1] scale before combining
normalized_errors = zeros(size(mean_errors));
for metric_idx = 1:4
    metric_slice = mean_errors(:, metric_idx, :);
    min_val = min(metric_slice(:));
    max_val = max(metric_slice(:));
    if max_val > min_val
        normalized_errors(:, metric_idx, :) = (metric_slice - min_val) / (max_val - min_val);
    end
end

% Calculate weighted impact score (you can adjust weights based on importance)
weights = [0.3, 0.2, 0.3, 0.2];  % [distance, orientation, velocity, acceleration]
impact_scores = squeeze(sum(normalized_errors .* reshape(weights, 1, 4, 1), 2));

% Create normalized impact heatmap
imagesc(impact_scores(non_attacker_ids, :));
colorbar;
xlabel('Attack Cases');
ylabel('Vehicles');
title(sprintf('Normalized Combined Impact Score - %s Attacks by V%d', attack_type, attacker_vehicle_id));
xticklabels(attack_descriptions(1:size(impact_scores, 2)));
yticklabels(cellstr("V" + string(non_attacker_ids)));
xtickangle(45);

% Add text annotations
for i = 1:length(non_attacker_ids)
    for j = 1:size(impact_scores, 2)
        score = impact_scores(non_attacker_ids(i), j);
        if score > 0.5
            text_color = 'white';
        else
            text_color = 'black';
        end
        text(j, i, sprintf('%.2f', score), ...
             'HorizontalAlignment', 'center', ...
             'Color', text_color);
    end
end

%% 3. VULNERABILITY ANALYSIS PER VEHICLE
figure('Position', [300, 300, 1000, 400]);
for v_idx = 1:length(non_attacker_ids)
    subplot(1, length(non_attacker_ids), v_idx);
    vehicle_errors = squeeze(mean_errors(non_attacker_ids(v_idx), :, :));  % All metrics for this vehicle
    bar(vehicle_errors');
    xticklabels(attack_descriptions(1:size(vehicle_errors,2)));
    xtickangle(45);
    ylabel('Error Magnitude');
    title(sprintf('V%d Vulnerability', non_attacker_ids(v_idx)));
    legend(metric_labels, 'Location', 'best', 'FontSize', 8);
    grid on;
end
sgtitle('Individual Vehicle Vulnerability Analysis', 'FontSize', 14, 'FontWeight', 'bold');

%% 4. COMPREHENSIVE TRUST ANALYSIS FOR ALL CASES
% Create trust evolution plots for ALL attack cases
if exist('all_case_trust_logs', 'var') && ~isempty(all_case_trust_logs{end})
    
    fprintf('\n=== COMPREHENSIVE TRUST ANALYSIS FOR ALL CASES ===\n');
    
    n_cases = length(all_case_trust_logs);
    
    % CREATE INDIVIDUAL TRUST EVOLUTION PLOTS FOR EACH CASE
    for case_idx = 1:n_cases
        if ~isempty(all_case_trust_logs{case_idx})
            
            vehicles_data = all_case_vehicles{case_idx};
            scenario_data = all_case_scenarios{case_idx};
            
            % Create trust evolution plot for this specific case
            figure('Position', [100 + case_idx*50, 100 + case_idx*50, 1200, 800]);
            
            attack_start_idx = round(scenario_data.t_start / scenario_data.dt);
            attack_end_idx = round(scenario_data.t_end / scenario_data.dt);
            
            % Plot for each non-attacker vehicle
            for v_idx = 1:length(non_attacker_ids)
                vehicle_id = non_attacker_ids(v_idx);
                
                subplot(2, 2, v_idx);
                
                if vehicle_id <= length(vehicles_data) && isfield(vehicles_data(vehicle_id), 'trust_log')
                    trust_log = vehicles_data(vehicle_id).trust_log;
                    
                    if size(trust_log, 3) >= attacker_vehicle_id && size(trust_log, 2) > 1
                        trust_in_attacker = squeeze(trust_log(1, :, attacker_vehicle_id));
                        time_vector = (1:length(trust_in_attacker)) * scenario_data.dt;
                        
                        plot(time_vector, trust_in_attacker, 'b-', 'LineWidth', 2);
                        hold on;
                        
                        % Highlight attack period
                        attack_region = [scenario_data.t_start, scenario_data.t_start, scenario_data.t_end, scenario_data.t_end];
                        y_limits = [min(trust_in_attacker)-0.1, max(trust_in_attacker)+0.1];
                        attack_height = [y_limits(1), y_limits(2), y_limits(2), y_limits(1)];
                        patch(attack_region, attack_height, 'red', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
                        
                        % Add trust threshold line
                        trust_threshold = 0.7;
                        plot([time_vector(1), time_vector(end)], [trust_threshold, trust_threshold], 'r--', 'LineWidth', 1.5);
                        
                        xlabel('Time (s)');
                        ylabel('Trust Score');
                        title(sprintf('V%d → V%d (Attacker)', vehicle_id, attacker_vehicle_id));
                        grid on;
                        ylim(y_limits);
                        
                        % Calculate metrics for this case
                        if attack_start_idx <= length(trust_in_attacker) && attack_end_idx <= length(trust_in_attacker)
                            pre_attack_trust = mean(trust_in_attacker(1:attack_start_idx));
                            during_attack_trust = mean(trust_in_attacker(attack_start_idx:attack_end_idx));
                            degradation = pre_attack_trust - during_attack_trust;
                            
                            text(0.05, 0.95, sprintf('Degradation: %.3f', degradation), ...
                                 'Units', 'normalized', 'BackgroundColor', 'white', 'FontSize', 9);
                            
                            % Find detection time
                            attack_trust = trust_in_attacker(attack_start_idx:end);
                            detection_idx = find(attack_trust < trust_threshold, 1);
                            if ~isempty(detection_idx)
                                detection_time = detection_idx * scenario_data.dt;
                                detection_time_abs = scenario_data.t_start + detection_time;
                                plot(detection_time_abs, trust_threshold, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'red');
                                text(0.05, 0.85, sprintf('Detection: %.2fs', detection_time), ...
                                     'Units', 'normalized', 'BackgroundColor', 'white', 'FontSize', 9);
                            else
                                text(0.05, 0.85, 'No Detection', ...
                                     'Units', 'normalized', 'BackgroundColor', 'yellow', 'FontSize', 9);
                            end
                        end
                        
                        if v_idx == 1
                            legend('Trust Score', 'Attack Period', 'Detection Threshold', 'Detection Point', ...
                                   'Location', 'best', 'FontSize', 8);
                        end
                    end
                end
            end
            
            case_desc = attack_descriptions{case_idx};
            sgtitle(sprintf('Trust Evolution - Case %d: %s Attack by V%d', ...
                    case_idx, case_desc, attacker_vehicle_id), ...
                    'FontSize', 14, 'FontWeight', 'bold');
        end
    end
    
    % CREATE COMPARISON PLOT ACROSS ALL CASES
    figure('Position', [400, 400, 1400, 900]);
    
    % Calculate summary metrics across all cases
    trust_degradation_summary = zeros(n_cases, length(non_attacker_ids));
    detection_time_summary = zeros(n_cases, length(non_attacker_ids));
    
    for case_idx = 1:n_cases
        if ~isempty(all_case_trust_logs{case_idx})
            vehicles_data = all_case_vehicles{case_idx};
            scenario_data = all_case_scenarios{case_idx};
            
            attack_start_idx = round(scenario_data.t_start / scenario_data.dt);
            attack_end_idx = round(scenario_data.t_end / scenario_data.dt);
            
            for v_idx = 1:length(non_attacker_ids)
                vehicle_id = non_attacker_ids(v_idx);
                
                if vehicle_id <= length(vehicles_data) && isfield(vehicles_data(vehicle_id), 'trust_log')
                    trust_log = vehicles_data(vehicle_id).trust_log;
                    
                    if size(trust_log, 3) >= attacker_vehicle_id && size(trust_log, 2) > attack_end_idx
                        trust_in_attacker = squeeze(trust_log(1, :, attacker_vehicle_id));
                        
                        % Calculate degradation
                        pre_attack_trust = mean(trust_in_attacker(1:attack_start_idx));
                        during_attack_trust = mean(trust_in_attacker(attack_start_idx:attack_end_idx));
                        trust_degradation_summary(case_idx, v_idx) = pre_attack_trust - during_attack_trust;
                        
                        % Calculate detection time
                        trust_threshold = 0.7;
                        attack_trust = trust_in_attacker(attack_start_idx:end);
                        detection_idx = find(attack_trust < trust_threshold, 1);
                        if ~isempty(detection_idx)
                            detection_time_summary(case_idx, v_idx) = detection_idx * scenario_data.dt;
                        else
                            detection_time_summary(case_idx, v_idx) = NaN;
                        end
                    end
                end
            end
        end
    end
    
    % Create SUMMARY comparison plots
    case_labels = arrayfun(@(x) attack_descriptions{x}, 1:n_cases, 'UniformOutput', false);
    
    % Plot 1: Trust degradation comparison
    subplot(2, 3, 1);
    imagesc(trust_degradation_summary');
    colorbar;
    title('Trust Degradation Across All Cases');
    xlabel('Attack Cases');
    ylabel('Vehicles');
    xticks(1:n_cases);
    xticklabels(case_labels);
    yticklabels(cellstr("V" + string(non_attacker_ids)));
    xtickangle(45);
    
    % Add values on heatmap
    for i = 1:size(trust_degradation_summary, 1)
        for j = 1:size(trust_degradation_summary, 2)
            if ~isnan(trust_degradation_summary(i, j))
                text(i, j, sprintf('%.2f', trust_degradation_summary(i, j)), ...
                     'HorizontalAlignment', 'center', 'Color', 'white', 'FontSize', 8);
            end
        end
    end
    
    % Plot 2: Detection time comparison
    subplot(2, 3, 2);
    imagesc(detection_time_summary');
    colorbar;
    title('Detection Time (s) Across All Cases');
    xlabel('Attack Cases');
    ylabel('Vehicles');
    xticks(1:n_cases);
    xticklabels(case_labels);
    yticklabels(cellstr("V" + string(non_attacker_ids)));
    xtickangle(45);
    
    % Add values on heatmap
    for i = 1:size(detection_time_summary, 1)
        for j = 1:size(detection_time_summary, 2)
            if ~isnan(detection_time_summary(i, j))
                text(i, j, sprintf('%.1f', detection_time_summary(i, j)), ...
                     'HorizontalAlignment', 'center', 'Color', 'white', 'FontSize', 8);
            end
        end
    end
    
    % Plot 3: Average trust degradation per case
    subplot(2, 3, 3);
    mean_degradation = nanmean(trust_degradation_summary, 2);
    std_degradation = nanstd(trust_degradation_summary, 0, 2);
    bar(1:n_cases, mean_degradation, 'FaceColor', [0.3, 0.7, 0.9]);
    hold on;
    errorbar(1:n_cases, mean_degradation, std_degradation, 'k.', 'LineWidth', 1.5);
    xlabel('Attack Cases');
    ylabel('Mean Trust Degradation');
    title('Average Trust Response by Case');
    xticks(1:n_cases);
    xticklabels(case_labels);
    xtickangle(45);
    grid on;
    
    % Plot 4: Average detection time per case
    subplot(2, 3, 4);
    mean_detection = nanmean(detection_time_summary, 2);
    std_detection = nanstd(detection_time_summary, 0, 2);
    bar(1:n_cases, mean_detection, 'FaceColor', [0.9, 0.5, 0.2]);
    hold on;
    errorbar(1:n_cases, mean_detection, std_detection, 'k.', 'LineWidth', 1.5);
    xlabel('Attack Cases');
    ylabel('Mean Detection Time (s)');
    title('Average Detection Speed by Case');
    xticks(1:n_cases);
    xticklabels(case_labels);
    xtickangle(45);
    grid on;
    
    % Plot 5: Trust degradation vs detection correlation
    subplot(2, 3, 5);
    valid_data = ~isnan(trust_degradation_summary(:)) & ~isnan(detection_time_summary(:));
    if sum(valid_data) > 1
        scatter(trust_degradation_summary(valid_data), detection_time_summary(valid_data), 100, 'filled');
        xlabel('Trust Degradation');
        ylabel('Detection Time (s)');
        title('Trust Response vs Detection Speed');
        
        % Add case labels to points
        valid_indices = find(valid_data);
        for k = 1:length(valid_indices)
            [case_idx, vehicle_idx] = ind2sub(size(trust_degradation_summary), valid_indices(k));
            text(trust_degradation_summary(valid_indices(k)), detection_time_summary(valid_indices(k)), ...
                 sprintf('C%d-V%d', case_idx, non_attacker_ids(vehicle_idx)), ...
                 'FontSize', 7, 'HorizontalAlignment', 'left');
        end
        grid on;
    end
    
    % Plot 6: Case-by-case comparison
    subplot(2, 3, 6);
    for case_idx = 1:n_cases
        plot(1:length(non_attacker_ids), trust_degradation_summary(case_idx, :), ...
             'o-', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', sprintf('Case %d', case_idx));
        hold on;
    end
    xlabel('Vehicles');
    ylabel('Trust Degradation');
    title('Trust Degradation by Vehicle & Case');
    xticks(1:length(non_attacker_ids));
    xticklabels(cellstr("V" + string(non_attacker_ids)));
    legend('Location', 'best');
    grid on;
    
    sgtitle(sprintf('Trust Analysis Summary - %s Attacks by V%d (All Cases)', attack_type, attacker_vehicle_id), ...
            'FontSize', 14, 'FontWeight', 'bold');
    
    % Print detailed statistics
    fprintf('Attack Type: %s, Attacker: V%d\n', attack_type, attacker_vehicle_id);
    fprintf('Cases Analyzed: %d\n', n_cases);
    
    for case_idx = 1:n_cases
        fprintf('\nCase %d (%s):\n', case_idx, case_labels{case_idx});
        case_degradation = trust_degradation_summary(case_idx, :);
        case_detection = detection_time_summary(case_idx, :);
        
        valid_deg = case_degradation(~isnan(case_degradation));
        valid_det = case_detection(~isnan(case_detection));
        
        if ~isempty(valid_deg)
            fprintf('  Trust Degradation: Mean=%.3f, Max=%.3f\n', mean(valid_deg), max(valid_deg));
        end
        if ~isempty(valid_det)
            fprintf('  Detection Time: Mean=%.2fs, Min=%.2fs\n', mean(valid_det), min(valid_det));
        end
    end
    
else
    fprintf('Warning: No direct trust data collected from simulations\n');
    fprintf('Using last simulation data for trust analysis...\n');
    
    % Use the last simulation's trust data for demonstration
    if exist('platton_vehicles', 'var') && ~isempty(platton_vehicles)
        
        % Create trust evolution plot during attack
        figure('Position', [400, 400, 1200, 800]);
        
        attack_start_idx = round(t_star / Scenarios_config.dt);
        attack_end_idx = round(t_end / Scenarios_config.dt);
        time_vector = (1:size(platton_vehicles(1).trust_log, 2)) * Scenarios_config.dt;
        
        % Plot trust evolution for each vehicle toward the attacker
        for v_idx = 1:length(non_attacker_ids)
            vehicle_id = non_attacker_ids(v_idx);
            
            subplot(2, 2, v_idx);
            
            if vehicle_id <= length(platton_vehicles) && size(platton_vehicles(vehicle_id).trust_log, 3) >= attacker_vehicle_id
                trust_in_attacker = squeeze(platton_vehicles(vehicle_id).trust_log(1, :, attacker_vehicle_id));
                
                plot(time_vector, trust_in_attacker, 'b-', 'LineWidth', 2);
                hold on;
                
                % Highlight attack period
                attack_region = [t_star, t_star, t_end, t_end];
                y_limits = ylim;
                attack_height = [y_limits(1), y_limits(2), y_limits(2), y_limits(1)];
                patch(attack_region, attack_height, 'red', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
                
                % Add trust threshold line
                trust_threshold = 0.7;
                plot([time_vector(1), time_vector(end)], [trust_threshold, trust_threshold], 'r--', 'LineWidth', 1.5);
                
                xlabel('Time (s)');
                ylabel('Trust Score');
                title(sprintf('V%d Trust in V%d (Attacker)', vehicle_id, attacker_vehicle_id));
                grid on;
                legend('Trust Score', 'Attack Period', 'Detection Threshold', 'Location', 'best');
                
                % Calculate and display metrics
                pre_attack_trust = mean(trust_in_attacker(1:attack_start_idx));
                during_attack_trust = mean(trust_in_attacker(attack_start_idx:attack_end_idx));
                degradation = pre_attack_trust - during_attack_trust;
                
                text(0.05, 0.95, sprintf('Degradation: %.3f', degradation), ...
                     'Units', 'normalized', 'BackgroundColor', 'white', 'FontSize', 10);
                
                % Find detection time
                attack_trust = trust_in_attacker(attack_start_idx:end);
                detection_idx = find(attack_trust < trust_threshold, 1);
                if ~isempty(detection_idx)
                    detection_time = detection_idx * Scenarios_config.dt;
                    detection_time_abs = t_star + detection_time;
                    plot(detection_time_abs, trust_threshold, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'red');
                    text(0.05, 0.85, sprintf('Detection: %.2fs', detection_time), ...
                         'Units', 'normalized', 'BackgroundColor', 'white', 'FontSize', 10);
                end
            end
        end
        
        sgtitle(sprintf('Trust Evolution During %s Attack by V%d (Case %d)', ...
                attack_type, attacker_vehicle_id, total_num_attack_cases), ...
                'FontSize', 14, 'FontWeight', 'bold');
        
        % Additional trust analysis using trip models
        figure('Position', [500, 500, 1400, 600]);
        
        % Plot detailed trust components for one representative vehicle-pair
        if length(non_attacker_ids) > 0
            repr_vehicle_id = non_attacker_ids(1);  % Use first non-attacker
            if repr_vehicle_id <= length(platton_vehicles) && ~isempty(platton_vehicles(repr_vehicle_id).trip_models)
                
                % Find trust model for the attacker
                trust_model = [];
                for tm_idx = 1:length(platton_vehicles(repr_vehicle_id).trip_models)
                    if ~isempty(platton_vehicles(repr_vehicle_id).trip_models{tm_idx})
                        trust_model = platton_vehicles(repr_vehicle_id).trip_models{tm_idx};
                        break;
                    end
                end
                
                if ~isempty(trust_model) && ~isempty(trust_model.trust_sample_log)
                    time_steps = 1:length(trust_model.trust_sample_log);
                    time_vector_tm = time_steps * Scenarios_config.dt;
                    
                    % Plot 1: Trust components
                    subplot(2, 3, 1);
                    plot(time_vector_tm, trust_model.trust_sample_log, 'b-', 'LineWidth', 1.5);
                    hold on;
                    plot(time_vector_tm, trust_model.final_score_log, 'r-', 'LineWidth', 1.5);
                    xlabel('Time (s)');
                    ylabel('Trust Score');
                    title('Trust Sample vs Final Score');
                    legend('Trust Sample', 'Final Score', 'Location', 'best');
                    grid on;
                    
                    % Plot 2: Trust factors
                    subplot(2, 3, 2);
                    if ~isempty(trust_model.gamma_cross_log)
                        plot(time_vector_tm, trust_model.gamma_cross_log, 'g-', 'LineWidth', 1.5);
                        hold on;
                    end
                    if ~isempty(trust_model.gamma_local_log)
                        plot(time_vector_tm, trust_model.gamma_local_log, 'm-', 'LineWidth', 1.5);
                    end
                    xlabel('Time (s)');
                    ylabel('Trust Factor');
                    title('Trust Validation Factors');
                    legend('Cross-validation', 'Local consistency', 'Location', 'best');
                    grid on;
                    
                    % Plot 3: Individual scores
                    subplot(2, 3, 3);
                    if ~isempty(trust_model.v_score_log)
                        plot(time_vector_tm, trust_model.v_score_log, 'b-', 'LineWidth', 1);
                        hold on;
                    end
                    if ~isempty(trust_model.d_score_log)
                        plot(time_vector_tm, trust_model.d_score_log, 'r-', 'LineWidth', 1);
                    end
                    if ~isempty(trust_model.a_score_log)
                        plot(time_vector_tm, trust_model.a_score_log, 'g-', 'LineWidth', 1);
                    end
                    xlabel('Time (s)');
                    ylabel('Score');
                    title('Individual Trust Metrics');
                    legend('Velocity', 'Distance', 'Acceleration', 'Location', 'best');
                    grid on;
                    
                    sgtitle(sprintf('Detailed Trust Analysis: V%d evaluating V%d', repr_vehicle_id, attacker_vehicle_id), ...
                            'FontSize', 12, 'FontWeight', 'bold');
                end
            end
        end
        
        % Use existing plotting functions for comprehensive view
        fprintf('Generating comprehensive trust plots using existing functions...\n');
        simulator0.plot_all_trust_log(platton_vehicles);
        
        % Generate individual trust model plots
        fprintf('Generating individual trust model plots...\n');
        for v_idx = 1:length(non_attacker_ids)
            vehicle_id = non_attacker_ids(v_idx);
            if vehicle_id <= length(platton_vehicles) && ~isempty(platton_vehicles(vehicle_id).trip_models)
                for tm_idx = 1:length(platton_vehicles(vehicle_id).trip_models)
                    if ~isempty(platton_vehicles(vehicle_id).trip_models{tm_idx})
                        try
                            platton_vehicles(vehicle_id).trip_models{tm_idx}.plot_trust_log(vehicle_id, tm_idx);
                        catch
                            fprintf('Warning: Could not plot trust log for V%d->V%d\n', vehicle_id, tm_idx);
                        end
                    end
                end
            end
        end
    end
end

%% 5. SUMMARY STATISTICS FOR PAPER
fprintf('\n=== SUMMARY STATISTICS FOR PAPER ===\n');
fprintf('Attack Type: %s, Attacker: V%d\n', attack_type, attacker_vehicle_id);
fprintf('Total Attack Cases Tested: %d\n', total_num_attack_cases);
fprintf('Attack Duration: %.1f seconds (t=%.1fs to %.1fs)\n', t_end-t_star, t_star, t_end);

% Find most/least vulnerable cases
overall_errors = squeeze(mean(mean(mean_errors(non_attacker_ids, :, :), 1), 2));
[max_error, max_case] = max(overall_errors);
[min_error, min_case] = min(overall_errors);

fprintf('\nMost Severe Attack: Case %d (%s) - Average Error: %.4f\n', max_case, attack_descriptions{max_case}, max_error);
fprintf('Least Severe Attack: Case %d (%s) - Average Error: %.4f\n', min_case, attack_descriptions{min_case}, min_error);

% Vehicle resilience ranking
vehicle_resilience = squeeze(mean(mean(mean_errors(non_attacker_ids, :, :), 2), 3));
[~, resilience_rank] = sort(vehicle_resilience);
fprintf('\nVehicle Resilience Ranking (most to least resilient):\n');
for i = 1:length(resilience_rank)
    v_id = non_attacker_ids(resilience_rank(i));
    fprintf('  %d. V%d (Average Error: %.4f)\n', i, v_id, vehicle_resilience(resilience_rank(i)));
end

fprintf('\nResults saved to: %s\n', excel_filename);
fprintf('Use these figures and statistics in your paper!\n');