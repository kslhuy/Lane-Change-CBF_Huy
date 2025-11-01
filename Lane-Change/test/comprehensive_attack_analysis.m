%% COMPREHENSIVE ATTACK ANALYSIS FOR PAPER
%% This script tests all major attack types and generates paper-ready plots
%% Author: Your Name
%% Purpose: Complete analysis of V2V cyber-attacks on distributed estimation

clear; clc; close all;

% Load configuration
Config

%% ===================================================================
%% EXPERIMENT CONFIGURATION
%% ===================================================================

% Test configuration
attack_types_to_test = ["Bogus"];
attacker_vehicles = [1];  % Test different attackers
data_types = ["local"];

% Attack case ranges for each type
attack_case_ranges = containers.Map();
attack_case_ranges("Bogus") = [1, 10];
attack_case_ranges("POS") = [1, 7];
attack_case_ranges("VEL") = [1, 9];
attack_case_ranges("ACC") = [1, 9];
attack_case_ranges("DoS") = [1, 1];  % DoS only has one case

% Results storage
all_results = [];
comparison_data = [];

%% ===================================================================
%% MAIN EXPERIMENT LOOP
%% ===================================================================

total_experiments = 0;
for attack_type = attack_types_to_test
    range = attack_case_ranges(attack_type);
    total_experiments = total_experiments + (range(2) - range(1) + 1) * length(attacker_vehicles) * length(data_types);
end

experiment_count = 0;
fprintf('Starting comprehensive attack analysis...\n');
fprintf('Total experiments to run: %d\n\n', total_experiments);

for attack_type = attack_types_to_test
    for attacker_id = attacker_vehicles
        for data_type = data_types
            
            % Get attack case range for this attack type
            case_range = attack_case_ranges(attack_type);
            
            fprintf('=== Testing %s attacks by V%d (%s data) ===\n', ...
                attack_type, attacker_id, data_type);
            
            % Configure attack parameters
            t_star = 10;
            t_end = 15;
            victim_id = -1;  % All vehicles are victims
            
            % Initialize results for this configuration
            config_results = struct();
            config_results.attack_type = attack_type;
            config_results.attacker_id = attacker_id;
            config_results.data_type = data_type;
            config_results.cases = [];
            config_results.errors = [];
            config_results.trust_metrics = [];
            
            % Test all cases for this attack type
            for case_nb = case_range(1):case_range(2)
                experiment_count = experiment_count + 1;
                fprintf('  Case %d/%d (Progress: %d/%d)\n', case_nb, case_range(2), experiment_count, total_experiments);
                
                try
                    % Set up attack
                    attack_module = Attack_module(Scenarios_config.dt);
                    attack_module = Atk_Scenarios(attack_module, attack_type, data_type, case_nb, t_star, t_end, attacker_id, victim_id);
                    center_communication = CenterCommunication(attack_module);
                    
                    % Create vehicles
                    initial_lane_id = 1;
                    direction_flag = 0;
                    
                    car1 = Vehicle(1, "None", param_sys, [80; 0.5 * lane_width; 0; 23; 0], initial_lane_id, straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);
                    car2 = Vehicle(2, "IDM", param_sys, [60; 0.5 * lane_width; 0; 26; 0], initial_lane_id, straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);
                    car3 = Vehicle(3, "IDM", param_sys, [40; 0.5 * lane_width; 0; 26; 0], initial_lane_id, straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);
                    car4 = Vehicle(4, "IDM", param_sys, [20; 0.5 * lane_width; 0; 26; 0], initial_lane_id, straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);
                    
                    platoon_vehicles = [car1; car2; car3; car4];
                    
                    % Assign neighbors
                    car1.assign_neighbor_vehicle(platoon_vehicles, [], "None", center_communication, graph);
                    car2.assign_neighbor_vehicle(platoon_vehicles, [], "CACC", center_communication, graph);
                    car3.assign_neighbor_vehicle(platoon_vehicles, [], "CACC", center_communication, graph);
                    car4.assign_neighbor_vehicle(platoon_vehicles, [], "CACC", center_communication, graph);
                    
                    % Run simulation
                    simulator = Simulator(straightLanes, [], platoon_vehicles, Scenarios_config.dt, false);
                    [state_log, input_log] = simulator.startSimulation(Scenarios_config.simulation_time, t_star, t_end, attacker_id);
                    
                    % Calculate metrics
                    case_errors = [];
                    case_trust = [];
                    
                    all_vehicle_ids = [1, 2, 3, 4];
                    non_attacker_ids = all_vehicle_ids(all_vehicle_ids ~= attacker_id);
                    
                    for v_idx = non_attacker_ids
                        v = platoon_vehicles(v_idx);
                        [dist_err, theta_err, vel_err, acc_err] = v.observer.calculate_global_errors();
                        
                        case_errors(end+1, :) = [mean(dist_err), mean(theta_err), mean(vel_err), mean(acc_err)];
                        
                        % Trust metrics (if available)
                        try
                            if isfield(v, 'trust_model') && ~isempty(v.trust_model)
                                trust_scores = v.trust_model.trust_sample_log;
                                if ~isempty(trust_scores) && size(trust_scores, 2) >= attacker_id
                                    case_trust(end+1) = mean(trust_scores(:, attacker_id));
                                else
                                    case_trust(end+1) = NaN;
                                end
                            else
                                case_trust(end+1) = NaN;
                            end
                        catch
                            case_trust(end+1) = NaN;
                        end
                    end
                    
                    % Store results
                    config_results.cases(end+1) = case_nb;
                    config_results.errors{end+1} = case_errors;
                    config_results.trust_metrics{end+1} = case_trust;
                    
                catch ME
                    fprintf('    Error in case %d: %s\n', case_nb, ME.message);
                    continue;
                end
            end
            
            % Store configuration results
            all_results{end+1} = config_results;
        end
    end
end

%% ===================================================================
%% GENERATE COMPREHENSIVE PAPER PLOTS
%% ===================================================================

fprintf('\nGenerating comprehensive analysis plots...\n');

%% Plot 1: Attack Type Comparison
figure('Position', [100, 100, 1400, 800]);

attack_severity_summary = [];
attack_labels = {};

for i = 1:length(all_results)
    result = all_results{i};
    if ~isempty(result.errors)
        % Calculate average error across all cases and vehicles
        all_case_errors = [];
        for j = 1:length(result.errors)
            all_case_errors = [all_case_errors; result.errors{j}];
        end
        avg_error = mean(mean(all_case_errors, 1));
        attack_severity_summary(end+1) = avg_error;
        attack_labels{end+1} = sprintf('%s-V%d-%s', result.attack_type, result.attacker_id, result.data_type);
    end
end

subplot(2,2,1);
bar(attack_severity_summary);
xticklabels(attack_labels);
xtickangle(45);
ylabel('Average Estimation Error');
title('(a) Attack Severity Comparison');
grid on;

%% Plot 2: Trust Score Analysis
subplot(2,2,2);
trust_degradation_summary = [];
trust_labels = {};

for i = 1:length(all_results)
    result = all_results{i};
    if ~isempty(result.trust_metrics)
        all_trust = [];
        for j = 1:length(result.trust_metrics)
            all_trust = [all_trust; result.trust_metrics{j}];
        end
        if ~isempty(all_trust) && ~all(isnan(all_trust(:)))
            avg_trust = mean(all_trust(~isnan(all_trust)));
            trust_degradation_summary(end+1) = 1 - avg_trust;  % Trust degradation
            trust_labels{end+1} = sprintf('%s-V%d', result.attack_type, result.attacker_id);
        end
    end
end

if ~isempty(trust_degradation_summary)
    bar(trust_degradation_summary);
    xticklabels(trust_labels);
    xtickangle(45);
    ylabel('Trust Degradation');
    title('(b) Trust System Response');
    grid on;
else
    text(0.5, 0.5, 'No Trust Data Available', 'HorizontalAlignment', 'center');
    title('(b) Trust System Response - No Data');
end

%% Plot 3: Data Type Impact (Local vs Global)
subplot(2,2,3);
local_errors = [];
global_errors = [];
comparison_labels = {};

% Get unique attack types
unique_attack_types = {};
for i = 1:length(all_results)
    if ~ismember(all_results{i}.attack_type, unique_attack_types)
        unique_attack_types{end+1} = all_results{i}.attack_type;
    end
end

for attack_type_idx = 1:length(unique_attack_types)
    attack_type = unique_attack_types{attack_type_idx};
    local_avg = [];
    global_avg = [];
    
    for i = 1:length(all_results)
        result = all_results{i};
        if strcmp(result.attack_type, attack_type) && ~isempty(result.errors)
            all_errors = [];
            for j = 1:length(result.errors)
                all_errors = [all_errors; result.errors{j}];
            end
            avg_error = mean(mean(all_errors, 1));
            
            if strcmp(result.data_type, "local")
                local_avg(end+1) = avg_error;
            elseif strcmp(result.data_type, "global")
                global_avg(end+1) = avg_error;
            end
        end
    end
    
    if ~isempty(local_avg) && ~isempty(global_avg)
        local_errors(end+1) = mean(local_avg);
        global_errors(end+1) = mean(global_avg);
        comparison_labels{end+1} = attack_type;
    end
end

if ~isempty(local_errors)
    x = 1:length(comparison_labels);
    bar(x-0.2, local_errors, 0.4, 'DisplayName', 'Local Attack');
    hold on;
    bar(x+0.2, global_errors, 0.4, 'DisplayName', 'Global Attack');
    xticklabels(comparison_labels);
    ylabel('Average Estimation Error');
    title('(c) Local vs Global Attack Impact');
    legend('Location', 'best');
    grid on;
    hold off;
else
    text(0.5, 0.5, 'Insufficient Data for Comparison', 'HorizontalAlignment', 'center');
    title('(c) Local vs Global Attack Impact - Insufficient Data');
end

%% Plot 4: Vehicle Vulnerability Matrix
subplot(2,2,4);
vulnerability_matrix = [];
vehicle_ids = [1, 2, 3, 4];

% Get unique attack types for vulnerability matrix
attack_type_list = {};
for i = 1:length(all_results)
    if ~ismember(all_results{i}.attack_type, attack_type_list)
        attack_type_list{end+1} = all_results{i}.attack_type;
    end
end

for v_id = vehicle_ids
    v_vulnerabilities = [];
    for attack_type_idx = 1:length(attack_type_list)
        attack_type = attack_type_list{attack_type_idx};
        v_error_sum = 0;
        count = 0;
        
        for i = 1:length(all_results)
            result = all_results{i};
            if strcmp(result.attack_type, attack_type) && result.attacker_id ~= v_id && ~isempty(result.errors)
                for j = 1:length(result.errors)
                    errors = result.errors{j};
                    v_row = find([1,2,3,4] ~= result.attacker_id);
                    if length(v_row) >= find(vehicle_ids == v_id)
                        target_row = find(v_row == v_id);
                        if ~isempty(target_row) && target_row <= size(errors, 1)
                            v_error_sum = v_error_sum + mean(errors(target_row, :));
                            count = count + 1;
                        end
                    end
                end
            end
        end
        
        if count > 0
            v_vulnerabilities(end+1) = v_error_sum / count;
        else
            v_vulnerabilities(end+1) = 0;
        end
    end
    vulnerability_matrix(end+1, :) = v_vulnerabilities;
end

if ~isempty(vulnerability_matrix)
    imagesc(vulnerability_matrix);
    colorbar;
    xlabel('Attack Types');
    ylabel('Vehicles');
    title('(d) Vehicle Vulnerability Matrix');
    xticklabels(attack_type_list);
    yticklabels(cellstr("V" + string(vehicle_ids)));
else
    text(0.5, 0.5, 'No Vulnerability Data', 'HorizontalAlignment', 'center');
    title('(d) Vehicle Vulnerability Matrix - No Data');
end

sgtitle('Comprehensive V2V Cyber-Attack Analysis', 'FontSize', 16, 'FontWeight', 'bold');

%% Save results to Excel
fprintf('Saving comprehensive results to Excel...\n');
excel_filename = sprintf('Comprehensive_Attack_Analysis_%s.xlsx', datestr(now, 'yyyy-mm-dd_HH-MM'));

% Create summary sheet
summary_data = {};
summary_headers = {'Attack_Type', 'Attacker_ID', 'Data_Type', 'Avg_Distance_Error', 'Avg_Velocity_Error', 'Avg_Trust_Degradation', 'Num_Cases_Tested'};

row = 1;
for i = 1:length(all_results)
    result = all_results{i};
    if ~isempty(result.errors)
        all_errors = [];
        for j = 1:length(result.errors)
            all_errors = [all_errors; result.errors{j}];
        end
        
        avg_dist = mean(all_errors(:, 1));
        avg_vel = mean(all_errors(:, 3));
        
        avg_trust_deg = NaN;
        if ~isempty(result.trust_metrics)
            all_trust = [];
            for j = 1:length(result.trust_metrics)
                all_trust = [all_trust; result.trust_metrics{j}];
            end
            if ~all(isnan(all_trust(:)))
                avg_trust_deg = 1 - mean(all_trust(~isnan(all_trust)));
            end
        end
        
        summary_data{row, 1} = result.attack_type;
        summary_data{row, 2} = result.attacker_id;
        summary_data{row, 3} = result.data_type;
        summary_data{row, 4} = avg_dist;
        summary_data{row, 5} = avg_vel;
        summary_data{row, 6} = avg_trust_deg;
        summary_data{row, 7} = length(result.cases);
        row = row + 1;
    end
end

% Write to Excel
writecell([summary_headers; summary_data], excel_filename, 'Sheet', 'Summary');

fprintf('\n=== COMPREHENSIVE ANALYSIS COMPLETE ===\n');
fprintf('Results saved to: %s\n', excel_filename);
fprintf('Total experiments completed: %d\n', experiment_count);
fprintf('Use the generated plots and Excel data for your paper!\n');

%% Display key findings
fprintf('\n=== KEY FINDINGS FOR PAPER ===\n');
if ~isempty(attack_severity_summary)
    [max_severity, max_idx] = max(attack_severity_summary);
    [min_severity, min_idx] = min(attack_severity_summary);
    fprintf('Most severe attack configuration: %s (Error: %.4f)\n', attack_labels{max_idx}, max_severity);
    fprintf('Least severe attack configuration: %s (Error: %.4f)\n', attack_labels{min_idx}, min_severity);
end

if ~isempty(trust_degradation_summary)
    fprintf('Average trust degradation across all attacks: %.3f\n', mean(trust_degradation_summary));
    fprintf('Maximum trust degradation: %.3f\n', max(trust_degradation_summary));
end