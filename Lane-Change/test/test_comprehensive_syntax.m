%% QUICK TEST FOR COMPREHENSIVE ATTACK ANALYSIS
%% This script tests just one attack type to verify the syntax is correct

clear; clc; close all;

% Load configuration
Config

%% Test with just one attack type to verify syntax
attack_types_to_test = ["Bogus"];  % Just test Bogus for now
attacker_vehicles = [2];  % Just test one attacker
data_types = ["local"];   % Just test local

% Attack case ranges
attack_case_ranges = containers.Map();
attack_case_ranges("Bogus") = [1, 3];  % Just test first 3 cases

% Results storage
all_results = [];

fprintf('Testing syntax with limited scope...\n');

for attack_type = attack_types_to_test
    for attacker_id = attacker_vehicles
        for data_type = data_types
            
            case_range = attack_case_ranges(attack_type);
            
            fprintf('Testing %s attacks by V%d (%s data)\n', ...
                attack_type, attacker_id, data_type);
            
            % Initialize results
            config_results = struct();
            config_results.attack_type = attack_type;
            config_results.attacker_id = attacker_id;
            config_results.data_type = data_type;
            config_results.cases = [];
            config_results.errors = [];
            config_results.trust_metrics = [];
            
            % Test just first few cases
            for case_nb = case_range(1):min(case_range(2), 2)  % Just test first 2 cases
                fprintf('  Testing Case %d\n', case_nb);
                
                try
                    % Set up attack
                    attack_module = Attack_module(Scenarios_config.dt);
                    attack_module = Atk_Scenarios(attack_module, attack_type, data_type, case_nb, 10, 15, attacker_id, -1);
                    center_communication = CenterCommunication(attack_module);
                    
                    % Create vehicles (simplified)
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
                    
                    % Run short simulation
                    simulator = Simulator(straightLanes, [], platoon_vehicles, Scenarios_config.dt, false);
                    [state_log, input_log] = simulator.startSimulation(15, 10, 15, attacker_id);  % Short simulation
                    
                    % Calculate basic metrics
                    case_errors = [];
                    case_trust = [];
                    
                    all_vehicle_ids = [1, 2, 3, 4];
                    non_attacker_ids = all_vehicle_ids(all_vehicle_ids ~= attacker_id);
                    
                    for v_idx = non_attacker_ids
                        v = platoon_vehicles(v_idx);
                        [dist_err, theta_err, vel_err, acc_err] = v.observer.calculate_global_errors();
                        case_errors(end+1, :) = [mean(dist_err), mean(theta_err), mean(vel_err), mean(acc_err)];
                        case_trust(end+1) = 0.5;  % Dummy trust value
                    end
                    
                    % Store results
                    config_results.cases(end+1) = case_nb;
                    config_results.errors{end+1} = case_errors;
                    config_results.trust_metrics{end+1} = case_trust;
                    
                    fprintf('    Case %d completed successfully\n', case_nb);
                    
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

%% Test the plotting section that had syntax errors
fprintf('\nTesting plotting syntax...\n');

try
    % Test Plot 3: Data Type Impact (the section that had errors)
    fprintf('Testing unique attack types extraction...\n');
    
    % Get unique attack types (fixed version)
    unique_attack_types = {};
    for i = 1:length(all_results)
        if ~ismember(all_results{i}.attack_type, unique_attack_types)
            unique_attack_types{end+1} = all_results{i}.attack_type;
        end
    end
    
    fprintf('Found %d unique attack types: %s\n', length(unique_attack_types), strjoin(unique_attack_types, ', '));
    
    % Test vulnerability matrix section
    fprintf('Testing vulnerability matrix creation...\n');
    
    attack_type_list = {};
    for i = 1:length(all_results)
        if ~ismember(all_results{i}.attack_type, attack_type_list)
            attack_type_list{end+1} = all_results{i}.attack_type;
        end
    end
    
    fprintf('Attack type list for vulnerability matrix: %s\n', strjoin(attack_type_list, ', '));
    
    fprintf('Syntax test completed successfully!\n');
    
catch ME
    fprintf('Syntax test failed: %s\n', ME.message);
    fprintf('Error location: %s\n', ME.stack(1).name);
end

fprintf('\nQuick test completed. The comprehensive analysis should now work!\n');