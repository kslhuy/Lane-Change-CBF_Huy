function save_results_csv(platton_vehicles, attack_module, Scenarios_config)
    % SAVE_RESULTS_CSV Quick function to save simulation results to CSV files
    %
    % Inputs:
    %   platton_vehicles - Array of vehicle objects
    %   attack_module    - Attack module object
    %   Scenarios_config - Configuration parameters
    
    % Create timestamp for unique file naming
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    
    fprintf('Saving CSV results with timestamp: %s\n', timestamp);
    
    %% 1. Save Trust Logs
    num_vehicles = length(platton_vehicles);
    
    % Collect all trust data
    trust_data = [];
    trust_headers = {'Time_Step', 'Observer_Vehicle', 'Target_Vehicle', 'Trust_Value'};
    
    for observer = 1:num_vehicles
        if ~isempty(platton_vehicles(observer).trust_log)
            time_steps = size(platton_vehicles(observer).trust_log, 2);
            for target = 1:num_vehicles
                for t = 1:time_steps
                    trust_value = platton_vehicles(observer).trust_log(1, t, target);
                    trust_data = [trust_data; t, observer, target, trust_value];
                end
            end
        end
    end
    
    % Save trust data
    if ~isempty(trust_data)
        filename = sprintf('trust_log_%s.csv', timestamp);
        writecell([trust_headers; num2cell(trust_data)], filename);
        fprintf('Saved: %s\n', filename);
    end
    
    %% 2. Save Global Estimation Errors
    error_data = [];
    error_headers = {'Time_Step', 'Vehicle_ID', 'Position_X_Error', 'Position_Y_Error', ...
                     'Theta_Error', 'Velocity_Error', 'Acceleration_Error'};
    
    for v = 1:num_vehicles
        vehicle = platton_vehicles(v);
        if ~isempty(vehicle.observer) && ~isempty(vehicle.observer.est_global_state_log)
            time_steps = size(vehicle.observer.est_global_state_log, 2) - 1;
            state_steps = size(vehicle.state_log, 2);
            min_steps = min(time_steps, state_steps);
            
            for t = 1:min_steps
                % Calculate errors for each state
                est_states = vehicle.observer.est_global_state_log(:, t, v);
                true_states = vehicle.state_log(:, t);
                
                errors = est_states - true_states;
                
                % Ensure we have 5 states (pad with NaN if necessary)
                if length(errors) < 5
                    errors = [errors; NaN(5 - length(errors), 1)];
                end
                
                error_data = [error_data; t, v, errors(1), errors(2), errors(3), errors(4), errors(5)];
            end
        end
    end
    
    % Save error data
    if ~isempty(error_data)
        filename = sprintf('global_errors_%s.csv', timestamp);
        writecell([error_headers; num2cell(error_data)], filename);
        fprintf('Saved: %s\n', filename);
    end
    
    %% 3. Save Local Estimation Errors
    local_error_data = [];
    local_error_headers = {'Time_Step', 'Vehicle_ID', 'Local_X_Error', 'Local_Y_Error', ...
                          'Local_Theta_Error', 'Local_Velocity_Error', 'Local_Acceleration_Error'};
    
    for v = 1:num_vehicles
        vehicle = platton_vehicles(v);
        if ~isempty(vehicle.observer) && ~isempty(vehicle.observer.est_local_state_log)
            time_steps = size(vehicle.observer.est_local_state_log, 2) - 1;
            state_steps = size(vehicle.state_log, 2);
            min_steps = min(time_steps, state_steps);
            
            for t = 1:min_steps
                % Calculate local estimation errors
                est_local = vehicle.observer.est_local_state_log(:, t);
                true_states = vehicle.state_log(:, t);
                
                local_errors = est_local - true_states;
                
                % Ensure we have 5 states
                if length(local_errors) < 5
                    local_errors = [local_errors; NaN(5 - length(local_errors), 1)];
                end
                
                local_error_data = [local_error_data; t, v, local_errors(1), local_errors(2), ...
                                   local_errors(3), local_errors(4), local_errors(5)];
            end
        end
    end
    
    % Save local error data
    if ~isempty(local_error_data)
        filename = sprintf('local_errors_%s.csv', timestamp);
        writecell([local_error_headers; num2cell(local_error_data)], filename);
        fprintf('Saved: %s\n', filename);
    end
    
    %% 4. Save Attack Scenario Information
    if ~isempty(attack_module.scenario)
        attack_info = [];
        attack_headers = {'Scenario_ID', 'Attack_Type', 'Data_Type', 'Attacker_ID', ...
                         'Victim_ID', 'Start_Time', 'End_Time', 'Fault_Intensity', 'Attack_Row'};
        
        for i = 1:length(attack_module.scenario)
            scenario = attack_module.scenario(i);
            attack_row_str = '';
            if iscell(scenario.attack_row)
                attack_row_str = strjoin(scenario.attack_row, ';');
            else
                attack_row_str = char(scenario.attack_row);
            end
            
            attack_info = [attack_info; i, string(scenario.attack_type), string(scenario.data_type), ...
                          scenario.attacker_id, scenario.victim_id, scenario.start_time, ...
                          scenario.end_time, scenario.fault_intensity, string(attack_row_str)];
        end
        
        filename = sprintf('attack_scenarios_%s.csv', timestamp);
        writecell([attack_headers; num2cell(attack_info)], filename);
        fprintf('Saved: %s\n', filename);
    end
    
    %% 5. Save Trip Model Trust Details (for vehicles that have them)
    trip_trust_data = [];
    trip_headers = {'Time_Step', 'Host_Vehicle', 'Target_Vehicle', 'Local_Trust', ...
                   'Gamma_Cross', 'Gamma_Local', 'V_Score', 'D_Score', 'A_Score', 'Final_Score'};
    
    for v = 1:num_vehicles
        vehicle = platton_vehicles(v);
        if ~isempty(vehicle.trip_models)
            for tm = 1:length(vehicle.trip_models)
                if ~isempty(vehicle.trip_models{tm})
                    trip_model = vehicle.trip_models{tm};
                    
                    % Get the minimum length across all logs
                    log_lengths = [length(trip_model.trust_sample_log), ...
                                  length(trip_model.gamma_cross_log), ...
                                  length(trip_model.gamma_local_log), ...
                                  length(trip_model.v_score_log), ...
                                  length(trip_model.d_score_log), ...
                                  length(trip_model.a_score_log), ...
                                  length(trip_model.final_score_log)];
                    
                    min_length = min(log_lengths(log_lengths > 0));
                    
                    if min_length > 0
                        for t = 1:min_length
                            local_trust = get_log_value(trip_model.trust_sample_log, t);
                            gamma_cross = get_log_value(trip_model.gamma_cross_log, t);
                            gamma_local = get_log_value(trip_model.gamma_local_log, t);
                            v_score = get_log_value(trip_model.v_score_log, t);
                            d_score = get_log_value(trip_model.d_score_log, t);
                            a_score = get_log_value(trip_model.a_score_log, t);
                            final_score = get_log_value(trip_model.final_score_log, t);
                            
                            trip_trust_data = [trip_trust_data; t, v, tm, local_trust, ...
                                             gamma_cross, gamma_local, v_score, d_score, a_score, final_score];
                        end
                    end
                end
            end
        end
    end
    
    % Save trip model trust data
    if ~isempty(trip_trust_data)
        filename = sprintf('trip_model_trust_%s.csv', timestamp);
        writecell([trip_headers; num2cell(trip_trust_data)], filename);
        fprintf('Saved: %s\n', filename);
    end
    
    %% 6. Save Summary Statistics
    summary_data = [];
    summary_headers = {'Vehicle_ID', 'Controller_Type', 'Mean_Trust_To_Others', ...
                      'Mean_Global_Position_Error', 'Mean_Global_Velocity_Error', ...
                      'Mean_Local_Position_Error', 'Mean_Local_Velocity_Error'};
    
    for v = 1:num_vehicles
        vehicle = platton_vehicles(v);
        
        % Mean trust to others
        mean_trust = NaN;
        if ~isempty(vehicle.trust_log)
            other_vehicles = setdiff(1:num_vehicles, v);
            trust_values = squeeze(vehicle.trust_log(1, :, other_vehicles));
            if ~isempty(trust_values)
                mean_trust = mean(trust_values(:), 'omitnan');
            end
        end
        
        % Mean global errors
        mean_global_pos_err = NaN;
        mean_global_vel_err = NaN;
        if ~isempty(vehicle.observer) && ~isempty(vehicle.observer.est_global_state_log)
            try
                [~, ~, vel_err, ~] = vehicle.observer.calculate_global_errors();
                pos_err = sqrt(sum(vehicle.observer.est_global_state_log(1:2, 1:end-1, v) - vehicle.state_log(1:2, 1:end-1)).^2, 1);
                mean_global_pos_err = mean(pos_err, 'omitnan');
                mean_global_vel_err = mean(vel_err, 'omitnan');
            catch
                % Skip if error calculation fails
            end
        end
        
        % Mean local errors  
        mean_local_pos_err = NaN;
        mean_local_vel_err = NaN;
        if ~isempty(vehicle.observer) && ~isempty(vehicle.observer.est_local_state_log)
            try
                local_pos_err = sqrt(sum((vehicle.observer.est_local_state_log(1:2, 1:end-1) - vehicle.state_log(1:2, 1:end-1)).^2, 1));
                local_vel_err = abs(vehicle.observer.est_local_state_log(4, 1:end-1) - vehicle.state_log(4, 1:end-1));
                mean_local_pos_err = mean(local_pos_err, 'omitnan');
                mean_local_vel_err = mean(local_vel_err, 'omitnan');
            catch
                % Skip if error calculation fails
            end
        end
        
        summary_data = [summary_data; v, string(vehicle.controller_type), mean_trust, ...
                       mean_global_pos_err, mean_global_vel_err, mean_local_pos_err, mean_local_vel_err];
    end
    
    % Save summary data
    filename = sprintf('summary_%s.csv', timestamp);
    writecell([summary_headers; num2cell(summary_data)], filename);
    fprintf('Saved: %s\n', filename);
    
    fprintf('CSV export completed!\n');
end

function value = get_log_value(log_array, index)
    % Helper function to safely get a value from a log array
    if isempty(log_array) || index > length(log_array)
        value = NaN;
    else
        value = log_array(index);
    end
end