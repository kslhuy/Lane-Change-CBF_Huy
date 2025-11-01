function save_config_csv()
    % SAVE_CONFIG_CSV Simple function to save simulation configuration to CSV
    % Appends each run to the same file
    
    % Use fixed filename - all runs will be saved here
    filename = 'simulation_configs.csv';
    
    % Create timestamp for this run
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    
    % Create data structure to hold config info
    config_data = {};
    config_headers = {'Run_ID', 'Parameter', 'Value', 'Type', 'Timestamp'};
    
    % List of workspace variables to save (not Scenarios_config properties)
    workspace_vars = {'dt', 'simulation_time', 'Road_type', 'trust_threshold', 'kappa', ...
                     't_star', 't_end', 'attacker_vehicle_id', 'victim_id', 'case_nb_attack', ...
                     'data_type_attack', 'attack_type', 'lane_width', 'num_lanes', 'max_length', ...
                     'IsShowAnimation', 'debug_mode', 'initial_lane_id', 'direction_flag'};
    
    % List of Scenarios_config properties to save
    scenarios_config_props = {'lead_senario', 'Use_predict_observer', 'predict_controller_type', ...
                             'Local_observer_type', 'Is_noise_mesurement', 'noise_probability', ...
                             'use_local_data_from_other', 'attacker_update_locally', ...
                             'MAX_PREDICT_ONLY_TIME', 'N_good', 'blend_thresh', 'control_use_accel', ...
                             'gamma_type', 'controller_type', 'CACC_bidirectional', 'data_type_for_u2', ...
                             'using_weight_trust_observer', 'Use_weight_local_trust', 'Use_weight_global_trust', ...
                             'opinion_type', 'Dichiret_type', 'Monitor_sudden_change', ...
                             'is_know_data_not_nearby', 'model_vehicle_type', 'debug_mode', 'where'};
    
    % Generate unique run ID
    if exist(filename, 'file')
        existing_data = readcell(filename);
        if size(existing_data, 1) > 1  % Has data beyond header
            % Get last run ID and increment - handle mixed data types safely
            run_id_column = existing_data(2:end, 1);  % Get all run IDs
            numeric_run_ids = [];
            
            for i = 1:length(run_id_column)
                if isnumeric(run_id_column{i}) && ~isempty(run_id_column{i})
                    numeric_run_ids = [numeric_run_ids, run_id_column{i}];
                end
            end
            
            if ~isempty(numeric_run_ids)
                run_id = max(numeric_run_ids) + 1;
            else
                run_id = 1;
            end
        else
            run_id = 1;
        end
    else
        run_id = 1;
    end
    
    % Helper function to convert values to string
    function value_str = convert_to_string(var_value)
        if isnumeric(var_value)
            if length(var_value) == 1
                value_str = num2str(var_value);
            else
                value_str = ['[' num2str(var_value) ']'];
            end
        elseif islogical(var_value)
            value_str = char(string(var_value));
        elseif isstring(var_value) || ischar(var_value)
            value_str = char(var_value);
        else
            value_str = 'Complex_Object';
        end
    end
    
    % Collect workspace variables
    for i = 1:length(workspace_vars)
        var_name = workspace_vars{i};
        try
            var_value = evalin('base', var_name);
            var_type = class(var_value);
            value_str = convert_to_string(var_value);
            
            config_data{end+1, 1} = run_id;
            config_data{end, 2} = var_name;
            config_data{end, 3} = value_str;
            config_data{end, 4} = var_type;
            config_data{end, 5} = timestamp;
        catch
            config_data{end+1, 1} = run_id;
            config_data{end, 2} = var_name;
            config_data{end, 3} = 'N/A';
            config_data{end, 4} = 'N/A';
            config_data{end, 5} = timestamp;
        end
    end
    
    % Collect Scenarios_config properties
    try
        scenarios_config = evalin('base', 'Scenarios_config');
        for i = 1:length(scenarios_config_props)
            prop_name = scenarios_config_props{i};
            try
                var_value = scenarios_config.(prop_name);
                var_type = class(var_value);
                value_str = convert_to_string(var_value);
                
                config_data{end+1, 1} = run_id;
                config_data{end, 2} = ['Scenarios_config.' prop_name];
                config_data{end, 3} = value_str;
                config_data{end, 4} = var_type;
                config_data{end, 5} = timestamp;
            catch
                config_data{end+1, 1} = run_id;
                config_data{end, 2} = ['Scenarios_config.' prop_name];
                config_data{end, 3} = 'N/A';
                config_data{end, 4} = 'N/A';
                config_data{end, 5} = timestamp;
            end
        end
    catch
        % Scenarios_config object doesn't exist
        config_data{end+1, 1} = run_id;
        config_data{end, 2} = 'Scenarios_config';
        config_data{end, 3} = 'Object_Not_Found';
        config_data{end, 4} = 'N/A';
        config_data{end, 5} = timestamp;
    end
    
    % Add graph matrix as a special case
    try
        graph = evalin('base', 'graph');
        if ~isempty(graph)
            graph_str = '';
            for row = 1:size(graph, 1)
                graph_str = [graph_str '[' num2str(graph(row, :)) ']'];
                if row < size(graph, 1)
                    graph_str = [graph_str '; '];
                end
            end
            config_data{end+1, 1} = run_id;
            config_data{end, 2} = 'graph_matrix';
            config_data{end, 3} = graph_str;
            config_data{end, 4} = 'matrix';
            config_data{end, 5} = timestamp;
        end
    catch
        % Skip if graph doesn't exist
    end
    
    % Check if file exists and append or create new
    if exist(filename, 'file')
        % File exists, append new data with blank line separator
        existing_data = readcell(filename);
        
        % Add blank line to separate runs
        blank_line = cell(1, size(config_headers, 2));
        for j = 1:length(blank_line)
            blank_line{j} = '';
        end
        
        all_data = [existing_data; blank_line; config_data];
        writecell(all_data, filename);
        fprintf('Configuration appended to: %s (Run ID: %d)\n', filename, run_id);
    else
        % File doesn't exist, create new with headers
        all_data = [config_headers; config_data];
        writecell(all_data, filename);
        fprintf('New configuration file created: %s (Run ID: %d)\n', filename, run_id);
    end
    
    fprintf('Parameters saved for this run: %d\n', size(config_data, 1));
end