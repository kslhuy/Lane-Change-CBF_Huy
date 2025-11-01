classdef Attack_module < handle
    % ATTACK_MODULE Class to simulate cyber-attacks on vehicle communication data.

    properties
        network_jitter;      % Maximum network jitter in seconds
        packet_loss_rate;    % Probability of packet loss (0 to 1)
        scenario;            % Struct with attack scenario details
        dt;                  % Time step for simulation
        attack_values_global;
        attack_values_local;
    end

    methods
        % Constructor with default values
        function self = Attack_module(dt)
            % Initialize default values for network jitter and packet loss rate
            self.network_jitter = 0.05;    % 50ms jitter
            self.packet_loss_rate = 0.1;   % 10% packet loss rate
            self.dt = dt;
            self.scenario = struct('type', {},'attacker_id',{} , 'victim_id',{} , 'start_time', {}, 'end_time', {}, ...
                'attack_type', {}, 'fault_intensity', {},'data_type',{} ,'attack_row',{}); % Empty array

            self.attack_values_global = struct('vehicle_id', {}, 'timestamp', {}, ...
                'attack_type', {}, 'attack_rows', {}, 'perturbation', {});

            self.attack_values_local = struct('vehicle_id', {}, 'timestamp', {}, ...
                'attack_type', {}, 'attack_rows', {}, 'perturbation', {});
        end

        function setScenario(self, scenario_type, scenario_params)
            % SETSCENARIO Appends a new attack scenario.
            switch scenario_type
                case 'time_based'
                    required_fields = {'attacker_id' ,'victim_id','start_time', 'end_time', 'attack_type', 'fault_intensity','data_type' , 'attack_row'};
                    for i = 1:length(required_fields)
                        if ~isfield(scenario_params, required_fields{i})
                            error('Missing required field: %s', required_fields{i});
                        end
                    end
                    % Append new scenario to the array
                    new_scenario = struct('type', 'time_based', ...
                        'attacker_id', scenario_params.attacker_id, ...
                        'victim_id', scenario_params.victim_id, ...
                        'start_time', scenario_params.start_time, ...
                        'end_time', scenario_params.end_time, ...
                        'attack_type', scenario_params.attack_type, ...
                        'fault_intensity', scenario_params.fault_intensity,...
                        'data_type',scenario_params.data_type,...
                        'attack_row',scenario_params.attack_row);
                    self.scenario(end+1) = new_scenario;
                case 'none'
                    % Clear all scenarios
                    self.scenario = struct('type', {}, 'attacker_id', {}, 'victim_id', {}, ...
                        'start_time', {}, 'end_time', {}, 'attack_type', {}, ...
                        'fault_intensity', {}, 'data_type', {}, 'attack_row', {});
                otherwise
                    error('Unknown scenario type. Use "time_based" or "none".');
            end
        end

        % Simulate cyber-attack on global data (similar changes apply)
        function [x_hat_i_j , original_data] = SetGlobalAttack(self, vehicle_id, x_hat_i_j, instant_idx)

            timestamp = instant_idx * self.dt; %convert to seconds
            original_data = x_hat_i_j; % Preserve original for layering attacks
            if isempty(self.scenario)
                % disp('No attacks to apply');
                return;
            end

            % % Store history for replay attacks with size limit
            % persistent data_history;
            % if isempty(data_history)
            %     data_history = containers.Map;
            % end
            % key = sprintf('%d_%s', vehicle_id, num2str(self.dt));
            % if ~isKey(data_history, key)
            %     data_history(key) = struct('time', [], 'data', []);
            % end
            % history = data_history(key);
            % history.time(end+1) = timestamp;
            % history.data(:, end+1) = x_hat_i_j;
            %
            % % Limit history to last 100 entries to avoid memory issues
            % max_history_size = 100;
            % if length(history.time) > max_history_size
            %     history.time = history.time(end-max_history_size+1:end);
            %     history.data = history.data(:, end-max_history_size+1:end);
            % end
            % data_history(key) = history;

            for i = 1:length(self.scenario)
                if (strcmp(self.scenario(i).type, 'time_based') && ...
                        (strcmp(self.scenario(i).data_type, 'global') || strcmp(self.scenario(i).data_type, 'both')) && ...
                        ismember(vehicle_id ,self.scenario(i).attacker_id) && ...
                        timestamp >= self.scenario(i).start_time && ...
                        timestamp <= self.scenario(i).end_time)

                    attack_type = self.scenario(i).attack_type;
                    fault_intensity = self.scenario(i).fault_intensity;

                    attack_rows = self.GetAttackRows_global(self.scenario(i).attack_row, size(x_hat_i_j, 1));

                    perturbation = zeros(size(x_hat_i_j));
                    switch attack_type
                        case 'faulty'
                            % Random faulty behavior - sometimes fault, sometimes normal
                            if isstruct(fault_intensity)
                                % New format: struct with 'intensity' and 'probability'
                                fault_prob = fault_intensity.probability;
                                fault_mag = fault_intensity.intensity;
                            else
                                % Backward compatibility: old format (simple number)
                                fault_prob = 0.3; % Default 30% probability
                                fault_mag = fault_intensity;
                            end
                            
                            if rand < fault_prob
                                % Apply random fault when probability triggers
                                perturbation(attack_rows) = randn(size(attack_rows)) * fault_mag;
                                x_hat_i_j(attack_rows) = x_hat_i_j(attack_rows) + perturbation(attack_rows);
                            else
                                % No fault this time step - behave normally
                                perturbation(attack_rows) = 0;
                            end
                        case 'scaling'
                            perturbation(attack_rows) = x_hat_i_j(attack_rows) * fault_intensity; % Added value
                            x_hat_i_j(attack_rows) = x_hat_i_j(attack_rows) * (1 + fault_intensity);
                        case 'bias'
                            perturbation(attack_rows) = fault_intensity; % Added value
                            x_hat_i_j(attack_rows) = x_hat_i_j(attack_rows) + fault_intensity;
                        case 'linear'
                            perturbation(attack_rows) = fault_intensity * (timestamp - self.scenario(i).start_time);
                            x_hat_i_j(attack_rows) = x_hat_i_j(attack_rows) + fault_intensity * (timestamp - self.scenario(i).start_time);
                        case 'sinusoidal'
                            amplitude = fault_intensity.amplitude;
                            frequency = fault_intensity.frequency;
                            perturbation(attack_rows) = amplitude * sin(2 * pi * frequency * (timestamp - self.scenario(i).start_time));
                            x_hat_i_j(attack_rows) = x_hat_i_j(attack_rows) + perturbation(attack_rows);
                        case 'replay'
                            % Replay data from `fault_intensity` seconds ago
                            delay = abs(fault_intensity);
                            idx = find(history.time <= timestamp - delay, 1, 'last');
                            if ~isempty(idx)
                                perturbation(attack_rows) = history.data(attack_rows, idx) - x_hat_i_j(attack_rows);
                                x_hat_i_j(attack_rows) = history.data(attack_rows, idx);
                            else
                                perturbation(attack_rows) = 0; % No change if no history
                            end
                        case 'coordinated'
                            % For acceleration-velocity coordination
                            a_bias = fault_intensity.a_bias;
                            if any(strcmp(self.scenario(i).attack_row, 'acceleration'))
                                acc_rows = attack_rows(strcmp(self.scenario(i).attack_row, 'acceleration'));
                                perturbation(acc_rows) = a_bias;
                                x_hat_i_j(acc_rows) = x_hat_i_j(acc_rows) + a_bias;
                            end
                            if any(strcmp(self.scenario(i).attack_row, 'velocity'))
                                vel_rows = attack_rows(strcmp(self.scenario(i).attack_row, 'velocity'));
                                perturbation(vel_rows) = a_bias * (timestamp - self.scenario(i).start_time);
                                x_hat_i_j(vel_rows) = x_hat_i_j(vel_rows) + perturbation(vel_rows);

                            end
                            % case 'drop'
                            %     x_hat_i_j(attack_rows) = NaN; % No data transmitted

                        case 'drop'
                            drop_chance = fault_intensity;  % Use fault_intensity as drop rate
                            if rand < drop_chance
                                perturbation = NaN; % No data transmitted
                                x_hat_i_j = NaN;
                            end
                        otherwise
                            warning('Unknown attack type in scenario %d: %s', i, attack_type);
                    end

                    % Store attack value
                    self.attack_values_global(end+1) = struct(...
                        'vehicle_id', vehicle_id, ...
                        'timestamp', timestamp, ...
                        'attack_type', attack_type, ...
                        'attack_rows', {attack_rows}, ...
                        'perturbation', perturbation);
                end
            end
        end

        % Simulate cyber-attack on local aggregated data
        function [x_bar_j,original_data] = SetLocalAttack(self,vehicle_id, x_bar_j, instant_idx)
            persistent data_history;


            timestamp = instant_idx * self.dt; %convert to seconds
            original_data = x_bar_j; % Preserve original for layering attacks
            % SETLOCALATTACK Simulates attacks based on all active scenarios.
            if isempty(self.scenario)
                % disp('No attacks to apply');
                return; % No attacks to apply
            end

            % Store history for replay attacks with size limit
            if isempty(data_history)
                data_history = containers.Map;
            end
            key = sprintf('%d_%s', vehicle_id, num2str(self.dt));
            if ~isKey(data_history, key)
                data_history(key) = struct('time', [], 'data', []);
            end
            history = data_history(key);
            history.time(end+1) = timestamp;
            history.data(:, end+1) = x_bar_j;

            % Limit history to last 100 entries to avoid memory issues
            max_history_size = 100;
            if length(history.time) > max_history_size
                history.time = history.time(end-max_history_size+1:end);
                history.data = history.data(:, end-max_history_size+1:end);
            end
            data_history(key) = history;



            for i = 1:length(self.scenario)
                if strcmp(self.scenario(i).type, 'time_based') && ...
                        (strcmp(self.scenario(i).data_type, 'local') || strcmp(self.scenario(i).data_type, 'both')) && ...
                        ismember(vehicle_id , self.scenario(i).attacker_id) && ...
                        timestamp >= self.scenario(i).start_time && ...
                        timestamp <= self.scenario(i).end_time
                    attack_type = self.scenario(i).attack_type;
                    fault_intensity = self.scenario(i).fault_intensity;

                    attack_rows = self.GetAttackRows_global(self.scenario(i).attack_row, size(x_bar_j, 1));

                    % Compute perturbation before applying attack
                    perturbation = zeros(size(x_bar_j));
                    switch attack_type
                        case 'faulty'
                            % Random faulty behavior - sometimes fault, sometimes normal
                            if isstruct(fault_intensity)
                                % New format: struct with 'intensity' and 'probability'
                                fault_prob = fault_intensity.probability;
                                fault_mag = fault_intensity.intensity;
                            else
                                % Backward compatibility: old format (simple number)
                                fault_prob = 0.3; % Default 30% probability
                                fault_mag = fault_intensity;
                            end
                            
                            if rand < fault_prob
                                % Apply random fault when probability triggers
                                perturbation(attack_rows) = randn(size(attack_rows)) * fault_mag;
                                x_bar_j(attack_rows) = x_bar_j(attack_rows) + perturbation(attack_rows);
                            else
                                % No fault this time step - behave normally
                                perturbation(attack_rows) = 0;
                            end
                        case 'bias'
                            perturbation(attack_rows) = fault_intensity;
                            x_bar_j(attack_rows) = x_bar_j(attack_rows) + perturbation(attack_rows);
                        case 'scaling'
                            perturbation(attack_rows) = x_bar_j(attack_rows) * fault_intensity;
                            x_bar_j(attack_rows) = x_bar_j(attack_rows) * (1 + fault_intensity);
                        case 'linear'
                            perturbation(attack_rows) = fault_intensity * (timestamp - self.scenario(i).start_time);
                            x_bar_j(attack_rows) = x_bar_j(attack_rows) + perturbation(attack_rows);
                        case 'sinusoidal'
                            amplitude = fault_intensity.amplitude;
                            frequency = fault_intensity.frequency;
                            perturbation(attack_rows) = amplitude * sin(2 * pi * frequency * (timestamp - self.scenario(i).start_time));
                            x_bar_j(attack_rows) = x_bar_j(attack_rows) + perturbation(attack_rows);
                        case 'replay'
                            % Replay data from `fault_intensity` seconds ago
                            delay = abs(fault_intensity);
                            idx = find(history.time <= timestamp - delay, 1, 'last');
                            if ~isempty(idx)
                                perturbation(attack_rows) = history.data(attack_rows, idx) - x_bar_j(attack_rows);
                                x_bar_j(attack_rows) = history.data(attack_rows, idx);
                            else
                                perturbation(attack_rows) = 0; % No change if no data available
                            end

                        case 'coordinated'
                            % For acceleration-velocity coordination
                            a_bias = fault_intensity.a_bias;
                            if any(strcmp(self.scenario(i).attack_row, 'acceleration'))
                                acc_rows = attack_rows(strcmp(self.scenario(i).attack_row, 'acceleration'));
                                perturbation(acc_rows) = a_bias;
                                x_bar_j(acc_rows) = x_bar_j(acc_rows) + perturbation(acc_rows);
                                % x_bar_j(attack_rows(strcmp(self.scenario(i).attack_row, 'acceleration'))) = ...
                                %     perturbation(acc_rows) = a_bias;
                                %     x_bar_j(attack_rows(strcmp(self.scenario(i).attack_row, 'acceleration'))) + a_bias;
                            end
                            if any(strcmp(self.scenario(i).attack_row, 'velocity'))
                                vel_rows = attack_rows(strcmp(self.scenario(i).attack_row, 'velocity'));
                                perturbation(vel_rows) = a_bias * (timestamp - self.scenario(i).start_time);
                                x_bar_j(vel_rows) = x_bar_j(vel_rows) + perturbation(vel_rows);
                            end
                            % case 'drop'
                            %     perturbation = NaN;
                            %     x_bar_j = NaN; % No data transmitted
                        case 'drop'
                            drop_chance = fault_intensity;  % Use fault_intensity as drop rate
                            if rand < drop_chance
                                perturbation = NaN;
                                x_bar_j = NaN;
                            end
                            % Add 'delay' logic here if needed, adjusting for timestamp
                        otherwise
                            warning('Unknown attack type in scenario %d: %s', i, attack_type);
                    end

                    % Store attack value
                    self.attack_values_local(end+1) = struct(...
                        'vehicle_id', vehicle_id, ...
                        'timestamp', timestamp, ...
                        'attack_type', attack_type, ...
                        'attack_rows', {attack_rows}, ...
                        'perturbation', perturbation);
                end
            end
        end

        %TODO : GLOBAL Should have another option to set attack in all rows

        % Helper function to get rows to attack
        function attack_rows = GetAttackRows_global(self, attacked_data, num_rows)
            % GETATTACKROWS_GLOBAL Determines the rows to attack based on the attacked data type.
            %
            % Inputs:
            %   attacked_data - Type of data to attack ('position', 'acceleration', 'all').
            %   num_rows      - Total number of rows in the data matrix.
            %
            % Output:
            %   attack_rows   - Rows to attack in the data matrix.

            attack_rows = [];
            if ischar(attacked_data)
                attacked_data = {attacked_data};
            end

            for i = 1:length(attacked_data)
                switch attacked_data{i}
                    case 'X'
                        attack_rows = [attack_rows, 1]; % Position X
                    case 'Y'
                        attack_rows = [attack_rows, 2]; % Position Y
                    case 'heading'
                        attack_rows = [attack_rows, 3]; % Heading
                    case 'velocity'
                        attack_rows = [attack_rows, 4]; % Velocity
                    case 'acceleration'
                        attack_rows = [attack_rows, 5]; % Acceleration
                    case 'all'
                        attack_rows = 1:num_rows; % All rows
                    otherwise
                        warning('Unknown attacked data type: %s', attacked_data{i});
                end
            end

            % Ensure unique rows and within bounds
            attack_rows = unique(attack_rows);
            attack_rows = attack_rows(attack_rows <= num_rows);
        end



        % New method to plot attack values
        function plotAttackValues(self, varargin)
            % PLOTATTACKVALUES Plots perturbations from attack_values.
            % Inputs (optional):
            %   'vehicle_id': Filter by vehicle ID (scalar or array, default: all)
            %   'data_type': Filter by data type ('X', 'velocity', etc., default: all)
            %   'attack_type': Filter by attack type ('bias', 'scaling', etc., default: all)
            %
            % Example:
            %   attack_module.plotAttackValues('vehicle_id', 1, 'data_type', 'velocity');

            % Parse input arguments
            p = inputParser;
            addParameter(p, 'vehicle_id', [], @(x) isempty(x) || isnumeric(x));
            addParameter(p, 'data_plot', [], @(x) ischar(x) );
            addParameter(p, 'data_type', '', @(x) ischar(x) || iscellstr(x));
            addParameter(p, 'attack_type', '', @(x) ischar(x) || iscellstr(x));
            parse(p, varargin{:});

            vehicle_id_filter = p.Results.vehicle_id;
            data_type_filter = p.Results.data_type;
            attack_type_filter = p.Results.attack_type;

            data_plot = p.Results.data_plot;

            if isempty(data_plot)
                warning('Which data to plot?');
                return;
            end
            if data_plot == "global"
                attack_values = self.attack_values_global;
            elseif data_plot == "local"
                attack_values = self.attack_values_local;
            else
                error('Invalid data type for plotting. Use "global" or "local".');
            end

            if isempty(attack_values)
                warning('No attack values to plot.');
                return;
            end

            % Get unique vehicle IDs if not filtered
            if isempty(vehicle_id_filter)
                vehicle_id_filter = unique([attack_values.vehicle_id]);
            end

            % Map row indices to data type labels (based on GetAttackRows_global)
            row_to_label = containers.Map(...
                {1, 2, 3, 4, 5}, ...
                {'X', 'Y', 'heading', 'velocity', 'acceleration'});

            % Initialize figure
            figure;
            hold on;
            legend_entries = {};
            plot_handles = [];

            % Plot for each vehicle
            for vid = vehicle_id_filter
                % Filter attack values for this vehicle
                vehicle_mask = [attack_values.vehicle_id] == vid;
                if ~any(vehicle_mask)
                    continue;
                end
                vehicle_values = attack_values(vehicle_mask);

                % Filter by attack type if specified
                if ~isempty(attack_type_filter)
                    if ischar(attack_type_filter)
                        attack_type_filter = {attack_type_filter};
                    end
                    vehicle_values = vehicle_values(ismember({vehicle_values.attack_type}, attack_type_filter));
                end

                % Group by data type (row)
                unique_rows = vehicle_values(1).attack_rows;
                if ~isempty(data_type_filter)
                    if ischar(data_type_filter)
                        data_type_filter = {data_type_filter};
                    end
                    % Convert data types to row indices
                    data_type_rows = [];
                    for dt = data_type_filter
                        for k = row_to_label.keys
                            if strcmp(row_to_label(k{1}), dt{1})
                                data_type_rows = [data_type_rows, k{1}];
                            end
                        end
                    end
                    unique_rows = intersect(unique_rows, data_type_rows);
                end

                % Plot each data type
                for row = unique_rows
                    timestamps = [];
                    perturbations = [];
                    for i = 1:length(vehicle_values)
                        if ismember(row, vehicle_values(i).attack_rows)
                            timestamps(end+1) = vehicle_values(i).timestamp;
                            perturbations(end+1) = vehicle_values(i).perturbation(row);
                        end
                    end
                    if isempty(timestamps)
                        continue;
                    end

                    % Sort by timestamp for clean plotting
                    [timestamps, sort_idx] = sort(timestamps);
                    perturbations = perturbations(sort_idx);

                    % Plot
                    h = plot(timestamps, perturbations, ...
                        'DisplayName', sprintf('Vehicle %d, %s', vid, row_to_label(row)));
                    plot_handles(end+1) = h;
                    legend_entries{end+1} = sprintf('Vehicle %d, %s', vid, row_to_label(row));
                end
            end

            hold off;
            xlabel('Time (s)');
            ylabel('Perturbation Value');
            title('Attack Perturbations Over Time');
            legend(legend_entries, 'Location', 'best');
            grid on;

            if isempty(plot_handles)
                warning('No data to plot after filtering.');
                close(gcf);
            end
        end



        % % Simulate cyber-attack on global data (similar changes apply)
        % function x_hat_i_j  = GetGlobalAttack(self, vehicle_id, x_hat_i_j_attacked,x_hat_i_j_normal, instant_idx)

        %     timestamp = instant_idx * self.dt; %convert to seconds

        %     if isempty(self.scenario)
        %         % disp('No attacks to apply');
        %         return;
        %     end

        %     for i = 1:length(self.scenario)
        %         if (ismember(vehicle_id ,self.scenario(i).victim_id))

        %             x_hat_i_j = x_hat_i_j_attacked;
        %         else
        %             x_hat_i_j = x_hat_i_j_normal;
        %         end
        %     end
        % end

        % % Simulate cyber-attack on local aggregated data
        % function [x_bar_j,original_data] = GetLocalAttack(self,vehicle_id, x_bar_attacked, x_bar_normal, instant_idx)


        %     timestamp = instant_idx * self.dt; %convert to seconds
        %     original_data = x_bar_j; % Preserve original for layering attacks
        %     % SETLOCALATTACK Simulates attacks based on all active scenarios.
        %     if isempty(self.scenario)
        %         % disp('No attacks to apply');
        %         return; % No attacks to apply
        %     end


        %     for i = 1:length(self.scenario)
        %         if (ismember(vehicle_id ,self.scenario(i).victim_id))
        %             x_bar_j = x_bar_attacked;
        %         else
        %             x_bar_j = x_bar_normal;
        %         end
        %     end

        % end
    end
end