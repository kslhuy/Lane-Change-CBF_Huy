classdef Attack_module < handle
    % ATTACK_MODULE Class to simulate cyber-attacks on vehicle communication data.

    properties
        network_jitter;      % Maximum network jitter in seconds
        packet_loss_rate;    % Probability of packet loss (0 to 1)
        scenario;            % Struct with attack scenario details
        dt;                  % Time step for simulation
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
                    self.scenario = struct('type', {}, 'start_time', {}, 'end_time', {}, ...
                        'attack_type', {}, 'fault_intensity', {} , 'target',{}, 'attack_row',{}); % Empty array
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

            for i = 1:length(self.scenario)
                if (strcmp(self.scenario(i).type, 'time_based') && ...
                        (strcmp(self.scenario(i).data_type, 'global') || strcmp(self.scenario(i).data_type, 'both')) && ...
                        ismember(vehicle_id ,self.scenario(i).attacker_id) && ...
                        timestamp >= self.scenario(i).start_time && ...
                        timestamp <= self.scenario(i).end_time)

                    attack_type = self.scenario(i).attack_type;
                    fault_intensity = self.scenario(i).fault_intensity;

                    attack_rows = self.GetAttackRows_global(self.scenario(i).attack_row, size(x_hat_i_j, 1));
                    switch attack_type
                        case 'faulty'
                            x_hat_i_j(attack_rows) = x_hat_i_j(attack_rows) + randn(size(attack_rows)) * fault_intensity;
                        case 'bias'
                            x_hat_i_j(attack_rows) = x_hat_i_j(attack_rows) + fault_intensity;
                        case 'scaling'
                            x_hat_i_j(attack_rows) = x_hat_i_j(attack_rows) * (1 + fault_intensity);
                        case 'dos'
                            if rand < self.packet_loss_rate
                                x_hat_i_j(attack_rows) = NaN;
                            end
                        otherwise
                            warning('Unknown attack type in scenario %d: %s', i, attack_type);
                    end
                end
            end
        end

        % Simulate cyber-attack on local aggregated data
        function [x_bar_j,original_data] = SetLocalAttack(self,vehicle_id, x_bar_j, instant_idx)


            timestamp = instant_idx * self.dt; %convert to seconds
            original_data = x_bar_j; % Preserve original for layering attacks
            % SETLOCALATTACK Simulates attacks based on all active scenarios.
            if isempty(self.scenario)
                % disp('No attacks to apply');
                return; % No attacks to apply
            end


            for i = 1:length(self.scenario)
                if strcmp(self.scenario(i).type, 'time_based') && ...
                        (strcmp(self.scenario(i).data_type, 'local') || strcmp(self.scenario(i).data_type, 'both')) && ...
                        ismember(vehicle_id , self.scenario(i).attacker_id) && ...
                        timestamp >= self.scenario(i).start_time && ...
                        timestamp <= self.scenario(i).end_time
                    attack_type = self.scenario(i).attack_type;
                    fault_intensity = self.scenario(i).fault_intensity;

                    attack_rows = self.GetAttackRows_global(self.scenario(i).attack_row, size(x_bar_j, 1));

                    switch attack_type
                        case 'faulty'
                            x_bar_j(attack_rows) = x_bar_j(attack_rows) + randn(size(attack_rows)) * fault_intensity;
                        case 'bias'
                            x_bar_j(attack_rows) = x_bar_j(attack_rows) + fault_intensity;
                        case 'scaling'
                            x_bar_j(attack_rows) = x_bar_j(attack_rows) * (1 + fault_intensity);
                        case 'dos'
                            if rand < self.packet_loss_rate
                                x_bar_j(attack_rows) = NaN;
                            end
                            % Add 'delay' logic here if needed, adjusting for timestamp
                        otherwise
                            warning('Unknown attack type in scenario %d: %s', i, attack_type);
                    end
                end
            end
        end

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

            % Define rows based on data type (assumes 6x1 vector: 1-3 position, 4-6 acceleration)
            if strcmp(attacked_data, 'X')
                attack_rows = 1; % Position rows
            elseif strcmp(attacked_data, 'Y')
                attack_rows = 2;
            elseif strcmp(attacked_data, 'heading')
                attack_rows = 3;
            elseif strcmp(attacked_data, 'velocity')
                attack_rows = 4; % Velocity rows
            else
                attack_rows = 1:num_rows; % All rows
            end

            % Ensure attack_rows do not exceed num_rows
            attack_rows = attack_rows(attack_rows <= num_rows);
        end

        % Simulate cyber-attack on global data (similar changes apply)
        function x_hat_i_j  = GetGlobalAttack(self, vehicle_id, x_hat_i_j_attacked,x_hat_i_j_normal, instant_idx)

            timestamp = instant_idx * self.dt; %convert to seconds
            
            if isempty(self.scenario)
                % disp('No attacks to apply');
                return;
            end

            for i = 1:length(self.scenario)
                if (ismember(vehicle_id ,self.scenario(i).victim_id))

                    x_hat_i_j = x_hat_i_j_attacked;
                else
                    x_hat_i_j = x_hat_i_j_normal;
                end
            end
        end

        % Simulate cyber-attack on local aggregated data
        function [x_bar_j,original_data] = GetLocalAttack(self,vehicle_id, x_bar_attacked, x_bar_normal, instant_idx)


            timestamp = instant_idx * self.dt; %convert to seconds
            original_data = x_bar_j; % Preserve original for layering attacks
            % SETLOCALATTACK Simulates attacks based on all active scenarios.
            if isempty(self.scenario)
                % disp('No attacks to apply');
                return; % No attacks to apply
            end


            for i = 1:length(self.scenario)
                if (ismember(vehicle_id ,self.scenario(i).victim_id))
                    x_bar_j = x_bar_attacked;
                else
                    x_bar_j = x_bar_normal;
                end
            end
            
        end
    end
end