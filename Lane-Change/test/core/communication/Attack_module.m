classdef Attack_module < handle
    % ATTACK_MODULE Class to simulate cyber-attacks on vehicle communication data.
    
    properties
        network_jitter;      % Maximum network jitter in seconds
        packet_loss_rate;    % Probability of packet loss (0 to 1)
        scenario;            % Struct with attack scenario details
    end
    
    methods
        % Constructor with default values
        function self = Attack_module()
            % Initialize default values for network jitter and packet loss rate
            self.network_jitter = 0.05;    % 50ms jitter
            self.packet_loss_rate = 0.1;   % 10% packet loss rate
            self.scenario = struct('type', 'none'); % Default: no scenario
        end

        function setScenario(self, scenario_type, scenario_params)
            % SETSCENARIO Configures the attack scenario.
            %
            % Inputs:
            %   scenario_type   - Type of scenario ('time_based' or 'none').
            %   scenario_params - Struct with scenario details (for 'time_based': 
            %                     start_time, end_time, attack_type, fault_intensity).
            
            switch scenario_type
                case 'time_based'
                    % Validate required fields
                    required_fields = {'start_time', 'end_time', 'attack_type', 'fault_intensity'};
                    for i = 1:length(required_fields)
                        if ~isfield(scenario_params, required_fields{i})
                            error('Missing required field: %s', required_fields{i});
                        end
                    end
                    self.scenario = struct('type', 'time_based', ...
                                          'start_time', scenario_params.start_time, ...
                                          'end_time', scenario_params.end_time, ...
                                          'attack_type', scenario_params.attack_type, ...
                                          'fault_intensity', scenario_params.fault_intensity);
                case 'none'
                    self.scenario = struct('type', 'none');
                otherwise
                    error('Unknown scenario type. Use "time_based" or "none".');
            end
        end

        % Simulate cyber-attack on global data for a specific vehicle
        function x_hat_i_j = SetGlobalAttack(self, j, x_hat_i_j, attacker_id, attack_type, fault_intensity, attacked_data)
            % SETGLOBALATTACK Simulates a cyber-attack on global vehicle communication data.
            %
            % Inputs:
            %   j               - Index of the vehicle being attacked.
            %   x_hat_i_j       - Global data matrix (e.g., 6x1 vector: [position; acceleration]).
            %   attacker_id     - ID of the vehicle simulating the attack.
            %   attack_type     - Type of attack ('faulty', 'bias', 'scaling', 'dos').
            %   fault_intensity - Intensity of the attack (scalar or vector).
            %   attacked_data   - Data to attack ('position', 'acceleration', 'all').
            %
            % Output:
            %   x_hat_i_j       - Modified global data matrix including the simulated attack.

            % Apply attack only if the vehicle is the attacker
            if j == attacker_id
                % Determine rows to attack based on attacked_data
                attack_rows = self.GetAttackRows_global(attacked_data, size(x_hat_i_j, 1));
                
                % Apply the specified attack type
                switch attack_type
                    case 'faulty'
                        % Add Gaussian noise to selected rows
                        x_hat_i_j(attack_rows) = x_hat_i_j(attack_rows) + randn(size(attack_rows)) * fault_intensity;
                    case 'bias'
                        % Add a constant offset to selected rows
                        x_hat_i_j(attack_rows) = x_hat_i_j(attack_rows) + fault_intensity;
                    case 'scaling'
                        % Scale the selected rows by a factor
                        x_hat_i_j(attack_rows) = x_hat_i_j(attack_rows) * (1 + fault_intensity);
                    case 'dos'
                        % Simulate packet loss with probability packet_loss_rate
                        if rand < self.packet_loss_rate
                            x_hat_i_j(attack_rows) = NaN;
                        end
                    otherwise
                        error('Unknown attack type. Choose "faulty", "bias", "scaling", or "dos".');
                end
            end
        end

        % Simulate cyber-attack on local aggregated data
        function x_bar_j = SetLocalAttack(self, x_bar_j, timestamp, attack_type, fault_intensity, attacked_data)
            % SETLOCALATTACK Simulates a cyber-attack on local aggregated data.
            %
            % Inputs:
            %   x_bar_j         - Local aggregated data (e.g., 6x1 vector).
            %   timestamp       - Timestamp when the data was generated (unused here but included for flexibility).
            %   attack_type     - Type of attack ('faulty', 'bias', 'scaling', 'delay', 'dos').
            %   fault_intensity - Intensity of the attack (scalar or vector).
            %   attacked_data   - Data to attack ('position', 'velocity', 'all').
            %
            % Output:
            %   x_bar_j         - Modified local data including the simulated attack.

            % Persistent buffer for delayed data
            persistent delayed_data;
            if isempty(delayed_data)
                delayed_data = [];
            end

            % Determine rows to attack based on attacked_data
            attack_rows = self.GetAttackRows_global(attacked_data, size(x_bar_j, 1));
            
            % Apply the specified attack type
            switch attack_type
                case 'faulty'
                    % Add Gaussian noise to selected rows
                    x_bar_j(attack_rows) = x_bar_j(attack_rows) + randn(size(attack_rows)) * fault_intensity;
                case 'bias'
                    % Add a constant offset to selected rows
                    x_bar_j(attack_rows) = x_bar_j(attack_rows) + fault_intensity;
                case 'scaling'
                    % Scale the selected rows by a factor
                    x_bar_j(attack_rows) = x_bar_j(attack_rows) * (1 + fault_intensity);
                case 'delay'
                    % Simulate delayed data arrival
                    delay_time = 2; % Delay in seconds
                    current_time = toc; % Current time in seconds since program start
                    delayed_data = [delayed_data; current_time + delay_time, x_bar_j(:)'];
                    ready_idx = delayed_data(:,1) <= current_time;
                    if any(ready_idx)
                        x_bar_j = delayed_data(ready_idx, 2:end)';
                        delayed_data(ready_idx,:) = [];
                    else
                        x_bar_j = NaN(size(x_bar_j)); % No data ready yet
                    end
                case 'dos'
                    % Always deny service
                    x_bar_j = NaN(size(x_bar_j));
                otherwise
                    error('Unknown attack type. Choose "faulty", "bias", "scaling", "delay", or "dos".');
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
            if strcmp(attacked_data, 'position')
                attack_rows = 1:2; % Position rows
            elseif strcmp(attacked_data, 'velocity')
                attack_rows = 4; % Velocity rows
            else
                attack_rows = 1:num_rows; % All rows
            end

            % Ensure attack_rows do not exceed num_rows
            attack_rows = attack_rows(attack_rows <= num_rows);
        end
    end
end