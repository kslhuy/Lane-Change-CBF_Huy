classdef Observer < handle
    properties
        type; % type of the controller,
        param_sys; % parameter for vehicle
        straightlane;

        est_local_state_current;
        est_global_state_current;

        est_global_state_log;
        est_local_state_log;
        vehicle;
        controller;
        tolerances;
        log_element = [];
        Is_ok_log = [];
        L_gain;
        P; % error covariance
        Q; % process noise covariance
        R; % measurement noise covariance
        num_states;

        num_vehicles;

        % Properties for distributed observer / Controller
        self_belief; % Self-belief for distributed observer prediction
        reputation_scores; % Reputation scores for neighbors [num_vehicles x 1]
        P_pred_dist; % Predicted error covariance for distributed observer
        S_log_dist; % Innovation covariance log for distributed observer
        self_belief_log; % Log for self-belief
        target_weights_log; % Actual per-target observer weights [source/channel x time x target]
        target_weights_current; % Latest actual per-target weights [source/channel x target]
        u_j_last_predict = [0; 0]; % Last predicted control input for vehicle j

        predict_only_counter = []; % Counter for consecutive good steps in prediction-only mode
        predict_only_timer =   []; % Timer for prediction-only mode
        is_in_prediction_mode = []; % Boolean array tracking prediction-only mode for each vehicle
        
        % Properties for smoother noise and observer output
        noise_filter_alpha = 0.7; % Exponential smoothing parameter for noise (0.0 = no filtering, 1.0 = max filtering)
        previous_noise = []; % Store previous noise for smoothing
        output_filter_alpha = 0.3; % Low-pass filter parameter for observer output
        previous_local_output = []; % Store previous local observer output for smoothing
        measurement_noise_correlation = 0.8; % Temporal correlation for generated measurement noise
        correlated_noise_state = []; % State for correlated noise generation
        
        % Properties for contamination rollback system
        rollback_enabled = true; % Enable/disable rollback functionality
        rollback_window_size = 15; % Number of steps to keep in buffer (K)
        rollback_buffer = {}; % Circular buffer storing step data for rollback
        rollback_buffer_index = 1; % Current position in circular buffer
        rollback_buffer_full = false; % Whether buffer has been filled once
        malicious_vehicles = []; % List of currently flagged malicious vehicles
        trust_threshold = 0.5; % Threshold below which vehicle is considered malicious
        rollback_stats = struct('total_rollbacks', 0, 'vehicles_flagged', [], 'rollback_times', []); % Statistics
        rollback_trusted_state_history = {};
        rollback_trusted_state_history_size = 15;
        rollback_trusted_state_guard_steps = 0;
        rollback_rewrite_history_log = false;
        rollback_on_final_trust = true;
        rollback_on_local_est_check = true;
        rollback_on_global_est_check = true;
        rollback_start_time = 0;
        rollback_required_bad_steps = 1;
        rollback_bad_counters = [];
        rollback_recovery_good_steps = 1;
        rollback_recovery_counters = [];
        rollback_cooldown_steps = 0;
        rollback_last_trigger_step = -Inf;
        local_bad_zero_w0_neighbor_total_cap = 0.01;
    end
    methods
        function self = Observer( vehicle , veh_param, inital_global_state, inital_local_state)
            self.vehicle = vehicle;
            self.param_sys = veh_param; % Set the vehicle system parameters

            self.est_local_state_current = inital_local_state;

            self.est_global_state_current = inital_global_state;

            self.num_states = 5 ; % X , Y , theta , V,A
            Nt = self.vehicle.total_time_step;
            self.num_vehicles = length(self.vehicle.other_vehicles);

            self.est_global_state_log = zeros(self.num_states, Nt, self.num_vehicles);
            % Initialize the log with proper 3D indexing
            % for v = 1:self.num_vehicles
            %     self.est_global_state_log(:, 1, v) = inital_global_state(:, v);
            % end
            self.est_global_state_log(:, 1, self.vehicle.vehicle_number) = inital_global_state(:, self.vehicle.vehicle_number);

            

            self.est_local_state_log = zeros(self.num_states, Nt);
            self.est_local_state_log(:,1) = inital_local_state;

            self.tolerances = [3.5, 2, deg2rad(8), 2 , 1]; % Tolerances for each state [X, Y, theta, V, A]


            self.predict_only_counter = zeros(self.num_vehicles, 1);
            self.predict_only_timer = zeros(self.num_vehicles, 1);
            self.is_in_prediction_mode = false(self.num_vehicles, 1); % Initialize as boolean array
            % NEW : for calculate the self belief = gamma in the controller
            self.self_belief = 1; % Initial self-belief
            self.reputation_scores = ones(self.num_vehicles, 1); % Initial reputation = 1
            self.P_pred_dist = eye(self.num_states); % Initial covariance
            self.S_log_dist = zeros(self.num_states, self.num_states, Nt); % Log innovation covariance
            self.target_weights_log = NaN(self.num_vehicles + 1, Nt, self.num_vehicles);
            self.target_weights_current = NaN(self.num_vehicles + 1, self.num_vehicles);
            
            % Initialize smoothing properties
            self.noise_filter_alpha = self.config_unit_interval(self.vehicle.scenarios_config.noise_filter_alpha, 'noise_filter_alpha');
            self.output_filter_alpha = self.config_unit_interval(self.vehicle.scenarios_config.local_observer_output_filter_alpha, 'local_observer_output_filter_alpha');
            self.measurement_noise_correlation = self.config_unit_interval(self.vehicle.scenarios_config.measurement_noise_correlation, 'measurement_noise_correlation');
            self.previous_noise = zeros(self.num_states, 1);
            self.previous_local_output = inital_local_state;
            self.correlated_noise_state = zeros(self.num_states, 1);
            self.rollback_enabled = self.vehicle.scenarios_config.rollback_enabled;
            self.rollback_window_size = max(1, round(self.scenario_value('rollback_window_size', self.rollback_window_size)));
            self.rollback_trusted_state_history_size = max(1, round(self.scenario_value('rollback_trusted_state_history_size', self.rollback_window_size)));
            self.rollback_trusted_state_guard_steps = max(0, round(self.scenario_value('rollback_trusted_state_guard_steps', 0)));
            self.rollback_rewrite_history_log = logical(self.scenario_value('rollback_rewrite_history_log', false));
            self.rollback_on_final_trust = logical(self.scenario_value('rollback_on_final_trust', true));
            self.rollback_on_local_est_check = logical(self.scenario_value('rollback_on_local_est_check', true));
            self.rollback_on_global_est_check = logical(self.scenario_value('rollback_on_global_est_check', true));
            self.rollback_start_time = max(0, self.scenario_value('rollback_start_time', 0));
            self.rollback_required_bad_steps = max(1, round(self.scenario_value('rollback_required_bad_steps', 1)));
            self.rollback_recovery_good_steps = max(1, round(self.scenario_value('rollback_recovery_good_steps', 1)));
            self.rollback_cooldown_steps = max(0, round(self.scenario_value('rollback_cooldown_steps', 0)));
            self.local_bad_zero_w0_neighbor_total_cap = self.config_unit_interval( ...
                self.scenario_value('local_bad_zero_w0_neighbor_total_cap', 0.01), ...
                'local_bad_zero_w0_neighbor_total_cap');
            self.rollback_trusted_state_history = cell(self.num_vehicles, 1);
            self.rollback_bad_counters = zeros(self.num_vehicles, 1);
            self.rollback_recovery_counters = zeros(self.num_vehicles, 1);
            
            % Initialize rollback system
            if self.rollback_enabled
                % Pre-allocate rollback buffer for efficiency
                self.rollback_buffer = cell(self.rollback_window_size, 1);
                self.rollback_buffer_index = 1;
                self.rollback_buffer_full = false;
                self.malicious_vehicles = [];
                self.rollback_bad_counters = zeros(self.num_vehicles, 1);
                self.rollback_recovery_counters = zeros(self.num_vehicles, 1);
                self.rollback_last_trigger_step = -Inf;
                
                % Initialize rollback statistics
                self.rollback_stats.total_rollbacks = 0;
                self.rollback_stats.vehicles_flagged = [];
                self.rollback_stats.rollback_times = [];
                
                fprintf('[Observer V%d] Contamination rollback system initialized with window size %d, cooldown %d steps, recovery %d clean steps, start %.2fs, confirm %d steps\n', ...
                    self.vehicle.vehicle_number, self.rollback_window_size, self.rollback_cooldown_steps, ...
                    self.rollback_recovery_good_steps, self.rollback_start_time, self.rollback_required_bad_steps);
            end


            if (self.vehicle.scenarios_config.Local_observer_type == "kalman")
                % Kalman Filter Parameters

                self.P = eye(self.num_states);              % initial error covariance

            elseif  (self.vehicle.scenarios_config.Local_observer_type == "observer")
                % Observer Gain
                desired_poles = [-1 -2 -3 -4 -5]; % Desired poles for the observer
                self.L_gain = place(A', C', desired_poles)';
            end

            if self.vehicle.scenarios_config.Is_noise_mesurement == true % if the measurement is noisy
                %% Noise Covariances
                self.R = self.config_covariance(self.vehicle.scenarios_config.measurement_noise_variance, 'measurement_noise_variance');
                self.Q = self.config_covariance(self.vehicle.scenarios_config.process_noise_variance, 'process_noise_variance');

            else
                self.Q = self.config_covariance(self.vehicle.scenarios_config.no_noise_process_variance, 'no_noise_process_variance');
                self.R = self.config_covariance(self.vehicle.scenarios_config.no_noise_measurement_variance, 'no_noise_measurement_variance');
            end
        end


        function  Distributed_Observer(self,instant_index , weights)
            % Get the number of other vehicles
            Big_X_hat_1_tempo = zeros(size(self.est_global_state_current)); % Initialize the variable to store the results
            host_id = self.vehicle.vehicle_number; % The vehicle that is estimating the state of other vehicles
            confidence_scores = zeros(self.num_vehicles, 1); % Store per-vehicle confidence
            trust_scores = self.get_current_trust_scores(instant_index);

            % Initialize rollback data collection
            if self.rollback_enabled
                pre_update_states = self.est_global_state_current; % Store states before update
                weights_used = cell(self.num_vehicles, 1); % Store weights used for each vehicle
                target_components = cell(self.num_vehicles, 1); % Replayable source-state components
            end

            % j is the vehicle we want to estimate
            for j = 1:self.num_vehicles
                weights_new = self.get_weights_for_vehicle(j, host_id, weights, instant_index);
                [x_bar_j, weights_new, direct_available] = self.get_local_state_for_vehicle(j, host_id, weights_new);
                [x_hat_i_j, weights_new, source_available] = self.get_global_states_for_vehicle(j, self.num_vehicles, host_id, weights_new, instant_index);
                weights_new = self.get_python_target_weights_if_enabled( ...
                    j, host_id, weights_new, trust_scores, source_available, ...
                    direct_available, x_bar_j, instant_index);
                u_j = self.Get_controller(j);
                weights_new = self.adjust_weights_for_attacker(j, host_id, weights_new);
                self.record_target_weights(instant_index, j, weights_new);
                use_local_data_from_other = self.vehicle.scenarios_config.use_local_data_from_other;
                
                % Capture deltas for rollback before computing output
                if self.rollback_enabled
                    [output, ~, ~, ~, replay_component] = ...
                        self.distributed_Observer_each_with_deltas(self.vehicle.vehicle_number, j, x_bar_j, x_hat_i_j, u_j, weights_new, use_local_data_from_other, false);
                    
                    weights_used{j} = weights_new;
                    target_components{j} = replay_component;
                else
                    output = self.distributed_Observer_each(self.vehicle.vehicle_number, j, x_bar_j, x_hat_i_j, u_j, weights_new, use_local_data_from_other, false);
                end
                
                % %% TEST
                % % output = self.distributed_Observer_each_simplified(self.vehicle.vehicle_number, j, x_bar_j, x_hat_i_j, u_j, weights_new, use_local_data_from_other, false);
                % Big_X_hat_1_tempo(:,j) = output;
                % %% END TEST


                self.update_reputation(j, output, x_bar_j);
                if instant_index*self.param_sys.dt < 3
                    Big_X_hat_1_tempo(:,j) = output;
                    self.Is_ok_log = [self.Is_ok_log, true];
                elseif self.vehicle.scenarios_config.Use_predict_observer
                    % Use sophisticated prediction-only switching logic
                    [Big_X_hat_1_tempo(:,j), temp_confidence_scores] = self.handle_predict_only_switch(j, self.num_vehicles, output, x_bar_j, x_hat_i_j, u_j, weights_new, use_local_data_from_other, instant_index, confidence_scores);
                    confidence_scores = temp_confidence_scores;
                else
                    % Prediction-only mode disabled: always use normal output
                    Big_X_hat_1_tempo(:,j) = output;
                    self.Is_ok_log = [self.Is_ok_log, true];
                    confidence_scores(j) = 1.0; % High confidence when not using prediction-only
                end
            end

            self.self_belief = 1; % Reset self-belief for next iteration
            % Compute self_belief as average of confidence scores
            if instant_index * self.param_sys.dt > 2
                self.self_belief = mean(confidence_scores);
            end

            self.self_belief_log = [self.self_belief_log, self.self_belief];


            %% Store rollback data and check for contamination
            if self.rollback_enabled && instant_index*self.param_sys.dt >= self.rollback_start_time
                % Store step data in rollback buffer
                self.create_step_data(instant_index, pre_update_states, weights_used, trust_scores, target_components);
                
                % Check for newly malicious vehicles based on trust threshold
                [rollback_applied, corrected_states] = self.check_and_trigger_rollback(instant_index, trust_scores, Big_X_hat_1_tempo);
                if rollback_applied
                    Big_X_hat_1_tempo = corrected_states;
                end

                self.update_rollback_trusted_state_history(instant_index, trust_scores, Big_X_hat_1_tempo);
            end

            %% Save the global state estmate to log
            self.est_global_state_current = Big_X_hat_1_tempo;
            self.est_global_state_log(:, instant_index, :) = Big_X_hat_1_tempo;
        end

        function weights_new = get_weights_for_vehicle(self, j, host_id, weights, instant_index)
            % Copy the shared row before target-specific trust edits.
            weights_new = double(weights(:)');
            if self.get_trip_flag(j, 'flag_local_est_check')
                weights_new(1) = 0;
                weights_new = self.cap_neighbor_influence_to_self(weights_new, host_id, self.local_bad_zero_w0_neighbor_total_cap);
            end
            weights_new = self.normalize_observer_weights(weights_new, host_id);
        end

        function record_target_weights(self, instant_index, target_id, weights)
            weights = double(weights(:));
            expected_len = self.num_vehicles + 1;
            if length(weights) < expected_len
                weights(end + 1:expected_len) = NaN;
            elseif length(weights) > expected_len
                weights = weights(1:expected_len);
            end

            self.target_weights_current(:, target_id) = weights;
            if instant_index > 0 && instant_index <= size(self.target_weights_log, 2)
                self.target_weights_log(:, instant_index, target_id) = weights;
            end
        end

        function [x_bar_j , weights_new, direct_available] = get_local_state_for_vehicle(self, j, host_id, weights_new)
            % Get local state for vehicle j, handle missing data
            x_bar_j = self.vehicle.center_communication.get_local_state(j, host_id);
            direct_available = true;
            if any(isnan(x_bar_j(:)))
                x_bar_j = zeros(size(self.est_local_state_current));
                weights_new(1) = 0;
                direct_available = false;
                weights_new = self.normalize_observer_weights(weights_new, host_id);
            end
        end

        function [x_hat_i_j , weights_new, source_available] = get_global_states_for_vehicle(self, j, num_vehicles, host_id, weights_new, instant_index)
            % Get global state estimates for all vehicles (PREVIOUS estimates, not current)
            x_hat_i_j = zeros(self.num_states, num_vehicles);
            source_available = false(1, num_vehicles);
            for k = 1:num_vehicles
                x_hat_i_j_full = self.vehicle.center_communication.get_global_state(k, self.vehicle.vehicle_number);
                if any(isnan(x_hat_i_j_full(:)))
                    x_hat_i_j_full = zeros(size(self.est_global_state_current));
                    weights_new(k+1) = 0;
                else
                    source_available(k) = true;
                end
                x_hat_i_j(:, k) = x_hat_i_j_full(:, j);
            end
            weights_new = self.normalize_observer_weights(weights_new, host_id);

        end

        function trust_scores = get_current_trust_scores(self, instant_index)
            trust_scores = ones(1, self.num_vehicles);
            if isempty(self.vehicle) || isempty(self.vehicle.trust_log)
                return;
            end
            for v = 1:self.num_vehicles
                if instant_index > 0 && instant_index <= size(self.vehicle.trust_log, 2)
                    trust_value = self.vehicle.trust_log(1, instant_index, v);
                    if isfinite(trust_value)
                        trust_scores(v) = trust_value;
                    end
                end
            end
        end

        function weights_new = get_python_target_weights_if_enabled(self, j, host_id, weights_new, trust_scores, source_available, direct_available, x_bar_j, instant_index)
            if ~self.should_use_python_target_weights()
                weights_new = self.normalize_observer_weights(weights_new, host_id);
                return;
            end

            direct_measurement = [];
            if direct_available
                direct_measurement = x_bar_j;
            end
            if isprop(self.vehicle.weight_module, 'vehicle_id')
                self.vehicle.weight_module.vehicle_id = host_id;
            end

            target_trust_model = [];
            if ~isempty(self.vehicle.trip_models) && j <= length(self.vehicle.trip_models)
                target_trust_model = self.vehicle.trip_models{j};
            end

            if self.use_startup_fixed_target_weights(instant_index)
                weights_new = self.vehicle.weight_module.calculate_startup_weights_for_target( ...
                    host_id, j, source_available, direct_measurement);
            else
                if ~self.is_direct_measurement_allowed_for_target(j, trust_scores, target_trust_model)
                    direct_measurement = [];
                end
                weights_new = self.vehicle.weight_module.calculate_weights_for_target( ...
                    host_id, j, trust_scores, source_available, direct_measurement, target_trust_model);
            end
            weights_new = self.normalize_observer_weights(weights_new, host_id);
        end

        function use_target_weights = should_use_python_target_weights(self)
            use_target_weights = false;
            if isempty(self.vehicle) || isempty(self.vehicle.scenarios_config)
                return;
            end
            if ~logical(self.scenario_value('using_weight_trust_observer', false))
                return;
            end
            if isempty(self.vehicle.weight_module) || ~ismethod(self.vehicle.weight_module, 'calculate_weights_for_target')
                return;
            end
            use_target_weights = true;
        end

        function use_startup = use_startup_fixed_target_weights(self, instant_index)
            use_startup = false;
            if isempty(self.vehicle) || isempty(self.vehicle.weight_module) || ~isprop(self.vehicle.weight_module, 'startup_fixed_duration_s')
                return;
            end
            duration_s = max(0, double(self.vehicle.weight_module.startup_fixed_duration_s));
            if duration_s <= 0
                return;
            end
            use_startup = instant_index * self.param_sys.dt < duration_s;
        end

        function allowed = is_direct_measurement_allowed_for_target(self, target_id, trust_scores, target_trust_model)
            % Match Python estimator gating: only local-channel failure suppresses w0.
            allowed = true;
            if self.get_trip_flag(target_id, 'flag_local_est_check')
                allowed = false;
                return;
            end

            local_trust = self.read_latest_trip_unit(target_trust_model, ...
                {'local_trust_sample', 'trust_sample_log', 'local_trust_decayed_log'}, NaN);
            if ~isfinite(local_trust)
                if target_id >= 1 && target_id <= numel(trust_scores)
                    local_trust = trust_scores(target_id);
                else
                    return;
                end
            end

            allowed = local_trust >= self.trust_threshold;
        end

        function value = read_latest_trip_unit(~, model, names, default_value)
            value = default_value;
            if isempty(model)
                return;
            end
            for idx = 1:length(names)
                name = names{idx};
                if isstruct(model) && isfield(model, name)
                    candidate = model.(name);
                elseif isobject(model) && isprop(model, name)
                    candidate = model.(name);
                else
                    continue;
                end
                if isempty(candidate)
                    continue;
                end
                candidate = double(candidate(:));
                candidate = candidate(isfinite(candidate));
                if isempty(candidate)
                    continue;
                end
                value = min(1.0, max(0.0, candidate(end)));
                return;
            end
        end

        function weights_new = adjust_weights_for_attacker(self, j, host_id, weights_new)
            % Adjust weights if host is attacker
            if self.vehicle.scenarios_config.attacker_update_locally
                if (~isempty(self.vehicle.center_communication.attack_module.scenario) && ismember(host_id, self.vehicle.center_communication.attack_module.scenario(1).attacker_id)) || (self.vehicle.scenarios_config.lead_senario ~= "constant")
                    weights_new = zeros(size(weights_new));
                    weights_new(1) = 1; % Keep the local state weight
                end
            end
            weights_new = self.normalize_observer_weights(weights_new, host_id);
        end

        function [output_final, confidence_scores] = handle_predict_only_switch(self, j, num_vehicles, output, x_bar_j, x_hat_i_j, u_j, weights_new, use_local_data_from_other, instant_index, confidence_scores)
            % Improved: add time limit and blending for prediction-only mode
            
            MAX_PREDICT_ONLY_TIME = self.vehicle.scenarios_config.MAX_PREDICT_ONLY_TIME; % seconds
            N_good = self.vehicle.scenarios_config.N_good; % Number of consecutive good steps to exit predict_only
            dt = self.param_sys.dt;
            if isempty(self.predict_only_counter)
                self.predict_only_counter = zeros(self.num_vehicles, 1);
            end
            if isempty(self.predict_only_timer)
                self.predict_only_timer = zeros(self.num_vehicles, 1);
            end

            output_2 = self.distributed_Observer_each(self.vehicle.vehicle_number, j, x_bar_j, x_hat_i_j, u_j, weights_new, use_local_data_from_other, true);
            [is_ok, log_elem, confidence] = self.check_elementwise_similarity(output, output_2, instant_index, j);
            confidence_scores(j) = confidence;
            self.Is_ok_log = [self.Is_ok_log, is_ok];

            % % Logging: similarity check result
            % fprintf('[Observer] t=%.2f, Vehicle %d: Similarity check: is_ok=%d, confidence=%.3f\n', instant_index*dt, j, is_ok, confidence);

            % prev_mode = 'normal';
            % if self.predict_only_counter(j) < N_good
            %     prev_mode = 'prediction-only';
            % end

            if is_ok
                self.predict_only_counter(j) = self.predict_only_counter(j) + 1;
                if self.predict_only_counter(j) == N_good
                    % Logging: switching back to normal mode
                    % fprintf('[Observer] t=%.2f, Vehicle %d: Switching from prediction-only to NORMAL mode (N_good reached)\n', instant_index*dt, j);
                end
            else
                if self.predict_only_counter(j) >= N_good
                    % Logging: switching to prediction-only mode
                    % fprintf('[Observer] t=%.2f, Vehicle %d: Switching from NORMAL to prediction-only mode (bad data detected)\n', instant_index*dt, j);
                end
                self.predict_only_counter(j) = 0;
            end
            
            % Update the prediction mode status for easy access
            self.is_in_prediction_mode(j) = (self.predict_only_counter(j) < N_good);

            % If in prediction-only mode, increment timer
            if self.predict_only_counter(j) < N_good && (~is_ok && self.vehicle.scenarios_config.Use_predict_observer)
                self.predict_only_timer(j) = self.predict_only_timer(j) + dt;
                % Logging: prediction-only timer increment
                % fprintf('[Observer] t=%.2f, Vehicle %d: In prediction-only mode, timer=%.2f/%.2f\n', instant_index*dt, j, self.predict_only_timer(j), MAX_PREDICT_ONLY_TIME);
            else
                if self.predict_only_timer(j) > 0
                    % Logging: prediction-only timer reset
                    % fprintf('[Observer] t=%.2f, Vehicle %d: Exiting prediction-only mode, timer reset.\n', instant_index*dt, j);
                end
                self.predict_only_timer(j) = 0;
            end

            % If prediction-only mode has lasted too long, force switch back
            if self.predict_only_timer(j) >= MAX_PREDICT_ONLY_TIME
                % fprintf('[Observer] t=%.2f, Vehicle %d: Prediction-only mode exceeded max time (%.2fs). Forcing switch to NORMAL mode.\n', instant_index*dt, j, MAX_PREDICT_ONLY_TIME);
                output_final = output; % Force trust new data
                self.predict_only_counter(j) = N_good; % Reset counter to allow normal mode
                self.predict_only_timer(j) = 0;
                return;
            end

            % Blending: if drift is large but data is likely good, blend outputs
            if self.predict_only_counter(j) < N_good && is_ok == false && self.vehicle.scenarios_config.Use_predict_observer
                % Weighted norm: scale each state by its tolerance
                weights = 1 ./ self.tolerances(:); % column vector
                diff_vec = (output - output_2) .* weights;
                diff_norm = norm(diff_vec);
                blend_thresh = self.vehicle.scenarios_config.blend_thresh; % You can tune this threshold
                if diff_norm < blend_thresh
                    alpha = min(0.5, diff_norm / blend_thresh); % Blend factor
                    output_final = (1-alpha)*output + alpha*output_2;
                    % Logging: blending outputs
                    % fprintf('[Observer] t=%.2f, Vehicle %d: Blending outputs (diff_norm=%.3f < thresh=%.3f, alpha=%.3f)\n', instant_index*dt, j, diff_norm, blend_thresh, alpha);
                else
                    self.log_element = [self.log_element, log_elem];
                    output_final = output_2;
                    % Logging: using prediction-only output (large drift)
                    % fprintf('[Observer] t=%.2f, Vehicle %d: Using prediction-only output (diff_norm=%.3f >= thresh=%.3f)\n', instant_index*dt, j, diff_norm, blend_thresh);
                end
            elseif self.predict_only_counter(j) >= N_good
                output_final = output;
            else
                output_final = output_2;
            end
        end

        function output = distributed_Observer_each( self , host_id , j, x_bar_j , x_hat_i_j, u_j ,weights,use_local, predict_only, instant_index )
            if nargin < 7
                use_local = true; % Default value
            end
            if nargin < 10
                instant_index = 1; % Default value
            end

            Sig = zeros(self.num_states,1); % Initialize consensus term
            
            % CORRECT: x_hat_i_j(:, host_id) is actually our previous estimate of vehicle j
            % This is what we want to use for both linearization and as the base state
            % x_j_prev = x_hat_i_j(:, host_id); % Our current/previous estimate of vehicle j
            
            [A , B]  = self.matrix();
            
            % Calculate the consensus term
            % Start from 2 , because the first element is the local state of the vehicle
            for L = 2:length(weights)
                % Why x_hat_i_j(: , l-1) because L start from 2 , but we need to start from 1 for vehicle
                Sig = weights(L)*( x_hat_i_j(: , L-1) - x_hat_i_j(: , host_id) ) + Sig;
            end
            

            %% Some situation
            % Only if we try to estimated j = i , so we have real local state that was estimated by Local observer .
            % But if we try to estimate j != i , we have (fake) the local state of j , that by vehicle j send to i vehicle

            % Estimate the j vehicle , in i vehicle
            % So the observer is implement in i vehicle
            w_i0 = weights(1); % Weight of the local state

            %% Original paper (TRUE WORK)

            if predict_only %% Just use the last state and predict the next state , maybe use also the last control
                if (host_id == j)
                    x_hat_i_j(: , host_id)  =  x_hat_i_j(: , host_id) + w_i0 * (x_bar_j - x_hat_i_j(: , host_id))  ;
                end
                [output, ~, S] = self.predict_kalman_dist(host_id, j, x_hat_i_j(:, host_id), u_j);
                self.S_log_dist(:, :, host_id) = S; % Store for check_elementwise_similarity
            else
                if use_local
                    % NORMAL CASE have received local state from other vehicle
                    output = A*( x_hat_i_j(: , host_id) + Sig + w_i0 * (x_bar_j - x_hat_i_j(: , host_id)) ) + B*u_j ;
                else
                    % CASE not received local state , only host vehicle have his own local state to use
                    if (host_id == j)
                        output = A*( x_hat_i_j(: , host_id) + Sig + w_i0 * (x_bar_j - x_hat_i_j(: , host_id)) ) + B*u_j ;
                    else
                        % the other vehicle that we want to estimate , we dont have local data of them
                        output = A*( x_hat_i_j(: , host_id) + Sig) + B*u_j ;
                    end
                end
            end
        end

        function [output, neighbor_deltas, anchor_delta, input_delta, replay_component] = distributed_Observer_each_with_deltas(self, host_id, j, x_bar_j, x_hat_i_j, u_j, weights, use_local, predict_only)
            % Same as distributed_Observer_each, plus replayable source-state terms.
            if nargin < 9
                use_local = true; % Default value
            end
            if nargin < 10
                predict_only = false;
            end

            [A, B] = self.matrix();
            
            % Initialize deltas
            neighbor_deltas = {}; % Cell array for each neighbor's delta
            anchor_delta = zeros(self.num_states, 1);
            input_delta = zeros(self.num_states, 1);
            replay_component = self.build_replay_component(j, host_id, x_bar_j, x_hat_i_j, u_j, weights, use_local, predict_only);
            
            % Calculate consensus term and store individual neighbor deltas
            Sig = zeros(self.num_states, 1);
            neighbor_count = 0;
            for L = 2:length(weights)
                neighbor_idx = L - 1; % Convert to vehicle index
                if neighbor_idx == host_id || weights(L) == 0
                    continue;
                end
                delta_neighbor = weights(L) * (x_hat_i_j(:, neighbor_idx) - x_hat_i_j(:, host_id));
                neighbor_deltas{neighbor_count + 1} = struct('vehicle_idx', neighbor_idx, 'delta', delta_neighbor, 'weight', weights(L));
                Sig = Sig + delta_neighbor;
                neighbor_count = neighbor_count + 1;
            end
            
            w_i0 = weights(1); % Weight of the local state
            
            if predict_only
                if (host_id == j)
                    x_hat_i_j(:, host_id) = x_hat_i_j(:, host_id) + w_i0 * (x_bar_j - x_hat_i_j(:, host_id));
                    anchor_delta = A *(w_i0 * (x_bar_j - x_hat_i_j(:, host_id))) ;
                end
                [output, ~, S] = self.predict_kalman_dist(host_id, j, x_hat_i_j(:, host_id), u_j);
                self.S_log_dist(:, :, host_id) = S;
                
                % For prediction mode, deltas are different (no consensus, different anchor)
                input_delta = B * u_j;
            else
                if use_local
                    % Calculate anchor delta
                    anchor_delta = A * w_i0 * (x_bar_j - x_hat_i_j(:, host_id));
                    
                    % Calculate input delta
                    input_delta = B * u_j;
                    
                    % Apply consensus transformation
                    consensus_delta = A * Sig;
                    for i = 1:length(neighbor_deltas)
                        neighbor_deltas{i}.delta = A * neighbor_deltas{i}.delta;
                    end
                    
                    output = A * x_hat_i_j(:, host_id) + consensus_delta + anchor_delta + input_delta;
                else
                    % Case not received local state, only host vehicle have his own local state to use
                    if (host_id == j)
                        anchor_delta = A * w_i0 * (x_bar_j - x_hat_i_j(:, host_id));
                        output = A * x_hat_i_j(:, host_id) + A * Sig + anchor_delta + B * u_j;
                    else
                        % No local data for other vehicles
                        anchor_delta = zeros(self.num_states, 1); % No anchor correction
                        output = A * x_hat_i_j(:, host_id) + A * Sig + B * u_j;
                    end
                    input_delta = B * u_j;
                    
                    % Apply A transformation to neighbor deltas
                    for i = 1:length(neighbor_deltas)
                        neighbor_deltas{i}.delta = A * neighbor_deltas{i}.delta;
                    end
                end
            end
        end


        function replay_component = build_replay_component(self, target_id, host_id, x_bar_j, x_hat_i_j, u_j, weights, use_local, predict_only)
            replay_component = struct();
            replay_component.direct = struct( ...
                'source', target_id, ...
                'state', x_bar_j, ...
                'weight', double(weights(1)), ...
                'enabled', logical(use_local || host_id == target_id));
            replay_component.neighbors = {};
            replay_component.prediction = struct( ...
                'control', u_j, ...
                'predict_only', logical(predict_only), ...
                'use_local', logical(use_local));

            neighbor_count = 0;
            for L = 2:length(weights)
                source_id = L - 1;
                if source_id == host_id || weights(L) <= 0
                    continue;
                end
                neighbor_count = neighbor_count + 1;
                replay_component.neighbors{neighbor_count} = struct( ...
                    'source', source_id, ...
                    'state', x_hat_i_j(:, source_id), ...
                    'weight', double(weights(L)));
            end
        end

        function weights = normalize_observer_weights(self, weights, host_id)
            weights = double(weights(:)');
            expected_len = self.num_vehicles + 1;
            if length(weights) < expected_len
                weights(end + 1:expected_len) = 0;
            elseif length(weights) > expected_len
                weights = weights(1:expected_len);
            end

            weights(~isfinite(weights)) = 0;
            weights = max(weights, 0);

            self_idx = host_id + 1;
            non_self = true(1, expected_len);
            non_self(self_idx) = false;
            external_sum = sum(weights(non_self));
            if external_sum > 1
                weights(non_self) = weights(non_self) / external_sum;
                external_sum = 1;
            end

            weights(self_idx) = max(0, 1 - external_sum);
            residual = 1 - sum(weights);
            if abs(residual) > 1e-12
                weights(self_idx) = max(0, weights(self_idx) + residual);
            end

            if sum(weights) <= eps
                weights = zeros(1, expected_len);
                weights(1) = 1;
            else
                weights = weights / sum(weights);
            end
        end

        function weights = cap_neighbor_influence_to_self(self, weights, host_id, total_cap)
            weights = double(weights(:)');
            neighbor_indices = 2:length(weights);
            neighbor_indices(neighbor_indices == host_id + 1) = [];
            neighbor_total = sum(weights(neighbor_indices));
            total_cap = max(0, min(1, total_cap));
            if neighbor_total > total_cap && neighbor_total > eps
                weights(neighbor_indices) = weights(neighbor_indices) * (total_cap / neighbor_total);
            end
        end

        function value = scenario_value(self, property_name, default_value)
            value = default_value;
            if isempty(self.vehicle) || isempty(self.vehicle.scenarios_config)
                return;
            end
            cfg = self.vehicle.scenarios_config;
            if isstruct(cfg) && isfield(cfg, property_name)
                value = cfg.(property_name);
            elseif isobject(cfg) && isprop(cfg, property_name)
                value = cfg.(property_name);
            end
        end

        function flag = get_trip_flag(self, vehicle_id, flag_name)
            flag = false;
            if isempty(self.vehicle.trip_models) || vehicle_id > length(self.vehicle.trip_models)
                return;
            end
            model = self.vehicle.trip_models{vehicle_id};
            candidates = {flag_name};
            if strcmp(flag_name, 'flag_target_attack')
                candidates{end + 1} = 'flag_taget_attk';
            elseif strcmp(flag_name, 'flag_global_est_check')
                candidates{end + 1} = 'flag_glob_est_check';
            end

            for idx = 1:length(candidates)
                prop_name = candidates{idx};
                if isstruct(model) && isfield(model, prop_name)
                    flag = logical(model.(prop_name));
                    return;
                elseif isobject(model) && isprop(model, prop_name)
                    flag = logical(model.(prop_name));
                    return;
                end
            end
        end

        function [is_ok, log_element, confidence] = check_elementwise_similarity(self, output1, output2, instant_index, vehicle_id)
            is_ok = true;
            log_element = [];
            confidence = 1; % Default confidence

            S = self.S_log_dist(:, :, vehicle_id);
            dynamic_tolerances = self.tolerances .* (1 + sqrt(diag(S))');

            % Compute per-vehicle confidence
            innovation = output1 - output2;
            confidence = exp(-0.5 * innovation' * pinv(S) * innovation / self.num_states);

            for i = 1:length(output1)
                diff_val = abs(output1(i) - output2(i));
                if diff_val > dynamic_tolerances(i) * self.reputation_scores(vehicle_id)
                    is_ok = false;
                    log_element = [log_element, i];
                end
            end
        end


        function u_j =  Get_controller(self,j)
            if self.vehicle.scenarios_config.predict_controller_type == "self"
                % If we are using local estimation, we need to use the local state of the vehicle
                % u_j = self.vehicle.other_vehicles(j).input; % Control input of the current vehicle
                u_j = self.vehicle.input; % Control input of the current vehicle
            elseif self.vehicle.scenarios_config.predict_controller_type == "true_other"
                % Go inside vehicle j , get control input of j
                u_j = self.vehicle.other_vehicles(j).input; % Control input of the current vehicle

            else % "predict_other"


                if (self.vehicle.vehicle_number ~= 1) % we are not lead vehicle
                    if (j==1) % if we are estimating the lead vehicle controller
                        u_j = [0;0]; % keep the lead vehicle's control input as zero
                    elseif j == self.vehicle.vehicle_number % if we are estimating our own vehicle
                        u_j = self.vehicle.input; % Control input of the current vehicle
                    else % if we are estimating another vehicle (Depending on the controller type)
                        % Calculate the control input for each vehicle locally by the estimated state of other vehicle
                        if(self.vehicle.scenarios_config.controller_type == "coop")
                            % If we are using cooperative control, we need to use the estimated state of the vehicle
                            est_local_j =  self.est_global_state_current(:,j); % get est_local_j in our host vehicle
                            [~, u_j ,~] = self.vehicle.controller2.get_optimal_input(j, est_local_j, [0;0], self.vehicle.other_vehicles(j).lane_id, 0, self.vehicle.initial_lane_id, self.vehicle.other_vehicles(j).direction_flag, "est", 0);
                        elseif (self.vehicle.scenarios_config.controller_type == "local")
                            est_local_j =  self.est_global_state_current(:,j); % get est_local_j in our host vehicle
                            [~, u_j ,~] = self.vehicle.controller.get_optimal_input(j, est_local_j, [0;0], self.vehicle.other_vehicles(j).lane_id, 0, self.vehicle.initial_lane_id, self.vehicle.other_vehicles(j).direction_flag, "est", 0);
                        else %"mix"
                            est_local_j =  self.est_global_state_current(:,j); % get est_local_j in our host vehicle
                            [~, u_1_predict ,~] = self.vehicle.controller1.get_optimal_input(j, est_local_j, [0;0], self.vehicle.other_vehicles(j).lane_id, 0, self.vehicle.initial_lane_id, self.vehicle.other_vehicles(j).direction_flag, "est", 0);
                            [~, u_2_predict ,~] = self.vehicle.controller2.get_optimal_input(j, est_local_j, [0;0], self.vehicle.other_vehicles(j).lane_id, 0, self.vehicle.initial_lane_id, self.vehicle.other_vehicles(j).direction_flag, "est", 0);
                            U_target = [0,0];

                            U_target(1) = (1 - self.vehicle.gamma)*u_1_predict(1) + self.vehicle.gamma*u_2_predict(1);
                            u_j = u_1_predict;
                            u_j(1) = self.u_j_last_predict(1) + self.vehicle.Param_opt.tau_filter*(U_target(1) - self.u_j_last_predict(1)) ;
                            % update the last predicted input ()
                            % TODO : First impression , look not good , beacause they delay not change quickly
                            self.u_j_last_predict = u_j;
                        end
                    end
                else % If we are lead vehicle, we are estimating our own vehicle , So use directly the input
                    u_j = self.vehicle.input;
                end


            end
        end

        function Local_observer(self , mesure_state , instant_index)
            % self.est_local_state_current = state;
            [A , B]  = self.matrix();

            % % Add small measurement noise for more realistic simulation
            % small_noise_cov = diag([0.01, 0.01, 0.001, 0.005, 0.001]); % Small noise for [X, Y, theta, V, A]
            % small_measurement_noise = mvnrnd(zeros(self.num_states,1), small_noise_cov)'; 
            % mesure_state = mesure_state + small_measurement_noise;

            if self.vehicle.scenarios_config.Is_noise_mesurement == true
                if rand() < self.vehicle.scenarios_config.noise_probability
                    if self.vehicle.scenarios_config.Use_smooth_filter == true
                        % Apply smoother, correlated measurement noise
                        % Generate correlated noise using first-order Markov process
                        correlation_factor = self.measurement_noise_correlation; % 0 = white noise, 1 = fully correlated
                        white_noise = self.sample_gaussian(self.R);
                        
                        % First-order Markov noise model: x[k] = correlation_factor * x[k-1] + sqrt(1-correlation_factor^2) * w[k]
                        self.correlated_noise_state = correlation_factor * self.correlated_noise_state + ...
                                                      sqrt(1 - correlation_factor^2) * white_noise;
                        
                        % Apply exponential smoothing to reduce sudden noise spikes
                        current_noise = (1 - self.noise_filter_alpha) * self.correlated_noise_state + ...
                                        self.noise_filter_alpha * self.previous_noise;
                        
                        mesure_state = mesure_state + current_noise;
                        self.previous_noise = current_noise; % Store for next iteration
                    else
                        % Original simple noise generation (no smoothing)
                        additional_measurement_noise = self.sample_gaussian(self.R);  % sample additional measurement noise
                        mesure_state = mesure_state + additional_measurement_noise; % Add additional noise to the measurement
                    end
                end
            end

            if self.vehicle.scenarios_config.Local_observer_type == "observer"
                raw_output = A * self.est_local_state_current + B * self.vehicle.input + self.L_gain * (mesure_state - self.est_local_state_current);
            elseif self.vehicle.scenarios_config.Local_observer_type == "kalman"
                % Kalman Filter
                self.kalman_filter(mesure_state);
                raw_output = self.est_local_state_current;
            else %"true"
                raw_output = mesure_state;
            end
            
            % Apply output smoothing conditionally based on configuration
            if self.vehicle.scenarios_config.Use_smooth_filter_in_local_observer == true
                % Apply exponential smoothing to reduce observer output oscillations
                if instant_index == 1
                    % For first iteration, use raw output
                    self.est_local_state_current = raw_output;
                else
                    % Apply low-pass filter: output = (1-alpha) * new_value + alpha * previous_value
                    self.est_local_state_current = (1 - self.output_filter_alpha) * raw_output + ...
                                                   self.output_filter_alpha * self.previous_local_output;
                end
                
                % Store current output for next iteration
                self.previous_local_output = self.est_local_state_current;
            else
                % No smoothing - use raw output directly
                self.est_local_state_current = raw_output;
            end
            
            self.est_local_state_log(:, instant_index+1) = self.est_local_state_current;


        end

        function kalman_filter(self, mesure_state)
            % Kalman Filter
            [A , B]  = self.matrix();

            % Add process noise to make it more realistic
            if self.vehicle.scenarios_config.Is_noise_mesurement == true
                process_noise = self.sample_gaussian(self.Q);  % sample process noise
            else
                process_noise = zeros(self.num_states,1); % No process noise for perfect case
            end

            % Prediction Step (with process noise)
            x_pred = A * self.est_local_state_current + B * self.vehicle.input + process_noise;
            P_pred = A * self.P * A' + self.Q;

            % Measurement Update Step
            y = mesure_state; % Measurement
            % More realistic measurement matrix - not all states may be directly measurable
            % For now, assume we can measure position, angle, and velocity but not acceleration directly
            C = eye(self.num_states); 
            % Make acceleration measurement less reliable
            % C(5,5) = 0.5; % Acceleration is harder to measure accurately
            
            S = C * P_pred * C' + self.R; % Innovation covariance (add measurement noise)
            K = P_pred * C' / (S + 1e-8 * eye(self.num_states)); % Kalman gain with regularization

            % Update the state estimate
            innovation = y - C * x_pred;
            self.est_local_state_current = x_pred + K * innovation;
            
            % Update the error covariance (Joseph form for numerical stability)
            I_KC = eye(self.num_states) - K * C;
            self.P = I_KC * P_pred * I_KC' + K * self.R * K';
            
            % Ensure P remains positive definite and well-conditioned
            self.P = (self.P + self.P') / 2; % Force symmetry
            [V, D] = eig(self.P);
            D = max(D, 1e-8 * eye(self.num_states)); % Ensure positive eigenvalues
            self.P = V * D * V';
        end

        function covariance = config_covariance(self, value, property_name)
            if ~isnumeric(value) || isempty(value)
                error('Scenarios_config.%s must be a numeric covariance vector or matrix.', property_name);
            end

            if isvector(value)
                if numel(value) ~= self.num_states
                    error('Scenarios_config.%s must have %d entries for [x, y, theta, v, a].', property_name, self.num_states);
                end
                covariance = diag(double(value(:)));
            else
                if ~isequal(size(value), [self.num_states, self.num_states])
                    error('Scenarios_config.%s must be a %dx%d covariance matrix.', property_name, self.num_states, self.num_states);
                end
                covariance = double(value);
            end

            covariance = (covariance + covariance') / 2;
            if any(~isfinite(covariance(:)))
                error('Scenarios_config.%s must contain only finite values.', property_name);
            end
        end

        function value = config_unit_interval(~, value, property_name)
            if ~isnumeric(value) || ~isscalar(value) || isnan(value) || value < 0 || value > 1
                error('Scenarios_config.%s must be a scalar in [0, 1].', property_name);
            end
            value = double(value);
        end

        function sample = sample_gaussian(~, covariance)
            covariance = (covariance + covariance') / 2;
            n = size(covariance, 1);
            [factor, p] = chol(covariance + 1e-10 * eye(n), 'lower');

            if p ~= 0
                [V, D] = eig(covariance);
                eig_values = max(diag(D), 0);
                factor = V * diag(sqrt(eig_values));
            end

            sample = factor * randn(n, 1);
        end

        % This model use in obesrver , so need to be here, in the observer class
        function  [A , B] = matrix(self, vehicle_state)
            if nargin < 2
                % Default: use current vehicle's state
                theta = self.vehicle.state(3);
                v = self.vehicle.state(4);
            else
                % Use provided vehicle state (for estimating other vehicles)
                theta = vehicle_state(3);
                v = vehicle_state(4);
            end
            
            Ts = self.param_sys.dt;
            tau = self.param_sys.tau;

            % state = X ,Y , theta , V , A

            if self.vehicle.scenarios_config.model_vehicle_type == "normal"
                % Here in discrete time
                A = [   1 0 -v*sin(theta)*Ts cos(theta)*Ts 0;
                    0 1  v*cos(theta)*Ts sin(theta)*Ts 0;
                    0 0 1 0 0 ;
                    0 0 0 1 0 ;
                    0 0 0 0 0];

                B = [0 0;
                    0 0;
                    0 Ts;
                    Ts 0;
                    0 0];

            elseif (self.vehicle.scenarios_config.model_vehicle_type == "delay_v")
                % Here in discrete time
                A = [   1 0 -v*sin(theta)*Ts cos(theta)*Ts 0;
                    0 1 v*cos(theta)*Ts sin(theta)*Ts 0;
                    0 0 1 0 0 ;
                    0 0 0 1 Ts;
                    0 0 0 0 1 - tau/Ts];

                B = [0 0;
                    0 0;
                    0 Ts;
                    Ts/tau 0;
                    0 0];

            elseif (self.vehicle.scenarios_config.model_vehicle_type == "delay_a")% "delay_a"
                A = [   1 0 -v*sin(theta)*Ts cos(theta)*Ts 0;
                    0 1 v*cos(theta)*Ts sin(theta)*Ts 0;
                    0 0 1 0 0;
                    0 0 0 1 Ts;
                    0 0 0 0 1 - Ts/tau];


                B = [0 0;
                    0 0;
                    0 Ts;
                    0 0;
                    Ts/tau 0];
            else %" Model In the paper"
                Ai_conti = [0 0 0 1 0;
                    0 0 0 0 0;
                    0 0 0 0 0;
                    0 0 0 0 1;
                    0 0 0 0 -1/tau];
                Bi_conti = [0 0;
                    0 0;
                    0 0;
                    0 0;
                    1/tau 0];

                % Descritization of the c-state space model
                A = (eye(length(Ai_conti))+Ts*Ai_conti);
                B = Ts*Bi_conti;
            end
        end




        function [output, innovation, S] = predict_kalman_dist(self, host_id, j, x_hat_i_j_host, u_j)
            [A, B] = self.matrix();
            C = eye(self.num_states);

            x_pred = A * x_hat_i_j_host + B * u_j;
            self.P_pred_dist = A * self.P_pred_dist * A' + self.Q;

            innovation = x_hat_i_j_host - C * x_pred;
            S = C * self.P_pred_dist * C' + self.R;

            output = x_pred;
        end


        function update_reputation(self, j, output, x_bar_j)
            error = norm(output - x_bar_j);
            agreement = exp(-error^2 / (2 * self.num_states));
            alpha = 0.1;
            self.reputation_scores(j) = self.reputation_scores(j) + alpha * (agreement - self.reputation_scores(j));
        end

        % ----------- Convenient helper methods for prediction-only mode
        function is_pred_mode = is_vehicle_in_prediction_mode(self, vehicle_id)
            % Check if a specific vehicle is in prediction-only mode
            if vehicle_id <= length(self.is_in_prediction_mode)
                is_pred_mode = self.is_in_prediction_mode(vehicle_id);
            else
                is_pred_mode = false;
            end
        end
        
        function any_pred_mode = any_vehicle_in_prediction_mode(self)
            % Check if any vehicle is in prediction-only mode
            any_pred_mode = any(self.is_in_prediction_mode);
        end
        
        function pred_vehicles = get_prediction_mode_vehicles(self)
            % Get list of vehicle IDs currently in prediction-only mode
            pred_vehicles = find(self.is_in_prediction_mode);
        end

        % -----------End Convenient helper methods for prediction-only mode

        %% Contamination Rollback Functions
        
        function add_to_rollback_buffer(self, step_data)
            % Add step data to the circular rollback buffer
            if ~self.rollback_enabled
                return;
            end
            
            self.rollback_buffer{self.rollback_buffer_index} = step_data;
            self.rollback_buffer_index = self.rollback_buffer_index + 1;
            
            % Handle circular buffer wraparound
            if self.rollback_buffer_index > self.rollback_window_size
                self.rollback_buffer_index = 1;
                self.rollback_buffer_full = true;
            end
        end
        
        function buffer_size = get_buffer_size(self)
            % Get the current effective size of the rollback buffer
            if self.rollback_buffer_full
                buffer_size = self.rollback_window_size;
            else
                buffer_size = self.rollback_buffer_index - 1;
            end
        end
        
        function step_data = get_buffer_step(self, relative_index)
            % Get step data from buffer using relative indexing (1 = oldest, buffer_size = newest)
            buffer_size = self.get_buffer_size();
            if relative_index < 1 || relative_index > buffer_size
                error('Buffer index out of range: %d (size: %d)', relative_index, buffer_size);
            end
            
            if self.rollback_buffer_full
                % Calculate actual index in circular buffer
                actual_index = mod(self.rollback_buffer_index - 1 + relative_index - 1, self.rollback_window_size) + 1;
            else
                actual_index = relative_index;
            end
            
            step_data = self.rollback_buffer{actual_index};
        end
        
        function clear_rollback_buffer(self)
            % Clear the rollback buffer and reset indices
            if ~self.rollback_enabled
                return;
            end
            
            self.rollback_buffer = cell(self.rollback_window_size, 1);
            self.rollback_buffer_index = 1;
            self.rollback_buffer_full = false;
            fprintf('[Observer V%d] Rollback buffer cleared\n', self.vehicle.vehicle_number);
        end
        
        function create_step_data(self, instant_index, pre_update_states, weights_used, trust_scores, target_components)
            % Create a replayable step data structure for the rollback buffer
            if nargin < 6
                target_components = cell(self.num_vehicles, 1);
            end
            step_data = struct();
            step_data.instant_index = instant_index;
            step_data.pre_update_states = pre_update_states; % [num_states x num_vehicles]
            step_data.weights_used = weights_used; % Cell array: {vehicle_j} = weights vector
            step_data.trust_scores = trust_scores; % [1 x num_vehicles]
            step_data.target_components = target_components; % Cell array of replayable source-state terms
            
            % Store prediction-only mode flags for proper rollback handling
            step_data.is_predict_only_mode = self.is_in_prediction_mode; % [1 x num_vehicles] boolean array
            
            % Store flags for virtual node 0 exclusion during rollback
            step_data.exclude_virtual_node0_flags = false(1, self.num_vehicles); % [1 x num_vehicles]
            host_id = self.vehicle.vehicle_number;
            for j = 1:self.num_vehicles
                if abs(host_id - j) == 1 && self.get_trip_flag(j, 'flag_local_est_check')
                    step_data.exclude_virtual_node0_flags(j) = true;
                end
            end
            
            self.add_to_rollback_buffer(step_data);
        end
        
        function [rollback_applied, corrected_states] = check_and_trigger_rollback(self, instant_index, trust_scores, current_states)
            % Check for newly malicious vehicles and trigger rollback if needed
            rollback_applied = false;
            corrected_states = [];
            if ~self.rollback_enabled
                return;
            end
            if nargin < 4 || isempty(current_states)
                current_states = self.est_global_state_current;
            end
            
            previous_malicious = self.malicious_vehicles;
            active_malicious = [];
            
            for vehicle_id = 1:length(trust_scores)
                current_trust = trust_scores(vehicle_id);
                if vehicle_id == self.vehicle.vehicle_number
                    continue;
                end

                should_flag = false;
                if self.rollback_on_final_trust && isfinite(current_trust) && current_trust < self.trust_threshold
                    should_flag = true;
                end
                if self.rollback_on_local_est_check && self.get_trip_flag(vehicle_id, 'flag_local_est_check')
                    should_flag = true;
                end
                if self.rollback_on_global_est_check && self.get_trip_flag(vehicle_id, 'flag_global_est_check')
                    should_flag = true;
                end

                if should_flag
                    self.rollback_bad_counters(vehicle_id) = self.rollback_bad_counters(vehicle_id) + 1;
                    is_confirmed_bad = self.rollback_bad_counters(vehicle_id) >= self.rollback_required_bad_steps;
                    if is_confirmed_bad || ismember(vehicle_id, previous_malicious)
                        active_malicious = [active_malicious, vehicle_id]; %#ok<AGROW>
                    end
                    self.rollback_recovery_counters(vehicle_id) = 0;
                elseif ismember(vehicle_id, previous_malicious)
                    self.rollback_bad_counters(vehicle_id) = 0;
                    self.rollback_recovery_counters(vehicle_id) = self.rollback_recovery_counters(vehicle_id) + 1;
                    if self.rollback_recovery_counters(vehicle_id) < self.rollback_recovery_good_steps
                        active_malicious = [active_malicious, vehicle_id]; %#ok<AGROW>
                    else
                        self.rollback_recovery_counters(vehicle_id) = 0;
                    end
                else
                    self.rollback_bad_counters(vehicle_id) = 0;
                    self.rollback_recovery_counters(vehicle_id) = 0;
                end
            end

            active_malicious = unique(active_malicious);
            newly_malicious = setdiff(active_malicious, previous_malicious);
            self.malicious_vehicles = active_malicious;
            
            if ~isempty(newly_malicious)
                if instant_index - self.rollback_last_trigger_step < self.rollback_cooldown_steps
                    return;
                end
                [rollback_applied, corrected_states] = self.trigger_contamination_rollback(active_malicious, instant_index, current_states);
                if rollback_applied
                    self.rollback_last_trigger_step = instant_index;
                end
            end
        end
        
        function [rollback_applied, corrected_states] = trigger_contamination_rollback(self, malicious_vehicle_ids, current_time, current_states)
            % Trigger rollback to remove contamination from active malicious vehicles
            rollback_applied = false;
            if nargin < 4 || isempty(current_states)
                current_states = self.est_global_state_current;
            end
            corrected_states = current_states;
            malicious_vehicle_ids = unique(malicious_vehicle_ids(:)');
            buffer_size = self.get_buffer_size();
            if buffer_size == 0
                fprintf('[Observer V%d] No rollback data available for vehicles [%s]\n', ...
                    self.vehicle.vehicle_number, num2str(malicious_vehicle_ids));
                return;
            end
            
            % Determine rollback window
            rollback_steps = min(self.rollback_window_size, buffer_size);
            rollback_start_buffer_index = max(1, buffer_size - rollback_steps + 1);
            oldest_available_step = self.get_buffer_step(rollback_start_buffer_index);
            newest_available_step = self.get_buffer_step(buffer_size);
            rollback_start_step = oldest_available_step.instant_index;
            rollback_end_time = newest_available_step.instant_index;
            
            fprintf('[Observer V%d] Starting contamination rollback for vehicles [%s] from time %d to %d (%d steps)\n', ...
                   self.vehicle.vehicle_number, num2str(malicious_vehicle_ids), rollback_start_step, rollback_end_time, rollback_steps);
            
            corrected_states = oldest_available_step.pre_update_states;
            current_self_state = current_states(:, self.vehicle.vehicle_number);
            replay_start_time_by_vehicle = nan(1, self.num_vehicles);

            for idx = 1:length(malicious_vehicle_ids)
                target_id = malicious_vehicle_ids(idx);
                if target_id < 1 || target_id > self.num_vehicles
                    continue;
                end
                [trusted_state, trusted_time, has_trusted_state] = self.get_rollback_trusted_state_entry(target_id);
                if has_trusted_state
                    corrected_states(:, target_id) = trusted_state;
                    replay_start_time_by_vehicle(target_id) = trusted_time;
                end
            end
            
            for step_idx = rollback_start_buffer_index:buffer_size
                step_data = self.get_buffer_step(step_idx);
                step_time = step_data.instant_index;
                
                for j = 1:self.num_vehicles
                    if j == self.vehicle.vehicle_number
                        continue;
                    end
                    if ~isnan(replay_start_time_by_vehicle(j)) && step_time <= replay_start_time_by_vehicle(j)
                        continue;
                    end
                    corrected_states(:, j) = self.replay_step_without_malicious(...
                        step_data, j, malicious_vehicle_ids, corrected_states(:, j));
                end
                
                if self.rollback_rewrite_history_log && step_time > 0 && step_time <= size(self.est_global_state_log, 2)
                    for v = 1:self.num_vehicles
                        self.est_global_state_log(:, step_time, v) = corrected_states(:, v);
                    end
                end
            end

            corrected_states(:, self.vehicle.vehicle_number) = current_self_state;
            
            % Update current state estimates
            self.est_global_state_current = corrected_states;
            rollback_applied = true;
            
            % Fix 4: Clear/mark buffer and increment epoch after rollback
            self.clear_rollback_buffer();
            
            % Update rollback statistics
            self.rollback_stats.total_rollbacks = self.rollback_stats.total_rollbacks + 1;
            self.rollback_stats.vehicles_flagged = unique([self.rollback_stats.vehicles_flagged, malicious_vehicle_ids]);
            self.rollback_stats.rollback_times = [self.rollback_stats.rollback_times, current_time];
            
            fprintf('[Observer V%d] Contamination rollback completed for vehicles [%s], buffer cleared\n', ...
                self.vehicle.vehicle_number, num2str(malicious_vehicle_ids));
        end
        
        function corrected_state = replay_step_without_malicious(self, step_data, vehicle_j, malicious_vehicle_ids, previous_state)
            % Replay one target update while excluding active malicious sources.
            base_state = previous_state;
            [A, ~] = self.matrix(base_state);

            if isfield(step_data, 'target_components') && length(step_data.target_components) >= vehicle_j && ...
                    ~isempty(step_data.target_components{vehicle_j})
                component = step_data.target_components{vehicle_j};
                correction = zeros(self.num_states, 1);
                prediction = component.prediction;
                predict_only = isfield(prediction, 'predict_only') && prediction.predict_only;

                direct = component.direct;
                use_direct = direct.enabled && direct.weight > 0 && ~ismember(direct.source, malicious_vehicle_ids);
                if use_direct && (~predict_only || self.vehicle.vehicle_number == vehicle_j)
                    correction = correction + direct.weight * (direct.state - base_state);
                end

                if ~predict_only
                    for neighbor_idx = 1:length(component.neighbors)
                        neighbor_data = component.neighbors{neighbor_idx};
                        if ismember(neighbor_data.source, malicious_vehicle_ids)
                            continue;
                        end
                        correction = correction + neighbor_data.weight * (neighbor_data.state - base_state);
                    end
                end

                control = prediction.control;
                [A, B] = self.matrix(base_state);
                corrected_state = A * (base_state + correction) + B * control;
                corrected_state = self.apply_replay_state_constraints(corrected_state);
                return;
            end

            % Backward-compatible fallback for older buffered delta payloads.
            consensus_delta = zeros(self.num_states, 1);
            if isfield(step_data, 'neighbor_deltas') && ~isempty(step_data.neighbor_deltas{vehicle_j})
                for neighbor_idx = 1:length(step_data.neighbor_deltas{vehicle_j})
                    neighbor_data = step_data.neighbor_deltas{vehicle_j}{neighbor_idx};
                    if ~ismember(neighbor_data.vehicle_idx, malicious_vehicle_ids)
                        consensus_delta = consensus_delta + neighbor_data.delta;
                    end
                end
            end
            anchor_delta = zeros(self.num_states, 1);
            if isfield(step_data, 'anchor_deltas')
                anchor_delta = step_data.anchor_deltas(:, vehicle_j);
            end
            input_delta = zeros(self.num_states, 1);
            if isfield(step_data, 'input_deltas')
                input_delta = step_data.input_deltas(:, vehicle_j);
            end
            corrected_state = A * base_state + consensus_delta + anchor_delta + input_delta;
            corrected_state = self.apply_replay_state_constraints(corrected_state);
        end

        function update_rollback_trusted_state_history(self, instant_index, trust_scores, states)
            if ~self.rollback_enabled
                return;
            end

            for vehicle_id = 1:min(length(trust_scores), self.num_vehicles)
                if vehicle_id == self.vehicle.vehicle_number
                    continue;
                end
                if trust_scores(vehicle_id) < self.trust_threshold
                    continue;
                end
                if self.get_trip_flag(vehicle_id, 'flag_local_est_check') || self.get_trip_flag(vehicle_id, 'flag_global_est_check')
                    continue;
                end

                entry = struct('state', states(:, vehicle_id), 'instant_index', instant_index);
                history = self.rollback_trusted_state_history{vehicle_id};
                if isempty(history)
                    history = entry;
                else
                    history(end + 1) = entry; %#ok<AGROW>
                end
                if length(history) > self.rollback_trusted_state_history_size
                    history = history(end - self.rollback_trusted_state_history_size + 1:end);
                end
                self.rollback_trusted_state_history{vehicle_id} = history;
            end
        end

        function [trusted_state, trusted_time, has_trusted_state] = get_rollback_trusted_state_entry(self, target_id)
            trusted_state = [];
            trusted_time = NaN;
            has_trusted_state = false;
            if target_id < 1 || target_id > length(self.rollback_trusted_state_history)
                return;
            end

            history = self.rollback_trusted_state_history{target_id};
            if isempty(history)
                return;
            end

            guard = min(self.rollback_trusted_state_guard_steps, length(history) - 1);
            entry = history(end - guard);
            trusted_state = entry.state;
            trusted_time = entry.instant_index;
            has_trusted_state = true;
        end

        function state = apply_replay_state_constraints(self, state)
            state(3) = atan2(sin(state(3)), cos(state(3)));
            if isstruct(self.param_sys) && isfield(self.param_sys, 'max_acceleration') && isfield(self.param_sys, 'min_acceleration')
                state(5) = max(self.param_sys.min_acceleration, min(self.param_sys.max_acceleration, state(5)));
            elseif isobject(self.param_sys) && isprop(self.param_sys, 'max_acceleration') && isprop(self.param_sys, 'min_acceleration')
                state(5) = max(self.param_sys.min_acceleration, min(self.param_sys.max_acceleration, state(5)));
            end
        end
        
        function print_rollback_statistics(self)
            % Print statistics about rollback operations
            fprintf('\n=== Contamination Rollback Statistics ===\n');
            fprintf('Total rollbacks performed: %d\n', self.rollback_stats.total_rollbacks);
            fprintf('Vehicles flagged as malicious: [%s]\n', num2str(self.rollback_stats.vehicles_flagged));
            fprintf('Rollback times: [%s]\n', num2str(self.rollback_stats.rollback_times));
            fprintf('Currently malicious vehicles: [%s]\n', num2str(self.malicious_vehicles));
            fprintf('Trust threshold: %.3f\n', self.trust_threshold);
            fprintf('Rollback window size: %d\n', self.rollback_window_size);
            fprintf('Rollback enabled: %s\n', mat2str(self.rollback_enabled));
            fprintf('========================================\n\n');
        end




        %%% ------------ Plotting Functions
        function plot_global_state_log(self)
            % Ensure est_global_state_log is not empty
            if isempty(self.est_global_state_log)
                error('est_global_state_log is empty. No data to plot.');
            end

            % num_states = 4; % Assuming X, Y, theta, and V as states
            % num_states = size(self.est_global_state_log, 1); % Number of states

            state_labels = {'Position X', 'Position Y', 'Theta', 'Velocity','Acc'};

            % Create time vector for plotting
            num_time_steps = size(self.est_global_state_log, 2) - 1;
            dt = self.vehicle.scenarios_config.dt;
            time_vector = 0:dt:(num_time_steps-1)*dt;

            figure("Name", "Global Position Estimates " + num2str(self.vehicle.vehicle_number), "NumberTitle", "off");
            % title(['Global Position Estimates ' num2str(self.vehicle.vehicle_number)]);

            for state_idx = 1:self.num_states
                subplot(self.num_states, 1, state_idx);
                hold on;
                for v = 1:self.num_vehicles
                    plot(time_vector, squeeze(self.est_global_state_log(state_idx, 1:end-1, v)), 'DisplayName', ['Vehicle ', num2str(v)]);
                end
                % Add vertical lines for observer mode switches (prediction-only <-> normal)
                if ~isempty(self.Is_ok_log)
                    % Assume Is_ok_log is a vector of booleans for each vehicle and time step
                    % Try to reshape to [num_vehicles, time_steps] if possible
                    total_steps = size(self.est_global_state_log, 2)-1;
                    try
                        is_ok_mat = reshape(self.Is_ok_log, [self.num_vehicles, total_steps]);
                    catch
                        is_ok_mat = [];
                    end
                    for v = 1:self.num_vehicles
                        if ~isempty(is_ok_mat)
                            mode_switches = find(diff(is_ok_mat(v,:)) ~= 0) + 1; % indices where mode changes
                            for ms = mode_switches
                                xline(time_vector(ms), '--r', 'LineWidth', 1.2, 'Alpha', 0.5);
                            end
                        end
                    end
                end
                title(state_labels{state_idx});
                xlabel('Time (s)');
                ylabel(state_labels{state_idx});
                legend;
                grid on;
            end
        end

        function plot_error_local_estimated(self)

            state_labels = {'Position X', 'Position Y', 'Theta', 'Velocity','Acc'};
            % num_states = length(self.est_local_state_current);
            
            % Create time vector for plotting
            num_time_steps = size(self.est_local_state_log, 2) - 1;
            dt = self.vehicle.scenarios_config.dt;
            time_vector = 0:dt:(num_time_steps-1)*dt;
            
            % Create a figure for the plot
            figure("Name", "Local error " + num2str(self.vehicle.vehicle_number), "NumberTitle", "off");
            hold on;

            for state_idx = 1:self.num_states
                subplot(self.num_states, 1, state_idx);
                hold on;
                plot(time_vector, self.est_local_state_log(state_idx, 1:end-1) - self.vehicle.state_log(state_idx, 1:end-1));
                % plot( self.vehicle.state_log(state_idx, 1:end-1));
                % plot(self.est_local_state_log(state_idx, :) );

                xlabel('Time (s)');
                ylabel(state_labels{state_idx});
                legend;
                grid on;
            end
        end

        function [global_dist_err, global_theta_err, global_vel_err,global_acc_err] = calculate_global_errors(self, time_window)
            % Extract actual and estimated states for error calculation
            estimated_states = self.est_global_state_log; % size: [num_states, num_time_steps, num_vehicles]
            % actual_states = self.vehicle.state_log;      % size: [num_states, num_time_steps]

            num_time_steps = size(estimated_states, 2);
            if nargin < 2 || isempty(time_window)
                time_indices = 1:num_time_steps;
            else
                if numel(time_window) ~= 2 || time_window(2) < time_window(1)
                    error('time_window must be [start_time, end_time] with end_time >= start_time.');
                end

                dt = self.vehicle.scenarios_config.dt;
                start_idx = max(1, ceil(time_window(1) / dt));
                end_idx = min(num_time_steps, floor(time_window(2) / dt));

                if start_idx > end_idx
                    error('Requested RMSE time window [%.3f, %.3f] is outside the available simulation log.', ...
                        time_window(1), time_window(2));
                end

                time_indices = start_idx:end_idx;
            end

            actual_states_all = zeros(self.num_states, num_time_steps+1, self.num_vehicles);
            for v = 1:self.num_vehicles
                actual_states_all(:,:,v) = self.vehicle.other_vehicles(v).state_log; % [num_states, num_time_steps]
            end

            % Initialize error matrices
            num_selected_steps = numel(time_indices);
            dist_err = zeros(self.num_vehicles, num_selected_steps);
            theta_err = zeros(self.num_vehicles, num_selected_steps);
            vel_err = zeros(self.num_vehicles, num_selected_steps);
            acc_err = zeros(self.num_vehicles, num_selected_steps);


            % Calculate errors for each vehicle at each time step
            for v = 1:self.num_vehicles
                for t_idx = 1:num_selected_steps
                    t = time_indices(t_idx);
                    est = estimated_states(:, t, v); % Estimated state for vehicle v at time t
                    act = actual_states_all(:, t+1, v); % Actual state for vehicle v at time t

                    % Distance error - Root Mean Square Error (RMSE)
                    dist_err(v, t_idx) = (est(1) - act(1))^2;

                    % Theta error (orientation) - RMSE
                    theta_err(v, t_idx) = (est(3) - act(3))^2;

                    % Velocity error - RMSE
                    vel_err(v, t_idx) = (est(4) - act(4))^2;

                    % Acceleration error - RMSE
                    acc_err(v, t_idx) = (est(5) - act(5))^2;
                end
            end

            % Compute RMSE over time for each vehicle
            global_dist_err = sqrt(mean(dist_err, 2));  % [num_vehicles, 1] - RMSE
            global_theta_err = sqrt(mean(theta_err, 2)); % [num_vehicles, 1] - RMSE
            global_vel_err = sqrt(mean(vel_err, 2));    % [num_vehicles, 1] - RMSE
            global_acc_err = sqrt(mean(acc_err, 2));    % [num_vehicles, 1] - RMSE

        end

        % function smoothed_output = apply_output_smoothing(self, raw_output, vehicle_idx, use_adaptive)
        %     % Apply smoothing to distributed observer output
        %     % raw_output: new estimate from distributed observer
        %     % vehicle_idx: which vehicle this estimate is for
        %     % use_adaptive: whether to use adaptive smoothing based on confidence
            
        %     if nargin < 4
        %         use_adaptive = false;
        %     end
            
        %     persistent previous_outputs; % Store previous outputs for each vehicle
        %     if isempty(previous_outputs)
        %         previous_outputs = [];
        %     end
            
        %     % Initialize if first call for this vehicle
        %     if size(previous_outputs, 2) < vehicle_idx || isempty(previous_outputs)
        %         if isempty(previous_outputs)
        %             previous_outputs = raw_output;
        %         else
        %             previous_outputs(:, vehicle_idx) = raw_output;
        %         end
        %         smoothed_output = raw_output;
        %         return;
        %     end
            
        %     % Get smoothing factor
        %     if use_adaptive && vehicle_idx <= length(self.reputation_scores)
        %         % Adaptive smoothing based on reputation score
        %         base_alpha = 0.3;
        %         reputation_factor = max(0.1, self.reputation_scores(vehicle_idx));
        %         alpha = base_alpha * reputation_factor;
        %     else
        %         alpha = 0.3; % Fixed smoothing factor
        %     end
            
        %     % Apply exponential smoothing
        %     smoothed_output = (1 - alpha) * raw_output + alpha * previous_outputs(:, vehicle_idx);
            
        %     % Update stored value
        %     previous_outputs(:, vehicle_idx) = smoothed_output;
        % end

        % function filtered_noise = generate_smooth_noise(self, base_covariance, correlation_factor)
        %     % Generate smooth, correlated noise for more realistic simulation
        %     % base_covariance: base noise covariance matrix
        %     % correlation_factor: temporal correlation (0-1, higher = more correlated)
            
        %     if nargin < 3
        %         correlation_factor = 0.8;
        %     end
            
        %     % Generate white noise
        %     white_noise = mvnrnd(zeros(self.num_states, 1), base_covariance)';
            
        %     % Apply first-order Markov correlation
        %     self.correlated_noise_state = correlation_factor * self.correlated_noise_state + ...
        %                                   sqrt(1 - correlation_factor^2) * white_noise;
            
        %     % Apply additional smoothing
        %     filtered_noise = (1 - self.noise_filter_alpha) * self.correlated_noise_state + ...
        %                      self.noise_filter_alpha * self.previous_noise;
            
        %     % Update previous noise for next iteration
        %     self.previous_noise = filtered_noise;
        % end



    end
end
