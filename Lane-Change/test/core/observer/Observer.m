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

        % Properties for distributed observer / Controller
        self_belief; % Self-belief for distributed observer prediction
        reputation_scores; % Reputation scores for neighbors [num_vehicles x 1]
        P_pred_dist; % Predicted error covariance for distributed observer
        S_log_dist; % Innovation covariance log for distributed observer
        self_belief_log; % Log for self-belief
        u_j_last_predict = [0; 0]; % Last predicted control input for vehicle j

        predict_only_counter = []; % Counter for consecutive good steps in prediction-only mode
        predict_only_timer =   []; % Timer for prediction-only mode
        is_in_prediction_mode = []; % Boolean array tracking prediction-only mode for each vehicle
        
        % Properties for smoother noise and observer output
        noise_filter_alpha = 0.7; % Exponential smoothing parameter for noise (0.0 = no filtering, 1.0 = max filtering)
        previous_noise = []; % Store previous noise for smoothing
        output_filter_alpha = 0.3; % Low-pass filter parameter for observer output
        previous_local_output = []; % Store previous local observer output for smoothing
        correlated_noise_state = []; % State for correlated noise generation
    end
    methods
        function self = Observer( vehicle , veh_param, inital_global_state, inital_local_state)
            self.vehicle = vehicle;
            self.param_sys = veh_param; % Set the vehicle system parameters

            self.est_local_state_current = inital_local_state;

            self.est_global_state_current = inital_global_state;

            self.num_states = 5 ; % X , Y , theta , V,A
            Nt = self.vehicle.total_time_step;
            num_vehicles = length(self.vehicle.other_vehicles);

            self.est_global_state_log = zeros(self.num_states, Nt, num_vehicles);
            % Initialize the log with proper 3D indexing
            for v = 1:num_vehicles
                self.est_global_state_log(:, 1, v) = inital_global_state(:, v);
            end

            self.est_local_state_log = zeros(self.num_states, Nt);
            self.est_local_state_log(:,1) = inital_local_state;

            self.tolerances = [4, 2, deg2rad(8), 2 , 1]; % Tolerances for each state [X, Y, theta, V, A]



            self.predict_only_counter = zeros(num_vehicles, 1);
            self.predict_only_timer = zeros(num_vehicles, 1);
            self.is_in_prediction_mode = false(num_vehicles, 1); % Initialize as boolean array
            % NEW : for calculate the self belief = gamma in the controller
            self.self_belief = 1; % Initial self-belief
            self.reputation_scores = ones(num_vehicles, 1); % Initial reputation = 1
            self.P_pred_dist = eye(self.num_states); % Initial covariance
            self.S_log_dist = zeros(self.num_states, self.num_states, Nt); % Log innovation covariance
            
            % Initialize smoothing properties
            self.previous_noise = zeros(self.num_states, 1);
            self.previous_local_output = inital_local_state;
            self.correlated_noise_state = zeros(self.num_states, 1);


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
                % These are example values; adjust based on your system/sensor characteristics.
                % Measurement noise covariance R (variances on sensor measurements)
                self.R = diag([0.02, 0.005, 0.0003, 0.01 ,0.0003]);  % variance for [x, y, theta, v,a]

                % Process noise covariance Q (model uncertainties)
                % self.Q = diag([0.005, 0.005, 0.001, 0.01,0.001]);
                self.Q = diag([0.001, 0.001, 0.0005, 0.002, 0.0005]); % More realistic process noise

            else
                % Make noise covariances more realistic even for "no noise" case
                self.Q = diag([0.001, 0.001, 0.0005, 0.002, 0.0005]); % More realistic process noise
                self.R = diag([0.01, 0.005, 0.0001, 0.005, 0.0002]); % More realistic measurement noise
            end
        end


        function  Distributed_Observer(self,instant_index , weights)
            % Get the number of other vehicles
            num_vehicles = length(self.vehicle.other_vehicles);
            Big_X_hat_1_tempo = zeros(size(self.est_global_state_current)); % Initialize the variable to store the results
            host_id = self.vehicle.vehicle_number; % The vehicle that is estimating the state of other vehicles
            confidence_scores = zeros(num_vehicles, 1); % Store per-vehicle confidence

            % j is the vehicle we want to estimate
            for j = 1:num_vehicles
                weights_new = self.get_weights_for_vehicle(j, host_id, weights, instant_index);
                x_bar_j = self.get_local_state_for_vehicle(j, host_id, weights_new);
                x_hat_i_j = self.get_global_states_for_vehicle(j, num_vehicles, host_id, weights_new, instant_index);
                u_j = self.Get_controller(j);
                weights_new = self.adjust_weights_for_attacker(j, host_id, weights_new);
                use_local_data_from_other = self.vehicle.scenarios_config.use_local_data_from_other;
                output = self.distributed_Observer_each(self.vehicle.vehicle_number, j, x_bar_j, x_hat_i_j, u_j, weights_new, use_local_data_from_other, false);
                
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
                    [Big_X_hat_1_tempo(:,j), temp_confidence_scores] = self.handle_predict_only_switch(j, num_vehicles, output, x_bar_j, x_hat_i_j, u_j, weights_new, use_local_data_from_other, instant_index, confidence_scores);
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
            if instant_index * self.param_sys.dt > 3
                self.self_belief = mean(confidence_scores);
            end

            self.self_belief_log = [self.self_belief_log, self.self_belief];


            %% Save the global state estmate to log
            self.est_global_state_current = Big_X_hat_1_tempo;
            self.est_global_state_log(:, instant_index, :) = Big_X_hat_1_tempo;
        end

        function weights_new = get_weights_for_vehicle(self, j, host_id, weights, instant_index)
            % Determine weights for a given vehicle based on local data/trust
            if abs(host_id - j) == 1 && (self.vehicle.trip_models{j}.flag_local_est_check)
                weights_new = weights;
                weights_new(1) = 0;
            else
                weights_new = weights;
            end
        end

        function x_bar_j = get_local_state_for_vehicle(self, j, host_id, weights_new)
            % Get local state for vehicle j, handle missing data
            x_bar_j = self.vehicle.center_communication.get_local_state(j, host_id);
            if isnan(x_bar_j)
                x_bar_j = zeros(size(self.est_local_state_current));
                weights_new(1) = 0;
            end
        end

        function x_hat_i_j = get_global_states_for_vehicle(self, j, num_vehicles, host_id, weights_new, instant_index)
            % Get global state estimates for all vehicles (PREVIOUS estimates, not current)
            x_hat_i_j = zeros(self.num_states, num_vehicles);
            for k = 1:num_vehicles
                x_hat_i_j_full = self.vehicle.center_communication.get_global_state(k, self.vehicle.vehicle_number);
                if isnan(x_hat_i_j_full)
                    x_hat_i_j_full = zeros(size(self.est_global_state_current));
                    weights_new(k+1) = 0;
                end
                x_hat_i_j(:, k) = x_hat_i_j_full(:, j);
            end
            % for k = 1:num_vehicles
            %     if k == host_id
            %         % For our own vehicle, use our previous estimate from log
            %         if instant_index > 1 && size(self.est_global_state_log, 2) >= instant_index-1
            %             x_hat_i_j(:, k) = self.est_global_state_log(:, instant_index-1, j);
            %         else
            %             % Fallback for first iteration: use current estimate
            %             x_hat_i_j(:, k) = self.est_global_state_current(:, j);
            %         end
            %     else
            %         % For other vehicles, we should ideally get their PREVIOUS estimates
            %         % But since communication center only has current data, we use current as approximation
            %         x_hat_i_j_full = self.vehicle.center_communication.get_global_state(k, self.vehicle.vehicle_number);
            %         if isnan(x_hat_i_j_full) %% Case of missing data
            %             x_hat_i_j_full = zeros(size(self.est_global_state_current));
            %             weights_new(k+1) = 0;
            %         end
            %         x_hat_i_j(:, k) = x_hat_i_j_full(:, j);
            %     end
            % end
        end

        function weights_new = adjust_weights_for_attacker(self, j, host_id, weights_new)
            % Adjust weights if host is attacker
            if self.vehicle.scenarios_config.attacker_update_locally
                if (~isempty(self.vehicle.center_communication.attack_module.scenario) && host_id == self.vehicle.center_communication.attack_module.scenario(1).attacker_id) || (self.vehicle.scenarios_config.lead_senario ~= "constant")
                    weights_new = zeros(size(weights_new));
                    weights_new(1) = 1; % Keep the local state weight
                end
            end
        end

        function [output_final, confidence_scores] = handle_predict_only_switch(self, j, num_vehicles, output, x_bar_j, x_hat_i_j, u_j, weights_new, use_local_data_from_other, instant_index, confidence_scores)
            % Improved: add time limit and blending for prediction-only mode
            
            MAX_PREDICT_ONLY_TIME = self.vehicle.scenarios_config.MAX_PREDICT_ONLY_TIME; % seconds
            N_good = self.vehicle.scenarios_config.N_good; % Number of consecutive good steps to exit predict_only
            dt = self.param_sys.dt;
            if isempty(self.predict_only_counter)
                self.predict_only_counter = zeros(num_vehicles, 1);
            end
            if isempty(self.predict_only_timer)
                self.predict_only_timer = zeros(num_vehicles, 1);
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

        function output = distributed_Observer_each_simplified( self , host_id , j, x_bar_j , x_hat_i_j, u_j ,weights,use_local, predict_only )
            if nargin < 7
                use_local = true; % Default value
            end

            % % SIMPLIFIED VERSION: No consensus term, only local data
            % [A , B]  = self.matrix();

            % % Use the estimated state of vehicle j for the matrix calculation
            % if host_id == j
            %     % If estimating own vehicle, use current state
            %     vehicle_state_for_matrix = self.vehicle.state;
            % else
            %     % If estimating other vehicle, use the estimated state
            %     vehicle_state_for_matrix = x_hat_i_j(:, host_id);
            % end
            
            [A , B]  = self.matrix(self.vehicle.other_vehicles(j).state);
            
            corrected_state = x_hat_i_j(:, host_id) ;
            output = A * corrected_state + B * u_j;
            % Store identity covariance for similarity check
            self.S_log_dist(:, :, host_id) = eye(self.num_states) * 0.1;
            
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
                        correlation_factor = 0.8; % Controls noise correlation (0 = white noise, 1 = fully correlated)
                        white_noise = mvnrnd(zeros(self.num_states,1), self.R)';
                        
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
                        additional_measurement_noise = mvnrnd(zeros(self.num_states,1), self.R)';  % sample additional measurement noise
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
                process_noise = mvnrnd(zeros(self.num_states,1), self.Q)';  % sample process noise
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

        % Convenient helper methods for prediction-only mode
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




        %%% Plotting Functions
        function plot_global_state_log(self)
            % Ensure est_global_state_log is not empty
            if isempty(self.est_global_state_log)
                error('est_global_state_log is empty. No data to plot.');
            end

            num_vehicles = size(self.est_global_state_log, 3); % Number of vehicles
            % num_states = 4; % Assuming X, Y, theta, and V as states
            num_states = size(self.est_global_state_log, 1); % Number of states

            state_labels = {'Position X', 'Position Y', 'Theta', 'Velocity','Acc'};

            figure("Name", "Global Position Estimates " + num2str(self.vehicle.vehicle_number), "NumberTitle", "off");
            % title(['Global Position Estimates ' num2str(self.vehicle.vehicle_number)]);

            for state_idx = 1:num_states
                subplot(num_states, 1, state_idx);
                hold on;
                for v = 1:num_vehicles
                    plot(squeeze(self.est_global_state_log(state_idx, 1:end-1, v)), 'DisplayName', ['Vehicle ', num2str(v)]);
                end
                % Add vertical lines for observer mode switches (prediction-only <-> normal)
                if ~isempty(self.Is_ok_log)
                    % Assume Is_ok_log is a vector of booleans for each vehicle and time step
                    % Try to reshape to [num_vehicles, time_steps] if possible
                    total_steps = size(self.est_global_state_log, 2)-1;
                    try
                        is_ok_mat = reshape(self.Is_ok_log, [num_vehicles, total_steps]);
                    catch
                        is_ok_mat = [];
                    end
                    for v = 1:num_vehicles
                        if ~isempty(is_ok_mat)
                            mode_switches = find(diff(is_ok_mat(v,:)) ~= 0) + 1; % indices where mode changes
                            for ms = mode_switches
                                xline(ms, '--r', 'LineWidth', 1.2, 'Alpha', 0.5);
                            end
                        end
                    end
                end
                title(state_labels{state_idx});
                % xlabel('Time (s)');
                ylabel(state_labels{state_idx});
                legend;
                grid on;
            end
        end

        function plot_error_local_estimated(self)

            state_labels = {'Position X', 'Position Y', 'Theta', 'Velocity','Acc'};
            num_states = length(self.est_local_state_current);
            % Create a figure for the plot
            figure("Name", "Local error " + num2str(self.vehicle.vehicle_number), "NumberTitle", "off");
            hold on;

            for state_idx = 1:num_states
                subplot(num_states, 1, state_idx);
                hold on;
                plot(self.est_local_state_log(state_idx, 1:end-1) - self.vehicle.state_log(state_idx, 1:end-1));
                % plot( self.vehicle.state_log(state_idx, 1:end-1));
                % plot(self.est_local_state_log(state_idx, :) );

                % xlabel('Time (s)');
                ylabel(state_labels{state_idx});
                legend;
                grid on;
            end
        end

        function [global_dist_err, global_theta_err, global_vel_err,global_acc_err] = calculate_global_errors(self)
            % Extract actual and estimated states for error calculation
            estimated_states = self.est_global_state_log; % size: [num_states, num_time_steps, num_vehicles]
            % actual_states = self.vehicle.state_log;      % size: [num_states, num_time_steps]

            num_vehicles = length(self.vehicle.other_vehicles);
            num_time_steps = size(estimated_states, 2);
            num_states = size(estimated_states, 1);

            actual_states_all = zeros(num_states, num_time_steps+1, num_vehicles);
            for v = 1:num_vehicles
                actual_states_all(:,:,v) = self.vehicle.other_vehicles(v).state_log; % [num_states, num_time_steps]
            end

            % Initialize error matrices
            dist_err = zeros(num_vehicles, num_time_steps);
            theta_err = zeros(num_vehicles, num_time_steps);
            vel_err = zeros(num_vehicles, num_time_steps);
            acc_err = zeros(num_vehicles, num_time_steps);


            % Calculate errors for each vehicle at each time step
            for v = 1:num_vehicles
                for t = 1:num_time_steps
                    est = estimated_states(:, t, v); % Estimated state for vehicle v at time t
                    act = actual_states_all(:, t, v); % Actual state for vehicle v at time t

                    % Distance error (Euclidean)
                    dist_err(v, t) = sqrt((est(1) - act(1))^2 + (est(2) - act(2))^2);

                    % Theta error (orientation)
                    theta_err(v, t) = abs(est(3) - act(3));

                    % Velocity error
                    vel_err(v, t) = abs(est(4) - act(4));

                    acc_err(v, t) = abs(est(5) - act(5));
                end
            end

            % Compute mean errors over time for each vehicle
            global_dist_err = mean(dist_err, 2);  % [num_vehicles, 1]
            global_theta_err = mean(theta_err, 2); % [num_vehicles, 1]
            global_vel_err = mean(vel_err, 2);    % [num_vehicles, 1]
            global_acc_err = mean(acc_err, 2);    % [num_vehicles, 1]

        end

        function smoothed_output = apply_output_smoothing(self, raw_output, vehicle_idx, use_adaptive)
            % Apply smoothing to distributed observer output
            % raw_output: new estimate from distributed observer
            % vehicle_idx: which vehicle this estimate is for
            % use_adaptive: whether to use adaptive smoothing based on confidence
            
            if nargin < 4
                use_adaptive = false;
            end
            
            persistent previous_outputs; % Store previous outputs for each vehicle
            if isempty(previous_outputs)
                previous_outputs = [];
            end
            
            % Initialize if first call for this vehicle
            if size(previous_outputs, 2) < vehicle_idx || isempty(previous_outputs)
                if isempty(previous_outputs)
                    previous_outputs = raw_output;
                else
                    previous_outputs(:, vehicle_idx) = raw_output;
                end
                smoothed_output = raw_output;
                return;
            end
            
            % Get smoothing factor
            if use_adaptive && vehicle_idx <= length(self.reputation_scores)
                % Adaptive smoothing based on reputation score
                base_alpha = 0.3;
                reputation_factor = max(0.1, self.reputation_scores(vehicle_idx));
                alpha = base_alpha * reputation_factor;
            else
                alpha = 0.3; % Fixed smoothing factor
            end
            
            % Apply exponential smoothing
            smoothed_output = (1 - alpha) * raw_output + alpha * previous_outputs(:, vehicle_idx);
            
            % Update stored value
            previous_outputs(:, vehicle_idx) = smoothed_output;
        end

        function filtered_noise = generate_smooth_noise(self, base_covariance, correlation_factor)
            % Generate smooth, correlated noise for more realistic simulation
            % base_covariance: base noise covariance matrix
            % correlation_factor: temporal correlation (0-1, higher = more correlated)
            
            if nargin < 3
                correlation_factor = 0.8;
            end
            
            % Generate white noise
            white_noise = mvnrnd(zeros(self.num_states, 1), base_covariance)';
            
            % Apply first-order Markov correlation
            self.correlated_noise_state = correlation_factor * self.correlated_noise_state + ...
                                          sqrt(1 - correlation_factor^2) * white_noise;
            
            % Apply additional smoothing
            filtered_noise = (1 - self.noise_filter_alpha) * self.correlated_noise_state + ...
                             self.noise_filter_alpha * self.previous_noise;
            
            % Update previous noise for next iteration
            self.previous_noise = filtered_noise;
        end



    end
end
