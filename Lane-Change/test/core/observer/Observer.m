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
            self.est_global_state_log(:,1,:) = inital_global_state;

            self.est_local_state_log = zeros(self.num_states, Nt);
            self.est_local_state_log(:,1) = inital_local_state;

            self.tolerances = [5, 2, deg2rad(8), 2 , 1]; % Tolerances for each state [X, Y, theta, V, A]

            % NEW : for calculate the self belief = gamma in the controller
            self.self_belief = 1; % Initial self-belief
            self.reputation_scores = ones(num_vehicles, 1); % Initial reputation = 1
            self.P_pred_dist = eye(self.num_states); % Initial covariance
            self.S_log_dist = zeros(self.num_states, self.num_states, Nt); % Log innovation covariance
            

            if (self.vehicle.scenarios_config.Local_observer_type == "kalman")
                % Kalman Filter Parameters

                self.P = eye(self.num_states);              % initial error covariance

                if self.vehicle.scenarios_config.Is_noise_mesurement == true % if the measurement is noisy
                    %% Noise Covariances
                    % These are example values; adjust based on your system/sensor characteristics.
                    % Measurement noise covariance R (variances on sensor measurements)
                    self.R = diag([0.01, 0.01, 0.0003, 0.01 ,0.001 ]);  % variance for [x, y, theta, v]

                    % Process noise covariance Q (model uncertainties)
                    self.Q = diag([0.005, 0.005, 0.001, 0.01,0.001]);
                else
                    self.Q = 1e-6 * eye(self.num_states); % Small process noise covariance (no model error)
                    self.R = 1e-10 * eye(self.num_states); % Very small measurement noise covariance (perfect measurements)
                end
            elseif  (self.vehicle.scenarios_config.Local_observer_type == "observer")
                % Observer Gain
                desired_poles = [-1 -2 -3 -4 -5]; % Desired poles for the observer
                self.L_gain = place(A', C', desired_poles)';
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
                %% TODO : need change the weight out side (and make it Like in the paper Shenya)
                %% if j is not host vehicle
                % And local data of j is flag_local_est_check (not good)
                % OR the final score of j is less than 0.5
                % So we need to set the weight of local state to 0
                if( (host_id ~= j )  && (self.vehicle.trip_models{j}.flag_local_est_check  ) )
                    weights_new  = weights;
                    weights_new(1) = 0;
                else
                    weights_new  = weights;
                end

                %%-----  Get local of j vehicle
                x_bar_j = self.vehicle.center_communication.get_local_state(j,host_id);
                if ( isnan(x_bar_j)) % Case DOS attack or no local state is received
                    x_bar_j = zeros(size(self.est_local_state_current)); % If we don't have local state of j vehicle
                    weights_new(1) = 0;
                end

                % ------- Get global state of other vehicles
                % Go through all the other vehicles , start from the second vehicle
                x_hat_i_j = zeros(self.num_states, num_vehicles); % Initialize the variable to store the global state of other vehicles

                for k = 1:num_vehicles
                    % Go in car number "k" , Get car "j" in global estimat of "k"
                    x_hat_i_j_full = self.vehicle.center_communication.get_global_state(k,self.vehicle.vehicle_number);
                    if ( isnan(x_hat_i_j_full))% Case DOS attack or no global state is received
                        x_hat_i_j_full = zeros(size(self.est_global_state_current)); % If we don't have local state of j vehicle
                        weights_new(k+1) = 0; 
                    end
                    x_hat_i_j(:,k) =  x_hat_i_j_full(:,j);
                end


                %% CONTROLLER
                u_j =  Get_controller(self,j); % Get the control input of vehicle j

                %% Just let the attacker update with its local state 
                if (self.vehicle.scenarios_config.attacker_update_locally && ~isempty(self.vehicle.center_communication.attack_module.scenario))
                    if host_id == self.vehicle.center_communication.attack_module.scenario(1).attacker_id
                        % If the host vehicle is the attacker, set all weights to 0
                        weights_new = zeros(size(weights));
                        weights_new(1) = weights(1); % Keep the local state weight
                    else
                        weights_new = weights; % Use the provided weights
                    end
                end

                use_local_data_from_other = self.vehicle.scenarios_config.use_local_data_from_other;
                %% Estimate the state of vehicle j using the distributed observer
                output = distributed_Observer_each( self , self.vehicle.vehicle_number, j , x_bar_j , x_hat_i_j, u_j, weights_new ,use_local_data_from_other , false);


                self.update_reputation(j, output, x_bar_j);

                % Check if we want to use the estimated state of vehicle j or not
                if instant_index*self.param_sys.dt < 3
                    Big_X_hat_1_tempo(:,j) = output; % Append the result
                    self.Is_ok_log = [self.Is_ok_log ,true];
                else
                    % We need to wait for 3 seconds to check if the output is similar
                    % Because the observer need time to converge (to have a good estimation)
                    
                    output_2 = distributed_Observer_each( self , self.vehicle.vehicle_number, j , x_bar_j , x_hat_i_j, u_j, weights_new , use_local_data_from_other , true);
                    % Check if 2 ouput is similar , in a range
                    [is_ok, log_elem, confidence] = check_elementwise_similarity(self, output, output_2, instant_index, j);
                    confidence_scores(j) = confidence;
                    self.Is_ok_log = [self.Is_ok_log ,is_ok];
                    if ~is_ok && self.vehicle.scenarios_config.Use_predict_observer
                        self.log_element = [self.log_element, log_elem]; % Append the first element
                        Big_X_hat_1_tempo(:,j) = output_2; % Append the result
                    else
                        Big_X_hat_1_tempo(:,j) = output; % Append the result
                    end

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

        function output = distributed_Observer_each( self , host_id , j, x_bar_j , x_hat_i_j, u_j ,weights,use_local, predict_only )
            if nargin < 7
                use_local = true; % Default value
            end

            Sig = zeros(self.num_states,1); % Initialize consensus term
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
                % CASE not send local state , only host vehicle local state is use
                if use_local
                    output = A*( x_hat_i_j(: , host_id) + Sig + w_i0 * (x_bar_j - x_hat_i_j(: , host_id)) ) + B*u_j ;
                else
                    if (host_id == j)
                        output = A*( x_hat_i_j(: , host_id) + Sig + w_i0 * (x_bar_j - x_hat_i_j(: , host_id)) ) + B*u_j ;
                    else
                        output = A*( x_hat_i_j(: , host_id) + Sig) + B*u_j ;
                    end
                end
            end


        end


        function [is_ok, log_element, confidence] = check_elementwise_similarity(self, output1, output2, instant_index, vehicle_id)
            is_ok = true;
            log_element = [];
            confidence = 1; % Default confidence

            S = self.S_log_dist(:, :, vehicle_id);
            dynamic_tolerances = self.tolerances * (1 + trace(S) / self.num_states);

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
                % Calculate the control input for each vehicle locally by the estimated state of other vehicle

                if (self.vehicle.vehicle_number ~= 1) % not lead vehicle
                    if (j==1) % if we are estimating the lead vehicle controller
                        u_j = [0;0]; % keep the lead vehicle's control input as zero
                    elseif j == self.vehicle.vehicle_number % if we are estimating our own vehicle
                        u_j = self.vehicle.input; % Control input of the current vehicle
                    else % if we are estimating another vehicle
                        est_local_j =  self.est_global_state_current(:,j); % get est_local_j in our host vehicle
                        [~, u_j ,~] = self.vehicle.controller2.get_optimal_input(j, est_local_j, [0;0], self.vehicle.other_vehicles(j).lane_id, 0, self.vehicle.initial_lane_id, self.vehicle.other_vehicles(j).direction_flag, "est", 0);
                    end
                else % Is lead vehicle
                    u_j = self.vehicle.input;
                end
            end
        end

        function Local_observer(self , mesure_state , instant_index)
            % self.est_local_state_current = state;
            [A , B]  = self.matrix();

            if self.vehicle.scenarios_config.Is_noise_mesurement == true
                % process_noise = mvnrnd(zeros(4,1), Q)';  % sample process noise
                measurement_noise = mvnrnd(zeros(4,1), self.R)';  % sample measurement noise
                mesure_state = mesure_state + measurement_noise; % Add noise to the measurement
            end

            if self.vehicle.scenarios_config.Local_observer_type == "observer"
                self.est_local_state_current = A * self.est_local_state_current + B * self.vehicle.input + self.L_gain * (mesure_state - self.est_local_state_current);
            elseif self.vehicle.scenarios_config.Local_observer_type == "kalman"
                % Kalman Filter
                self.kalman_filter(mesure_state);
                % self.est_local_state_log(:, instant_index+1) = self.est_local_state_current;
            else %"true"
                self.est_local_state_current = mesure_state;
                % self.est_local_state_log(:, instant_index+1) = self.est_local_state_current;
            end
            self.est_local_state_log(:, instant_index+1) = self.est_local_state_current;


        end

        function kalman_filter(self, mesure_state)
            % Kalman Filter
            [A , B]  = self.matrix();

            % Prediction Step
            x_pred = A * self.est_local_state_current + B * self.vehicle.input;
            P_pred = A * self.P * A' + self.Q;

            % Measurement Update Step
            y = mesure_state; % Measurement
            C = eye(self.num_states); % Measurement matrix (assuming we can measure all states)
            S = C * P_pred * C' ; % Innovation covariance
            K = P_pred * C' / S; % Kalman gain

            % Update the state estimate
            self.est_local_state_current = x_pred + K * (y - C * x_pred);
            % Update the error covariance
            self.P = (eye(size(K,1)) - K * C) * P_pred;
        end

        % This model use in obesrver , so need to be here, in the observer class
        function  [A , B] = matrix(self )
            % theta = self.vehicle.state(3);
            %% TODO : Need to be change
            theta = 0;
            Ts = self.param_sys.dt;
            v = self.vehicle.state(4);
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

        function [global_dist_err, global_theta_err, global_vel_err] = calculate_global_errors(self)
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
                end
            end

            % Compute mean errors over time for each vehicle
            global_dist_err = mean(dist_err, 2);  % [num_vehicles, 1]
            global_theta_err = mean(theta_err, 2); % [num_vehicles, 1]
            global_vel_err = mean(vel_err, 2);    % [num_vehicles, 1]
        end



    end
end
