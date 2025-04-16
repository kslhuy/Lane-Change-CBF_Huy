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
    end
    methods
        function self = Observer( vehicle , veh_param, inital_global_state, inital_local_state)
            self.vehicle = vehicle;
            self.param_sys = veh_param; % Set the vehicle system parameters

            self.est_local_state_current = inital_local_state;

            self.est_global_state_current = inital_global_state;

            num_states = 4 ; % X , Y , theta , V
            Nt = self.vehicle.total_time_step;
            num_vehicles = length(self.vehicle.other_vehicles);

            self.est_global_state_log = zeros(num_states, Nt, num_vehicles);
            self.est_global_state_log(:,1,:) = inital_global_state;

            self.est_local_state_log = zeros(num_states, Nt);
            self.est_local_state_log(:,1) = inital_local_state;

            self.tolerances = [5, 2, deg2rad(8), 2];

            if (self.vehicle.scenarios_config.Local_observer_type == "kalman")
                % Kalman Filter Parameters

                self.P = eye(4);              % initial error covariance

                %% Noise Covariances
                % These are example values; adjust based on your system/sensor characteristics.
                % Measurement noise covariance R (variances on sensor measurements)
                self.R = diag([0.01, 0.01, 0.0003, 0.01]);  % variance for [x, y, theta, v]

                % Process noise covariance Q (model uncertainties)
                self.Q = diag([0.005, 0.005, 0.001, 0.01]);

            elseif  (self.vehicle.scenarios_config.Local_observer_type == "observer")
                % Observer Gain
                self.L_gain = place(A', C', desired_poles)';
            end
        end


        function  Distributed_Observer(self,instant_index , weights)
            % Get the number of other vehicles
            num_vehicles = length(self.vehicle.other_vehicles);

            Big_X_hat_1_tempo = zeros(size(self.est_global_state_current)); % Initialize the variable to store the results

            % flag_glob_est_check = false;

            host_id = self.vehicle.vehicle_number; % The vehicle that is estimating the state of other vehicles
            % j is the vehicle we want to estimate
            for j = 1:num_vehicles

                x_hat_i_j = zeros(4, num_vehicles); % Initialize the variable to store the global state of other vehicles

                %% if j is not host vehicle
                % OR if j is not the direct neighbor of host vehicle
                % And if TrIP of host in j have flag_local_est_check
                % So we need to set the weight of local state to 0
                if( (host_id ~= j | abs(host_id - j) == 1)  && self.vehicle.trip_models{j}.flag_local_est_check)
                    weights_new  = weights;
                    weights_new(1) = 0;
                else
                    weights_new  = weights;
                end

                %% Get local of j vehicle
                x_bar_j = self.vehicle.center_communication.get_local_state(j,host_id);
                if ( isnan(x_bar_j))
                    x_bar_j = zeros(size(self.est_local_state_current)); % If we don't have local state of j vehicle
                    weights_new(1) = 0;
                end

                % ------- To get global state of other vehicles
                % Go through all the other vehicles , start from the second vehicle
                for k = 1:num_vehicles
                    % Go in car number "k" , Get car "j" in global estimat of "k"
                    x_hat_i_j_full = self.vehicle.center_communication.get_global_state(k,self.vehicle.vehicle_number);
                    if ( isnan(x_hat_i_j_full))
                        x_hat_i_j_full = zeros(size(self.est_global_state_current)); % If we don't have local state of j vehicle
                        weights_new(k+1) = 0; 
                    end
                    x_hat_i_j(:,k) =  x_hat_i_j_full(:,j);
                end


                %% CONTROLLER

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


                %% Estimate the state of vehicle j using the distributed observer
                output = distributed_Observer_each( self , self.vehicle.vehicle_number, j , x_bar_j , x_hat_i_j, u_j, weights_new ,true , false);

                % Check if we want to use the estimated state of vehicle j or not
                if self.vehicle.scenarios_config.Use_predict_observer == false
                    Big_X_hat_1_tempo(:,j) = output; % Append the result
                    self.Is_ok_log = [self.Is_ok_log ,true];
                else
                    if instant_index*self.param_sys.dt > 3
                        output_2 = distributed_Observer_each( self , self.vehicle.vehicle_number, j , x_bar_j , x_hat_i_j, u_j, weights_new , true , true);
                        % Check if 2 ouput is similar , in a range
                        [is_ok , log_elem] = check_elementwise_similarity(self ,output, output_2, self.tolerances, j, instant_index);
                        self.Is_ok_log = [self.Is_ok_log ,is_ok];
                        if is_ok
                            Big_X_hat_1_tempo(:,j) = output; % Append the result
                        else
                            self.log_element = [self.log_element, log_elem]; % Append the first element
                            Big_X_hat_1_tempo(:,j) = output_2; % Append the result
                        end
                    else
                        % If we are in the first 3 seconds, just use the output (because need time to converge)
                        Big_X_hat_1_tempo(:,j) = output; % Append the result
                    end
                end
            end

            %% Save the global state estmate to log
            self.est_global_state_current = Big_X_hat_1_tempo;
            self.est_global_state_log(:, instant_index, :) = Big_X_hat_1_tempo;
        end

        function output = distributed_Observer_each( self , host_id , j, x_bar_j , x_hat_i_j, u_j ,weights,use_local, predict_only )
            if nargin < 7
                use_local = true; % Default value
            end

            Sig = zeros(4,1); % Initialize consensus term
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
                if use_local
                    if (host_id == j)
                        output = A*( x_hat_i_j(: , host_id) + w_i0 * (x_bar_j - x_hat_i_j(: , host_id)) ) + B*u_j ;
                    else
                        output = A*( x_hat_i_j(: , host_id) ) + B*u_j ;
                    end
                else
                    output = A*( x_hat_i_j(: , host_id) ) + B*u_j ;
                end
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


        function [is_ok , log_element] = check_elementwise_similarity(self ,output1, output2, tolerances, vehicle_id, time_idx)
            is_ok = true; % Initialize as true
            log_element = [];
            for i = 1:length(output1)
                diff_val = abs(output1(i) - output2(i));
                if diff_val > tolerances(i)
                    % fprintf('Vehicle %d @ index %d | Element %d diff = %.4f exceeds tolerance %.4f\n', ...
                    %     vehicle_id, time_idx, i, diff_val, tolerances(i));
                    is_ok = false;
                    log_element = [log_element, i];
                    % if i == 1
                    %     log_element = log_element[];
                    %     log_element = log_element[i];
                    % elseif i == 2
                    %     log_element = "Y";
                    % elseif i == 3
                    %     log_element = "Theta";
                    % elseif i == 4
                    %     log_element = "V";
                    % end

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
            else
                self.est_local_state_current = state;
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
            C = eye(4); % Measurement matrix (assuming we can measure all states)
            S = C * P_pred * C' ; % Innovation covariance
            K = P_pred * C' / S; % Kalman gain

            % Update the state estimate
            self.est_local_state_current = x_pred + K * (y - C * x_pred);
            % Update the error covariance
            self.P = (eye(size(K,1)) - K * C) * P_pred;
        end

        % This model use in obesrver , so need to be here, in the observer class
        function  [A , B] = matrix(self)
            theta = self.vehicle.state(3);
            Ts = self.param_sys.dt;
            v = self.vehicle.state(4);
            % Here in continuous time
            % A = [ 0 0 0 cos(theta);
            %      0 0 0 sin(theta);
            %      0 0 1 0;
            %      0 0 0 0];

            % B = [ 0 0;
            %       0 0;
            %       0 0;
            %       1 0];

            % Here in discrete time
            A = [ 1 0 -v*sin(theta)*Ts cos(theta)*Ts;
                0 1 v*cos(theta)*Ts sin(theta)*Ts;
                0 0 1 0;
                0 0 0 1];

            B = [0 0
                0 0
                0 Ts
                Ts 0];
        end

        function plot_global_state_log(self)
            % Ensure est_global_state_log is not empty
            if isempty(self.est_global_state_log)
                error('est_global_state_log is empty. No data to plot.');
            end

            num_vehicles = size(self.est_global_state_log, 3); % Number of vehicles
            % num_states = 4; % Assuming X, Y, theta, and V as states
            num_states = size(self.est_global_state_log, 1); % Number of states

            state_labels = {'Position X', 'Position Y', 'Theta', 'Velocity'};

            figure("Name", "Global Position Estimates " + num2str(self.vehicle.vehicle_number), "NumberTitle", "off");
            % title(['Global Position Estimates ' num2str(self.vehicle.vehicle_number)]);

            for state_idx = 1:num_states
                subplot(4, 1, state_idx);
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

            state_labels = {'Position X', 'Position Y', 'Theta', 'Velocity'};
            num_states = length(self.est_local_state_current);
            % Create a figure for the plot
            figure("Name", "Local error " + num2str(self.vehicle.vehicle_number), "NumberTitle", "off");
            hold on;

            for state_idx = 1:num_states
                subplot(4, 1, state_idx);
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



    end
end
