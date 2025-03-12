classdef TriPTrustModel < handle


    %{
    Overview how of this ChatGPT
  https://discord.com/channels/1123389035713400902/1326223031667789885/1327306982218141727
    %}

    properties
        wv = 2.0;  % Weight for velocity
        wd = 1.0;  % Weight for distance
        wa = 1.0;  % Weight for acceleration
        wj = 1.0;  % Weight for jerkiness
        wt = 0.5; % Trust decay weight
        C = 0.2;   % Regularization constant
        tacc = 1.2;% Trust-based acceleration scaling factor
        k = 5;     % Number of trust levels
        rating_vector; % Trust rating vector

        % New properties for global estimate checks
        sigma2 = 1; % Sensitivity parameter for cross-validation trust factor
        tau2 = 0.5;   % Sensitivity parameter for local consistency trust factor
        last_d;
        trust_sample_log;
        gamma_cross_log;
        gamma_local_log;
        v_score_log;
        d_score_log;
        a_score_log;
        beacon_score_log;
        final_score_log;
    end

    methods
        function self = TriPTrustModel()

            % Initialize trust rating vector R_y is accumulates all past outcomes element of vector r_y^x
            % R_y = [R_y(1) ,R_y(2), R_y(3) , R_y(4) , R_y(5)]
            % 1 2 3 4 5 is each trust level

            self.rating_vector = zeros(1, self.k);
            self.rating_vector(5) = 1;
            self.last_d = 20;

            self.trust_sample_log = [];
            self.gamma_cross_log = [];
            self.gamma_local_log = [];
            self.v_score_log = [];
            self.d_score_log = [];
            self.a_score_log = [];
            self.beacon_score_log = [];
            self.final_score_log = [];

        end

        function v_score_final = evaluate_velocity(~,host_id , target_id, v_y,v_host, v_leader, a_leader, b_leader , tolerance)
            if nargin < 8
                tolerance = 0.1; % Default tolerance if not provided
            end

            % Get current leader velocity ,
            v_ref = v_leader + b_leader * a_leader;


            %{
            Evaluate velocity with a tolerance range for minor deviations.

            Parameters:
                v_y: Reported velocity of the follower vehicle.
                v_leader: Leader's velocity.
                a_leader: Leader's acceleration.
                b_leader: Beacon interval (time since the last beacon).
                tolerance: Acceptable fraction of v_ref deviation without penalty.

            Returns:
                Velocity trust score (0 to 1).

            Explain in https://discord.com/channels/1123389035713400902/1326223031667789885/1327291670911254528
            %}
            if (host_id - target_id) > 0
                alpha = 0.8;
            else
                alpha = 0.2;
            end



            % Meaning that leader is move back (brake) or stop
            if v_ref == 0 || v_leader == 0 || sign(v_ref*v_leader) < 0 || sign(v_ref*v_host) < 0
                % Handle edge case for v_ref <= 0
                % TODO : why abs(v_y)
                % Exemple v_y positive , try to run , so score is 0
                % v_y negative , try to brake , so is ?????
                v_score_final =  max(1 - abs(v_y), 0);

                return;
            end
            % Calculate absolute deviation as a fraction of reference velocity
            % meaning report vehicle is faster or slower than reference velocity
            deviation_ref = abs(v_y - v_ref) / v_ref;
            deviation_host = abs(v_y - v_host) / v_host;

            if deviation_ref <= tolerance
                % Within tolerance, give max score
                v_score_ref = 1.0;
            else
                % Scale penalty for deviations beyond tolerance
                scaled_penalty = (deviation_ref - tolerance) / (1 - tolerance);
                v_score_ref =  max(1 - scaled_penalty, 0);
            end

            v_score_host = max(1 - deviation_host, 0);

            v_score_final = (1-alpha)*v_score_ref +  (alpha)* v_score_host ;


        end

        function d_score = evaluate_distance(~, d_y, d_measured)
            d_score = max(1 - abs((d_y - d_measured)/d_measured), 0);
        end

        function a_score = evaluate_acceleration(~, a_y, a_host, d, ts)
            %% TODO : the d now is the diff between host and target (i think it still work)
            %% Maybe need to change to the diff between the measured distance of the host in one time step
            v_rel = diff(d) / ts;
            a_diff = a_y - a_host;
            a_score = max(1 - abs(v_rel / ts * a_diff), 0);
        end

        function j_score = evaluate_jerkiness(~, j_y, j_thresh)
            if nargin < 3
                j_thresh = 2.0;
            end
            if abs(j_y) > j_thresh
                j_score = min(j_thresh / abs(j_y), 1);
            else
                j_score = 1;
            end
        end

        function beacon_score = evaluate_beacon_timeout(~, beacon_received)
            if (beacon_received)
                beacon_score = 1;
            else
                beacon_score = 0;
            end
        end

        function trust_sample = calculate_trust_sample_wo_Acc(self, v_score, d_score, a_score, beacon_score)
            trust_sample = beacon_score * (v_score^self.wv) * (d_score^self.wd);
        end

        function trust_sample = calculate_trust_sample_w_Jek(self, v_score, d_score, a_score, j_score, beacon_score)
            trust_sample = beacon_score * (v_score^self.wv) * (d_score^self.wd) * (a_score^self.wa) * (j_score^self.wj);
        end

        function trust_sample = calculate_trust_sample(self, v_score, d_score, a_score, beacon_score)
            trust_sample = beacon_score * (v_score^self.wv) * (d_score^self.wd) * (a_score^self.wa) ;
        end

        function update_rating_vector(self, trust_sample)
            %  Map trust sample to a specific trust level in the rating vector

            trust_level = min(round(trust_sample * (self.k - 1) + 1)  , (self.k) );
            trust_vector = zeros(1, self.k); % trust_vector = r_y^x
            trust_vector(trust_level) = 1;

            % Compute current trust score (sigma_y) to use in lambda_y calculation

            current_trust_score = self.calculate_trust_score();
            % Define lambda_y as per equation (9): lambda_y = sigma_y * w_t
            lambda_y = current_trust_score * self.wt;

            % Update the rating vector (R_y) using the aging factor lambda_y
            self.rating_vector = (1 - lambda_y) * self.rating_vector + trust_vector;
            % self.rating_vector
            % lambda_y
        end

        function trust_score = calculate_trust_score(self)
            %Explain : https://discord.com/channels/1123389035713400902/1327310432381435914/1327403771663224873

            % Normalize the rating vector to ensure it represents probabilities
            % a = (1/self.k) = 1/5 = 0.2 in  self.C / self.k
            S_y = (self.rating_vector + self.C / self.k) / (self.C + sum(self.rating_vector));

            % Define weights with a small epsilon to avoid zero weight for the lowest level
            epsilon = 0.01; % Small positive constant
            weights = ((0:(self.k-1)) + epsilon) / (self.k-1 + epsilon);

            % Calculate the trust score as a weighted average
            trust_score = sum(weights .* S_y);
        end



        function [final_score,trust_sample,gamma_cross, v_score ,d_score,a_score,beacon_score] = calculateTrust(self , host_vehicle, target_vehicle, leader_vehicle, neighbors, leader_beacon_interval, time_step)
            % calculateTrust - Computes trust scores for a specific vehicle
            %
            % Inputs:
            %   host_id       - ID of the current vehicle
            %   x_local          - Local state of the current vehicle (3x1 vector)
            %   neighbors_states - A matrix where each column is the state of a neighbor vehicle (3xN matrix)
            %   neighbors_ids    - IDs of the neighbor vehicles (1xN vector)
            %   time_step        - Current simulation time step (scalar)
            %
            % Outputs:
            %   trust_scores     - Trust scores for each neighbor (1xN vector)

            host_id = host_vehicle.vehicle_number;
            target_id = target_vehicle.vehicle_number;
            % Reported data for trust evaluation

            half_lenght_vehicle = target_vehicle.param.l_r; % distance between vehicle's c.g. and rear axle

            % leader_velocity = leader_vehicle.observer.est_local_state_current(4);
            leader_state = host_vehicle.center_communication.get_local_state(leader_vehicle.vehicle_number);
            leader_input = host_vehicle.center_communication.get_input(leader_vehicle.vehicle_number);
            leader_velocity = leader_state(4) ;
            leader_acceleration = leader_input(1);

            % Measurement part host (radar, lidar  )
            host_pos_X = host_vehicle.observer.est_local_state_current(1);
            host_velocity = host_vehicle.observer.est_local_state_current(4);
            host_acceleration = host_vehicle.input(1);

            %%

            % if (host_id - neighbors_ids) < 0  =>  host is Front , else = Behind

            % host_distance_measurement = (host_id - target_id)*(target_vehicle.observer.est_local_state_current(1) - host_vehicle.observer.est_local_state_current(1)) - half_lenght_vehicle;

            % Why target_vehicle.state(1) : because is the host_distance_measurement , its in the point view of Host
            % So need to be acurate , not disturb by attack like "target_pos_X" (below )
            host_distance_measurement = (host_id - target_id)*(target_vehicle.state(1) - host_pos_X) - half_lenght_vehicle;


            %% ---

            %% TODO : need to get real distance between host and target
            target_state = host_vehicle.center_communication.get_local_state(target_id);
            target_input = host_vehicle.center_communication.get_input(target_id);

            target_pos_X = target_state(1);
            target_reported_distance = (host_id - target_id)*(target_pos_X - host_pos_X) - half_lenght_vehicle;
            target_reported_velocity = target_state(4);
            target_reported_acceleration = target_input(1);

            % reported_distance = (host_id - target_id)*(target_pos_X - host_pos_X) - half_lenght_vehicle;             % position host - pos neighbors
            % reported_velocity = target_vehicle.observer.est_local_state_current(4);
            % reported_acceleration = target_vehicle.input(1);


            % Trust evaluation
            v_score = self.evaluate_velocity( host_id , target_id , host_velocity , target_reported_velocity, leader_velocity, leader_acceleration, leader_beacon_interval,0.1);
            d_score = self.evaluate_distance(target_reported_distance, host_distance_measurement);

            a_score = self.evaluate_acceleration(target_reported_acceleration, host_acceleration, [host_distance_measurement, self.last_d], time_step);
            self.last_d = host_distance_measurement;

            beacon_score = self.evaluate_beacon_timeout(true);

            % Compute trust sample for local estimation
            trust_sample = self.calculate_trust_sample_wo_Acc(v_score, d_score, a_score, beacon_score);

            % Compute global estimate trust factors
            gamma_cross = self.compute_cross_host_target_factor(host_vehicle, target_vehicle);

            gamma_local = self.compute_local_consistency_factor(host_vehicle, target_vehicle, neighbors);



            % Compute extended trust sample
            trust_sample_ext = 0.5*trust_sample  +  0.5*(gamma_cross * gamma_local);

            % trust_sample_ext = trust_sample * gamma_cross ;
            % trust_sample_ext = trust_sample ;
            self.update_rating_vector(trust_sample_ext);
            final_score = self.calculate_trust_score();

            % Log data for analysis
            self.trust_sample_log = [self.trust_sample_log, trust_sample];
            self.gamma_cross_log = [self.gamma_cross_log, gamma_cross];
            self.gamma_local_log = [self.gamma_local_log, gamma_local];
            self.v_score_log = [self.v_score_log, v_score];
            self.d_score_log = [self.d_score_log, d_score];
            self.a_score_log = [self.a_score_log, a_score];
            self.beacon_score_log = [self.beacon_score_log, beacon_score];
            self.final_score_log = [self.final_score_log, final_score];
            
        end


        % Only compare with the directed neighbor (not all neighbors) , or more specific is the target vehicle
        function gamma_cross = compute_cross_host_target_factor(self, host_vehicle, target_vehicle)
            % Inputs:
            %   host_vehicle: The vehicle evaluating trust
            %   target_vehicle: The neighbor whose global estimate is being evaluated
            %   neighbors: Array of neighbor vehicle objects
            %
            % Output:
            %   gamma_cross: Trust factor based on cross-validation (0 to 1)

            host_global_estimate = host_vehicle.observer.est_global_state_current;

            % Get target vehicle's global estimate
            target_global_estimate = target_vehicle.center_communication.get_global_state(target_vehicle.vehicle_number);

            % Compute covariance matrix (adaptive variance)
            sigma2_diag_element = [2,1 ,0.01, 1];
            sigma2_matrix = diag(sigma2_diag_element);
            % Compute total discrepancy D_i,l(k)
            D = 0;
            num_vehicles = size(target_global_estimate,2);
            for j = 1:num_vehicles
                x_diff = target_global_estimate(:, j) - host_global_estimate(:, j);
                D = D + x_diff' * inv(sigma2_matrix) * x_diff; % Mahalanobis distance
            end

            % Compute trust factor
            gamma_cross = exp(-D);
        end



        function gamma_local = compute_local_consistency_factor(self, host_vehicle, target_vehicle, neighbors)
            % Inputs:
            %   host_vehicle: The vehicle evaluating trust
            %   target_vehicle: The neighbor whose global estimate is being evaluated
            %   local_measurements: Struct with sensor data (e.g., relative state to predecessor)
            %
            % Output:
            %   gamma_local: Trust factor based on local consistency (0 to 1)

            half_lenght_vehicle = target_vehicle.param.l_r; % distance between vehicle's c.g. and rear axle

            M_i = 0; % Set of vehicles used for local consistency check (max = 2 for predecessor and successor)
            predecessor = [];
            successor = [];
            host_id = host_vehicle.vehicle_number;
            target_global_estimate = target_vehicle.center_communication.get_global_state(target_vehicle.vehicle_number);

            x_l_i = target_global_estimate([1,4], host_id); % In L car , get the estimate of the host vehicle (i is host)
            % If position errors are around 2m and velocity errors around 1m/s
            tau2_diag_element = [2 , 0.5] ;
            tau2_matrix = diag(tau2_diag_element);
            E = 0; % Total error

            for m = 1:length(neighbors)
                car_idx = neighbors(m).vehicle_number;
                if abs(car_idx - host_vehicle.vehicle_number) == 1
                    if (car_idx > host_vehicle.vehicle_number)
                        M_i = M_i + 1;
                        successor = neighbors(m);
                    end
                    if (car_idx < host_vehicle.vehicle_number)
                        M_i = M_i + 1;
                        predecessor = neighbors(m); % Mean following vehicle
                    end
                end
            end

            % If no predecessor or successor is found, return maximum trust (no comparison possible)
            if isempty(predecessor) && isempty(successor)
                gamma_local = 1;
                return;
            end


            if (~isempty(predecessor))
                % Get local measurement (e.g., relative position and velocity to predecessor)
                % Assume local_measurements.predecessor is a vector [rel_pos; rel_vel]
                host_distance_measurement = (predecessor.state(1) - host_vehicle.state(1)) - half_lenght_vehicle;
                velocity_diff = abs(predecessor.state(4) - host_vehicle.state(4));
                y_i_pred = [host_distance_measurement; velocity_diff];

                pred_id = predecessor.vehicle_number;

                x_l_pred = target_global_estimate([1,4], pred_id); % In L car , get the estimate of the predceding of host vehicle (i + 1 is pred)

                % Compute expected relative state from global estimate
                rel_state_est = abs(x_l_pred - x_l_i - [half_lenght_vehicle;0]);

                % Compute consistency error e_i,l^(j)(k)
                e = rel_state_est - y_i_pred;

                E =  e' * inv(tau2_matrix) * e + E;

            end

            if (~isempty(successor))
                % Get local measurement (e.g., relative position and velocity to successor)
                % Assume local_measurements.successor is a vector [rel_pos; rel_vel]
                host_distance_measurement = (host_vehicle.state(1) - successor.state(1) ) - half_lenght_vehicle;
                velocity_diff = abs(successor.state(4) - host_vehicle.state(4));
                y_i_successor = [host_distance_measurement; velocity_diff]; % Host measurement

                successor_id = successor.vehicle_number;

                x_l_successor = target_global_estimate([1,4], successor_id); % In L car , get the estimate of the successor of host vehicle (i - 1 is succ)

                % Compute expected relative state from global estimate
                rel_state_est = abs(x_l_i - x_l_successor - [half_lenght_vehicle;0]);

                % Compute consistency error e_i,l^(j)(k)
                e = (rel_state_est - y_i_successor);
                E = e' * inv(tau2_matrix) * e + E;

            end

            % Compute trust factor
            gamma_local = exp(-E);
        end




        function plot_trust_log(self,nb_host_car , nb_target_car)
            figure;
            hold on;
            
            plot(self.trust_sample_log, 'DisplayName', 'Trust Sample');
            plot(self.gamma_cross_log, 'DisplayName', 'Gamma Cross');
            plot(self.gamma_local_log, 'DisplayName', 'Gamma Local');
            plot(self.v_score_log, 'DisplayName', 'V Score');
            plot(self.d_score_log, 'DisplayName', 'D Score');
            plot(self.final_score_log, 'DisplayName', 'Final Score' , 'LineWidth', 2);
            
            % plot(self.a_score_log, 'DisplayName', 'A Score');
            % plot(self.beacon_score_log, 'DisplayName', 'Beacon Score');
        
            xlabel('Time Step');
            ylabel('Value');
            title([num2str(nb_host_car) '-> Trust and Score Logs Over Time for car '  num2str(nb_target_car)],"LineWidth",1);
            legend show;
            grid on;
            
            hold off;
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Not Use %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        % function sigma2_matrix = get_adaptive_covariance(self, host_vehicle, target_vehicle)
        %     % Compute adaptive covariance matrix based on past estimation differences
        %     history_length = min(length(self.trust_history), 50); % Use the last 50 time steps
        %     if history_length < 2
        %         sigma2_matrix = eye(size(host_vehicle.observer.est_global_state_current, 1)); % Default to identity matrix
        %         return;
        %     end

        %     diff_history = []; % Store state differences
        %     for t = length(self.trust_history) - history_length + 1 : length(self.trust_history)
        %         diff_t = self.trust_history{t}.(target_vehicle.vehicle_number) - self.trust_history{t}.(host_vehicle.vehicle_number);
        %         diff_history = [diff_history, diff_t];
        %     end

        %     sigma2_matrix = cov(diff_history'); % Compute covariance from history

        %     % Ensure it's positive definite
        %     if rcond(sigma2_matrix) < 1e-6
        %         sigma2_matrix = sigma2_matrix + 1e-3 * eye(size(sigma2_matrix));
        %     end
        % end

        % function beta = compute_dynamic_weight(self, host_vehicle, target_vehicle)
        %     % Compute dynamic weight beta based on trust history
        %     history_length = min(length(self.trust_history), 50);
        %     if history_length < 2
        %         beta = 0.5; % Default equal weighting
        %         return;
        %     end

        %     error_sum = 0;
        %     for t = length(self.trust_history) - history_length + 1 : length(self.trust_history)
        %         error_t = norm(self.trust_history{t}.(target_vehicle.vehicle_number) - self.trust_history{t}.(host_vehicle.vehicle_number));
        %         error_sum = error_sum + error_t;
        %     end

        %     avg_error = error_sum / history_length;
        %     beta = exp(-avg_error); % Higher error -> lower trust in global estimate
        % end


        % Only compare with the directed neighbor (not all neighbors) , or more specific is the target vehicle
        function gamma_cross = compute_cross_validation_factor(self, host_vehicle, target_vehicle, neighbors)
            % Inputs:
            %   host_vehicle: The vehicle evaluating trust
            %   target_vehicle: The neighbor whose global estimate is being evaluated
            %   neighbors: Array of neighbor vehicle objects
            %
            % Output:
            %   gamma_cross: Trust factor based on cross-validation (0 to 1)

            % Collect global estimates from all neighbors
            num_neighbors = length(neighbors);
            if num_neighbors == 0
                gamma_cross = 1; % Default to full trust if no neighbors
                return;
            end

            % Assume each vehicle's observer provides the global estimate as a matrix
            % Columns represent vehicle states, rows represent state variables (e.g., position, velocity)
            global_estimates = cell(num_neighbors, 1);
            for m = 1:num_neighbors
                % host_vehicle.center_communication.get_global_state(neighbors(m).vehicle_number);
                global_estimates{m} = neighbors(m).observer.est_global_state_current;
            end

            % Get target vehicle's global estimate
            target_global_estimate = target_vehicle.observer.est_global_state_current;
            [state_dim, num_vehicles] = size(target_global_estimate);

            % Compute average estimate across all neighbors for each vehicle
            average_estimate = zeros(state_dim, num_vehicles);
            for m = 1:num_neighbors
                average_estimate = average_estimate + global_estimates{m};
            end
            average_estimate = average_estimate / num_neighbors;

            % Compute total discrepancy D_i,l(k)
            D = 0;
            for j = 1:num_vehicles
                d_j = norm(target_global_estimate(:, j) - average_estimate(:, j))^2;
                D = D + d_j;
            end

            % Compute trust factor
            gamma_cross = exp(-D / self.sigma2);
        end

        function following_distance = determine_following_distance(~, trust_score, ds, dacc)
            if trust_score > 0.8
                following_distance = ds;
            elseif trust_score > 0.2
                following_distance = ds + (dacc - ds) * (0.8 - trust_score);
            else
                following_distance = dacc;
            end
        end
    end
end