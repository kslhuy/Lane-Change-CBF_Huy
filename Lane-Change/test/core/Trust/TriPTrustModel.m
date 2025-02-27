classdef TriPTrustModel < handle
    
    
    %{
    Overview how of this ChatGPT
  https://discord.com/channels/1123389035713400902/1326223031667789885/1327306982218141727
    %}
    
    properties
        wv = 4.0;  % Weight for velocity
        wd = 1.0;  % Weight for distance
        wa = 2.0;  % Weight for acceleration
        wj = 1.0;  % Weight for jerkiness
        wt = 0.5; % Trust decay weight
        C = 0.2;   % Regularization constant
        tacc = 1.2;% Trust-based acceleration scaling factor
        k = 5;     % Number of trust levels
        rating_vector; % Trust rating vector
        
        % New properties for global estimate checks
        sigma2 = 1; % Sensitivity parameter for cross-validation trust factor
        tau2 = 0.5;   % Sensitivity parameter for local consistency trust factor
    end
    
    methods
        function self = TriPTrustModel()
            
            % Initialize trust rating vector R_y is accumulates all past outcomes element of vector r_y^x
            % R_y = [R_y(1) ,R_y(2), R_y(3) , R_y(4) , R_y(5)]
            % 1 2 3 4 5 is each trust level
            
            self.rating_vector = zeros(1, self.k);
            self.rating_vector(5) = 1;
            
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
            d_score = max(1 - abs(d_y - d_measured), 0);
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
            
            leader_velocity = leader_vehicle.observer.est_local_state_current(4);
            leader_acceleration = leader_vehicle.input(1);
            
            % Measurement part host (radar, lidar  )
            host_velocity = host_vehicle.observer.est_local_state_current(4);
            host_acceleration = host_vehicle.input(1);
            % if (host_id - neighbors_ids) < 0  =>  host is Front , else = Behind
            
            host_distance_measurement = (host_id - target_id)*(target_vehicle.observer.est_local_state_current(1) - host_vehicle.observer.est_local_state_current(1)) - half_lenght_vehicle;
            
            
            %% ---
            % position host - pos neighbors
            reported_distance = (host_id - target_id)*(target_vehicle.observer.est_local_state_current(1) - host_vehicle.observer.est_local_state_current(1)) - half_lenght_vehicle;
            reported_velocity = target_vehicle.observer.est_local_state_current(4);
            reported_acceleration = target_vehicle.input(1);
            
            % Trust evaluation
            v_score = self.evaluate_velocity( host_id , target_id , host_velocity , reported_velocity, leader_velocity, leader_acceleration, leader_beacon_interval,0.1);
            d_score = self.evaluate_distance(reported_distance, host_distance_measurement);
            
            
            a_score = self.evaluate_acceleration(reported_acceleration, host_acceleration, [host_distance_measurement, reported_distance], time_step);
            beacon_score = self.evaluate_beacon_timeout(true);
            
            
            trust_sample = self.calculate_trust_sample(v_score, d_score, a_score, beacon_score);
            
            % Compute global estimate trust factors
            gamma_cross = self.compute_cross_validation_factor(host_vehicle, target_vehicle, neighbors);
            % gamma_local = self.compute_local_consistency_factor(host_vehicle, target_vehicle, local_measurements);
            
            % Compute extended trust sample
            % trust_sample_ext = trust_sample * gamma_cross * gamma_local;

            trust_sample_ext = trust_sample * gamma_cross ;

            self.update_rating_vector(trust_sample_ext);
            final_score = self.calculate_trust_score();
        end
        
        
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
        
        
        function gamma_local = compute_local_consistency_factor(self, host_vehicle, target_vehicle, local_measurements)
            % Inputs:
            %   host_vehicle: The vehicle evaluating trust
            %   target_vehicle: The neighbor whose global estimate is being evaluated
            %   local_measurements: Struct with sensor data (e.g., relative state to predecessor)
            %
            % Output:
            %   gamma_local: Trust factor based on local consistency (0 to 1)
            
            % Assume local_measurements provides relative state to predecessor
            predecessor = host_vehicle.predecessor; % Vehicle object or empty if none
            if isempty(predecessor) || isempty(local_measurements)
                gamma_local = 1; % Default to full trust if no measurements
                return;
            end
            
            % Get local measurement (e.g., relative position and velocity to predecessor)
            % Assume local_measurements.predecessor is a vector [rel_pos; rel_vel]
            y_i_pred = local_measurements.predecessor;
            
            % Get target vehicle's global estimate
            target_global_estimate = target_vehicle.observer.get_global_estimate();
            
            % Map vehicle IDs to column indices (assuming columns ordered by vehicle_number)
            host_id = host_vehicle.vehicle_number;
            pred_id = predecessor.vehicle_number;
            % Adjust indices if vehicle_number starts at 1
            x_l_pred = target_global_estimate(:, pred_id);
            x_l_i = target_global_estimate(:, host_id);
            
            % Compute expected relative state from global estimate
            rel_state_est = x_l_pred - x_l_i;
            
            % Compute consistency error e_i,l^(j)(k)
            e = norm(rel_state_est - y_i_pred)^2;
            
            % Total error E_i,l(k) = e since M_i = {predecessor}
            E = e;
            
            % Compute trust factor
            gamma_local = exp(-E / self.tau2);
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