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
        wt = 0.85; % Trust decay weight
        C = 0.2;   % Regularization constant
        tacc = 1.2;% Trust-based acceleration scaling factor
        k = 5;     % Number of trust levels
        rating_vector; % Trust rating vector
    end

    methods
        function obj = TriPTrustModel()

            % Initialize trust rating vector R_y is accumulates all past outcomes element of vector r_y^x
            % R_y = [R_y(1) ,R_y(2), R_y(3) , R_y(4) , R_y(5)]
            % 1 2 3 4 5 is each trust level

            obj.rating_vector = zeros(1, obj.k); 
            obj.rating_vector(5) = 1;

        end

        function v_score = evaluate_velocity(~, v_y, v_leader, a_leader, b_leader , tolerance)
            v_ref = v_leader + b_leader * a_leader;
            % if v_ref > 0
            %     v_score = max(1 - abs(v_y - v_ref) / v_ref, 0);
            % else
            %     v_score = max(1 - abs(v_y), 0);
            % end

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
            if nargin < 6
                tolerance = 0.1; % Default tolerance if not provided
            end
            
            % Meaning that leader is move back (brake) or stop 
            if v_ref <= 0
                % Handle edge case for v_ref <= 0
                % TODO : why abs(v_y)
                % Exemple v_y positive , try to run , so score is 0
                % v_y negative , try to brake , so is ?????
                v_score =  max(1 - abs(v_y), 0);
                return;
            end
            % Calculate absolute deviation as a fraction of reference velocity
            % meaning report vehicle is faster or slower than reference velocity
            deviation = abs(v_y - v_ref) / v_ref;
    
            if deviation <= tolerance
                % Within tolerance, give max score
                v_score = 1.0;
            else
                % Scale penalty for deviations beyond tolerance
                scaled_penalty = (deviation - tolerance) / (1 - tolerance);
                v_score =  max(1 - scaled_penalty, 0);
            end
        end

        function d_score = evaluate_distance(~, d_y, d_measured)
            d_score = max(1 - abs(d_y - d_measured), 0);
        end

        function a_score = evaluate_acceleration(~, a_y, a_host, d, ts)
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

        function trust_sample = calculate_trust_sample_w_Jek(obj, v_score, d_score, a_score, j_score, beacon_score)
            trust_sample = beacon_score * (v_score^obj.wv) * (d_score^obj.wd) * (a_score^obj.wa) * (j_score^obj.wj);
        end
        
        function trust_sample = calculate_trust_sample(obj, v_score, d_score, a_score, beacon_score)
            trust_sample = beacon_score * (v_score^obj.wv) * (d_score^obj.wd) * (a_score^obj.wa) ;
        end

        function obj = update_rating_vector(obj, trust_sample)
            %  Map trust sample to a specific trust level in the rating vector

            trust_level = min(round(trust_sample * (obj.k - 1) + 1)  , (obj.k) );
            trust_vector = zeros(1, obj.k); % trust_vector = r_y^x
            trust_vector(trust_level) = 1;

            % Compute current trust score (sigma_y) to use in lambda_y calculation

            current_trust_score = obj.calculate_trust_score();
            % Define lambda_y as per equation (9): lambda_y = sigma_y * w_t
            lambda_y = current_trust_score * obj.wt;

            % Update the rating vector (R_y) using the aging factor lambda_y
            obj.rating_vector = (1 - lambda_y) * obj.rating_vector + trust_vector;
            % obj.rating_vector
            % lambda_y
        end

        function trust_score = calculate_trust_score(obj)
            %Explain : https://discord.com/channels/1123389035713400902/1327310432381435914/1327403771663224873

            % Normalize the rating vector to ensure it represents probabilities
            % a = (1/obj.k) = 1/5 = 0.2 in  obj.C / obj.k
            S_y = (obj.rating_vector + obj.C / obj.k) / (obj.C + sum(obj.rating_vector));
        
            % Define weights with a small epsilon to avoid zero weight for the lowest level
            epsilon = 0.01; % Small positive constant
            weights = ((0:(obj.k-1)) + epsilon) / (obj.k-1 + epsilon);
        
            % Calculate the trust score as a weighted average
            trust_score = sum(weights .* S_y);
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