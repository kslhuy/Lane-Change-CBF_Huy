classdef TriPTrustModel < handle


    %{
    Overview how of this ChatGPT
  https://discord.com/channels/1123389035713400902/1326223031667789885/1327306982218141727
    %}

    properties
        wv = 12;  % Weight for velocity
        wd = 2.5;  % Weight for distance
        wa = 0.3;  % Weight for acceleration
        wj = 1.0;  % Weight for jerkiness
        wh = 1.0;  % Weight for heading



        wv_nearby = 12;  % Weight for velocity
        wd_nearby = 8;  % Weight for distance
        wa_nearby = 0.3;  % Weight for acceleration
        wj_nearby = 1.0;  % Weight for jerkiness
        wh_nearby = 1.0;  % Weight for heading



        % ---- New properties for weighted based trust sample calculation
        w_lt_v = 0.35; % Weight for velocity in weighted trust sample
        w_lt_d = 0.35; % Weight for distance in weighted trust sample
        w_lt_a = 0.3; % Weight for acceleration in weighted trust sample

        % ---- Python local trust model parameters
        py_weight_velocity = 3.0;
        py_weight_distance = 2.0;
        py_weight_acceleration = 1.0;
        py_weight_heading = 1.0;

        local_trust_fusion_mode = "product"; % "product", "equal_geometric", or "weighted_geometric"
        local_weight_velocity = 0.30;
        local_weight_distance = 0.20;
        local_weight_acceleration = 0.15;
        local_weight_heading = 0.15;
        local_weight_beacon = 0.10;
        local_weight_quality = 0.10;

        stationary_velocity_threshold = 0.2;
        velocity_tolerance = 0.3;
        min_velocity_tolerance = 0.05;
        turn_velocity_tolerance_gain = 0.3;
        accel_velocity_tolerance_gain = 0.25;
        stationary_noise_tolerance = 0.15;
        active_trust_tolerance_scale = 1.0;

        acceleration_base_tolerance = 1.0;
        acceleration_speed_tolerance_gain = 0.15;
        acceleration_host_tolerance_gain = 0.6;
        acceleration_turn_tolerance_gain = 0.8;
        acceleration_distance_base_tolerance = 0.35;
        acceleration_distance_turn_gain = 0.4;
        acceleration_rel_velocity_tolerance = 0.2;

        heading_min_movement_m = 0.05;
        heading_base_tolerance_rad = 0.35;
        heading_turn_tolerance_gain = 1.0;
        heading_yaw_rate_tolerance = 0.8;

        distance_physical_violation_ratio = 1.5;
        distance_source_switch_grace_ratio = 3.0;
        severe_v2v_distance_violation_ratio = 2.5;
        severe_v2v_distance_score_cap = 0.05;
        local_pose_distance_tolerance = 0.5;
        local_pose_distance_relative_tolerance = 0.1;
        severe_local_pose_distance_ratio = 2.0;

        % ---- Parameters for trust evolution model 
        wt = 0.4; % Python dirichlet_wt_local
        wt_global = 0.5; % Trust decay weight

        C = 0.2;   % Regularization constant
        tacc = 1.2;% Trust-based acceleration scaling factor
        k = 5;     % Number of trust levels
        rating_vector; % Trust rating vector
        rating_vector_global;


        %% Score components
        alpha_v_score = 0.7; % Weight for velocity score


        self_trust_threshold = 0.5; % Threshold for self-consistency


        % New properties for global estimate checks
        use_python_global_trust = true; % Use Python-compatible global trust calculation
        distributed_trust_fallback = 0.2; % Fallback trust score when distributed trust cannot be calculated
        distributed_trust_state_indices = 1:5;
        distributed_trust_contribution_caps = [4.0, 4.0, 1.5, 2.0, 0.2];
        distributed_trust_accel_weight = 0.05;
        distributed_self_turn_distance_gain = 2.0;
        distributed_self_turn_velocity_gain = 1.0;
        % lower covariance = stronger penalty for same mismatch
        distributed_trust_covariance_diag = [1.5, 1.0, 1.8, 0.5, 0.25];
        distributed_local_tau2_diag = [1.5, 0.5];
        use_relative_velocity_in_relative_trust = true;
        theta_similarity_distance_scale = 1.5;
        theta_similarity_velocity_scale = 1.0;
        theta_similarity_gain = 2.5;
        theta_turn_gain = 2.0;
        theta_contribution_cap = 3.0;
        use_relative_bearing_in_gamma_self = true;
        gamma_self_bearing_tau2 = 0.25;
        distributed_self_tau2_diag = [];
        gamma_self_penalty_floor = 0.4;
        gamma_self_penalty_exponent = 0.9;
        ema_alpha = 0.5;
        previous_final_score = NaN;
        latest_target_turn_context = 0.0;
        sigma2 = 1; % Sensitivity parameter for cross-validation trust factor
        tau2 = 0.5;   % Sensitivity parameter for local consistency trust factor
        last_d;
        last_time_d = 0; % Last time step when distance was updated
        Period_a_score_distane = 10; % Time step for the simulation
        buffer_size = 5; % Number of time steps for moving average (e.g., n=5)
        distance_buffer ; % Initialize buffer

        trust_sample_log;
        gamma_cross_log;
        gamma_local_log;
        gamma_local_our_self_log;
        gamma_expected_log;
        v_score_log;
        d_score_log;
        a_score_log;
        h_score_log;
        beacon_score_log;
        final_score_log;

        flag_glob_est_check_log ;
        flag_taget_attk_log ;
        flag_local_est_check_log ;

        flag_taget_attk = false;
        flag_glob_est_check = false;

        flag_local_est_check = false;
        lead_state_lastest = [];
        lead_state_lastest_timestamp = 0;

        D_pos_log = [];          % Log for position discrepancies
        D_vel_log = [];          % Log for velocity discrepancies
        D_theta_log = [];        % Log for heading discrepancies
        anomaly_pos_log = [];    % Log for position anomaly flags
        anomaly_vel_log = [];    % Log for velocity anomaly flags
        anomaly_gamma_log = [];  % Already included from previous request

        % Parameters for anomaly detection
        w = 10;  % Sliding window size
        Threshold_anomalie  = 3;   % Threshold for cumulative anomalies
        reduce_factor = 0.5;  % Trust reduction factor

        previous_state;
        tau2_matrix_gamma_local = []; % Diagonal matrix for local consistency factor
        sigma2_matrix_gamma_cross = []; % Diagonal matrix for cross-host trust factor

        v_rel_log = []; % Log for relative velocity
        d_add_log = []; % Log for additional distance
        acc_rel_log = []; % Log for relative acceleration
        delta_acc_expected_log = []; % Log for expected acceleration difference
        distance_log = []; % Log for distance measurements

        scale_d_expected_log = []; % Scale factor for expected distance
        delta_d_log = []; % Log for delta distance (measured - epxected )

        a_score_expected_diff_acc_log = []; % Log for expected acceleration score
        a_score_vrel_dis_adjusted_log = []; % Log for acceleration difference score
        a_score_vrel_dis_log = []; % Log for relative velocity and distance score
        a_score_defaut_log = []; % Log for default acceleration score
        a_score_mathematical_log = []; % Log for mathematical formula acceleration score
        
        % Missing properties for anomaly detection
        D_acc_log = []; % Log for acceleration discrepancies
        D_total_log = []; % Log for total global Mahalanobis discrepancy
        anomaly_acc_log = []; % Log for acceleration anomaly flags

        % Trust decay parameters
        lambda_h = 0.8; % Trust decay factor when no beacon received, one missing packet keeps 80% of previous trust
        previous_trust_scores; % Store previous trust scores per vehicle
        
        % Physical constraints parameters
        MAX_ACCEL = 4.0; % Maximum acceleration (m/s²)
        MAX_DECEL = -8.0; % Maximum deceleration (m/s²)
        MAX_VELOCITY = 30.0; % Maximum velocity (m/s)
        MAX_JERK = 8.0; % Maximum jerk (m/s^3)
        temporal_pos_tolerance_m = 0.5;
        temporal_vel_tolerance = 0.5;
        
        % Temporal consistency tracking
        previous_states_map; % Store previous states per vehicle
        local_score_previous_states_map; % Python-style local score state history
        local_score_previous_host_state = [];
        local_score_current_host_state = [];
        local_score_host_instant_idx = -Inf;
        temporal_score_log = []; % Log for temporal consistency scores
        physical_valid_log = []; % Log for physical constraints validation
        
        % Trust decay logging
        local_trust_decayed_log = []; % Log for local trust after decay
        global_trust_decayed_log = []; % Log for global trust after decay
        
        % Separate beacon score tracking
        beacon_score_local_log = []; % Log for local channel beacon reception
        beacon_score_global_log = []; % Log for global channel beacon reception

        % ---------- Filtering System Properties ----------
        % Filter enable/disable flags
        enable_score_filtering = false; % Master switch for all filtering
        enable_velocity_filter = true;  % Enable velocity score filtering
        enable_distance_filter = true;  % Enable distance score filtering
        enable_acceleration_filter = true; % Enable acceleration score filtering
        enable_beacon_filter = true;    % Enable beacon score filtering
        enable_heading_filter = true;   % Enable heading score filtering
        
        % Special real-time acceleration filtering (independent of master switch)
        enable_realtime_acceleration_filter = true; % Enable real-time acceleration filtering only
        
        % Filter types for each score
        velocity_filter_type = 'moving_average';    % 'moving_average', 'median', 'exponential', 'threshold', 'none'
        distance_filter_type = 'moving_average';    % 'moving_average', 'median', 'exponential', 'threshold', 'none'
        acceleration_filter_type = 'median';       % 'moving_average', 'median', 'exponential', 'threshold', 'none'
        beacon_filter_type = 'threshold';          % 'moving_average', 'median', 'exponential', 'threshold', 'none'
        heading_filter_type = 'moving_average';    % 'moving_average', 'median', 'exponential', 'threshold', 'none'
        
        % Filter parameters
        filter_window_size = 5;         % Window size for moving average and median filters
        filter_alpha = 0.7;             % Alpha parameter for exponential filter (0 < alpha < 1)
        filter_threshold_min = 0.1;     % Minimum threshold for threshold filter
        filter_threshold_max = 0.9;     % Maximum threshold for threshold filter
        
        % Filter buffers for each score type
        velocity_score_buffer = [];
        distance_score_buffer = [];
        acceleration_score_buffer = [];
        beacon_score_buffer = [];
        heading_score_buffer = [];
        
        % Filtered score logs
        filtered_v_score_log = [];
        filtered_d_score_log = [];
        filtered_a_score_log = [];
        filtered_beacon_score_log = [];
        filtered_h_score_log = [];
        
        % Previous filtered values for exponential filtering
        prev_filtered_v_score = 1.0;
        prev_filtered_d_score = 1.0;
        prev_filtered_a_score = 1.0;
        prev_filtered_beacon_score = 1.0;
        prev_filtered_h_score = 1.0;


    end

    methods
        function self = TriPTrustModel(scenarios_config)

            if nargin >= 1 && ~isempty(scenarios_config)
                self.apply_scenario_config(scenarios_config);
            end

            % Initialize trust rating vector R_y is accumulates all past outcomes element of vector r_y^x
            % R_y = [R_y(1) ,R_y(2), R_y(3) , R_y(4) , R_y(5)]
            % 1 2 3 4 5 is each trust level

            self.rating_vector = zeros(1, self.k);

            % for global bal
            self.rating_vector_global = zeros(1, self.k);


            self.last_d = 20;
            self.last_time_d = 0; % Last time step when distance was updated

            self.buffer_size = 5; % Number of time steps for moving average (e.g., n=5)
            self.distance_buffer = zeros(1, self.buffer_size); % Initialize buffer

            % Initialize logs for trust samples and scores
            self.trust_sample_log = [];
            self.gamma_cross_log = [];
            self.gamma_local_log = [];
            self.gamma_local_our_self_log = [];
            self.v_score_log = [];
            self.d_score_log = [];
            self.a_score_log = [];
            self.beacon_score_log = [];
            self.final_score_log = [];

            self.lead_state_lastest = zeros(5,1);
            self.previous_state = zeros(4,1);

            tau2_diag_element = self.distributed_local_tau2_diag;
            self.tau2_matrix_gamma_local = diag(tau2_diag_element);

            % Compute covariance matrix (adaptive variance)
            sigma2_diag_element = self.distributed_trust_covariance_diag;
            self.sigma2_matrix_gamma_cross = diag(sigma2_diag_element);


            self.v_rel_log = [];
            self.acc_rel_log = [];
            self.d_add_log = [];
            self.delta_acc_expected_log = [];



            % % Parameters for anomaly detection
            % self.w = 10;  % Sliding window size
            % self.Threshold_anomalie  = 3;   % Threshold for cumulative anomalies
            % self.reduce_factor = 0.5;  % Trust reduction factor
            
            % Initialize separate arrays for local and global trust decay (much faster than maps)
            self.previous_trust_scores = struct('local', [], 'global', []); % Separate arrays for local and global trust
            self.previous_states_map = containers.Map('KeyType', 'int32', 'ValueType', 'any');
            self.local_score_previous_states_map = containers.Map('KeyType', 'int32', 'ValueType', 'any');
            self.local_score_previous_host_state = [];
            self.local_score_current_host_state = [];
            self.local_score_host_instant_idx = -Inf;
            
            % Initialize filter buffers
            self.velocity_score_buffer = [];
            self.distance_score_buffer = [];
            self.acceleration_score_buffer = [];
            self.beacon_score_buffer = [];
            self.heading_score_buffer = [];
            
            % Initialize filtered score logs
            self.filtered_v_score_log = [];
            self.filtered_d_score_log = [];
            self.filtered_a_score_log = [];
            self.filtered_beacon_score_log = [];
            self.filtered_h_score_log = [];
            
            % Enable real-time acceleration filtering by default
            self.enable_realtime_acceleration_filter = true;
        end

        function apply_scenario_config(self, cfg)
            % Copy the scenario's active trust configuration into this
            % per-target model. This is intentionally explicit so independent
            % batch scenarios cannot share or silently ignore trust settings.
            same_names = [ ...
                "py_weight_velocity", "py_weight_distance", ...
                "py_weight_acceleration", "py_weight_heading", ...
                "local_trust_fusion_mode", "local_weight_velocity", ...
                "local_weight_distance", "local_weight_acceleration", ...
                "local_weight_heading", "local_weight_beacon", ...
                "local_weight_quality", "stationary_velocity_threshold", ...
                "velocity_tolerance", "min_velocity_tolerance", ...
                "turn_velocity_tolerance_gain", "accel_velocity_tolerance_gain", ...
                "stationary_noise_tolerance", "acceleration_base_tolerance", ...
                "acceleration_speed_tolerance_gain", "acceleration_host_tolerance_gain", ...
                "acceleration_turn_tolerance_gain", "acceleration_distance_base_tolerance", ...
                "acceleration_distance_turn_gain", "acceleration_rel_velocity_tolerance", ...
                "heading_min_movement_m", "heading_base_tolerance_rad", ...
                "heading_turn_tolerance_gain", "heading_yaw_rate_tolerance", ...
                "theta_similarity_distance_scale", "theta_similarity_velocity_scale", ...
                "theta_similarity_gain", "theta_turn_gain", "theta_contribution_cap", ...
                "ema_alpha", "distributed_trust_fallback", ...
                "distributed_trust_state_indices", ...
                "distributed_trust_contribution_caps", ...
                "distributed_trust_accel_weight", ...
                "distributed_trust_covariance_diag", ...
                "distributed_local_tau2_diag", ...
                "distributed_self_turn_distance_gain", ...
                "distributed_self_turn_velocity_gain", ...
                "use_relative_velocity_in_relative_trust", ...
                "use_relative_bearing_in_gamma_self", "gamma_self_bearing_tau2", ...
                "distributed_self_tau2_diag", "gamma_self_penalty_floor", ...
                "gamma_self_penalty_exponent", "temporal_pos_tolerance_m", ...
                "temporal_vel_tolerance"];

            for config_name = same_names
                field_name = char(config_name);
                if isstruct(cfg) && isfield(cfg, field_name)
                    self.(field_name) = cfg.(field_name);
                elseif isobject(cfg) && isprop(cfg, field_name)
                    self.(field_name) = cfg.(field_name);
                end
            end

            special_names = { ...
                'dirichlet_wt_local', 'wt'; ...
                'dirichlet_wt_global', 'wt_global'; ...
                'dirichlet_C', 'C'; ...
                'num_trust_levels', 'k'; ...
                'trust_decay_lambda', 'lambda_h'; ...
                'max_velocity', 'MAX_VELOCITY'; ...
                'max_acceleration', 'MAX_ACCEL'; ...
                'max_deceleration', 'MAX_DECEL'; ...
                'max_jerk', 'MAX_JERK'; ...
                'Use_python_global_trust', 'use_python_global_trust'};
            for config_idx = 1:size(special_names, 1)
                source_name = special_names{config_idx, 1};
                target_name = special_names{config_idx, 2};
                if isstruct(cfg) && isfield(cfg, source_name)
                    self.(target_name) = cfg.(source_name);
                elseif isobject(cfg) && isprop(cfg, source_name)
                    self.(target_name) = cfg.(source_name);
                end
            end

            self.k = max(1, round(double(self.k)));
            self.lambda_h = min(1.0, max(0.0, double(self.lambda_h)));
            self.self_trust_threshold = self.clamp_unit(self.get_config_numeric( ...
                cfg, 'trust_threshold', self.self_trust_threshold));
        end

        function v_score_final_with_exp = evaluate_velocity(self,host_id , target_id, v_y, v_host, v_leader, a_leader, b_leader , is_nearby ,tolerance)
            if nargin < 9
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
            if (host_id - target_id) > 0 % Target is ahead host
                alpha = self.alpha_v_score;
            else
                alpha = 1- self.alpha_v_score; % Target is follwing host
            end

            % Meaning that leader is move back (brake) or stop
            if v_ref == 0 || v_leader == 0 || sign(v_ref*v_leader) < 0 || sign(v_ref*v_host) < 0
                % Handle edge case for v_ref <= 0
                % TODO : why abs(v_y)
                % Exemple v_y positive , try to run , so score is 0
                % v_y negative , try to brake , so is ?????
                v_score_final_with_exp =  max(1 - abs(v_y), 0);
                return;
            end
            % Calculate absolute deviation as a fraction of reference velocity
            % meaning report vehicle is faster or slower than reference velocity
            deviation_ref = (abs(v_y - v_ref) + 1) / v_ref;
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
            if (is_nearby)
                v_score_final_with_exp = v_score_final^self.wv_nearby ;
            else
                v_score_final_with_exp = v_score_final^self.wv ;
            end

        end

        function d_score = evaluate_distance(self, d_y, d_measured , is_nearby)
            if is_nearby
                % For nearby vehicles, use a stricter distance evaluation
                d_score = (max(1 - abs((d_y - d_measured)/d_measured), 0))^self.wd_nearby;
            else
                % For distant vehicles, allow more tolerance
                % TODO : need to change the distance measure for the far away vehicle
                % d_y is the reported distance , d_measured is the measured distance
                % so if the reported distance is close to the measured distance, then score is high
                % d_score = 0;
                d_score = max(1 - abs((d_y - d_measured)/d_measured), 0)^self.wd;
            end
        end

        function a_score = evaluate_acceleration(self,host_vehicle, host_id,target_id,a_y, a_host, d, ts, is_nearby)
            % Update distance buffer
            self.distance_buffer = [d(1), self.distance_buffer(1:end-1)]; % Shift and add new distance

            % Mathematical formulation parameters
            a_th = 1.0;      % Nominal acceleration tolerance  - increased for less sensitivity
            d_norm = 25;     % Normalization constant for typical spacing - increased from 19
            v_norm = 23;     % Normalization constant for typical speed (m/s) - new parameter

            % Compute relative velocity using distance difference over n time steps
            %% Relative acceleration calculation - implements v_rel from math formula
            if all(self.distance_buffer ~= 0) % Ensure buffer is filled
                v_rel_current = (self.distance_buffer(end-1) - self.distance_buffer(1)) / ((self.buffer_size-1) * ts);
                v_rel_previous = (self.distance_buffer(end) - self.distance_buffer(2)) / ((self.buffer_size-1) * ts);
                % Expected relative acceleration: a_expect^rel = (v_rel(t) - v_rel(t-n))/n
                expected_a_relative_diff  = (v_rel_current - v_rel_previous)/(ts);
                v_rel = v_rel_current;
            else
                % v_rel = 0; % Default if buffer not yet filled
                v_rel = diff(d) / ts;
                expected_a_relative_diff = 0;
            end

            %% Calculate accelerations according to mathematical formula
            % a_recv^rel = â₀^(l) - â₀^(i) (received relative acceleration)
            a_recv_rel = a_y - a_host;
            
            % a_expect^rel = (v_rel(t) - v_rel(t-n))/n (expected relative acceleration)
            a_expect_rel = expected_a_relative_diff;
            
            % Inter-vehicle distance
            d_i_l = d(1);
            
            % Calculate mismatch with mathematical normalization
            accel_mismatch = abs(a_recv_rel - a_expect_rel);
            
            % Mathematical normalization: a_th * (1 + d_i_l/d_norm) * (1 + |v_rel|/v_norm)
            normalization_factor = a_th * (1 + d_i_l/d_norm) * (1 + abs(v_rel)/v_norm);
            
            % Calculate mathematical acceleration score
            a_score_mathematical = max(1 - (accel_mismatch / normalization_factor), 0);


            %% Keep your enhanced implementation as alternative (with reduced sensitivity)
            constance_regulation = 1; % depending on the sign of the acceleration difference , and position index

            if  ~is_nearby
                d_add = d_norm*abs(target_id-host_id)  / (d(1)); % Added +10 to reduce sensitivity
            else
                % id_diff = host_id - target_id;
                % 0.9*(target_id-1)
                if target_id > 1
                    % Calculate sign mismatch
                    Signe_mismatch_relative = (sign(a_host) ~= sign(a_recv_rel));
                    % Update constance regulation based on sign mismatch
                    if (Signe_mismatch_relative == 0)
                        % Sign consistent
                        constance_regulation = 0.7;
                    else
                        % Sign inconsistent
                        constance_regulation = 0.8;
                    end

                    % Calculate additional distance based on target and host IDs
                    d_add = d_norm*constance_regulation / (d(1) ); % Added +5 to reduce sensitivity
                    % if (target_id > host_id)
                    %     d_add = d_norm*constance_regulation / (d(1));
                    % else
                    %     d_add = d_norm / (d(1));
                    % end
                else
                    % special case : target_id <= 1 and host_id = 2
                    d_add = d_norm*constance_regulation / (d(1) ); % Added +5 to reduce sensitivity
                end

                % d_add = 1; % Default value for distant vehicles
            end


            %%% Calculate distance expected depending  Host ID , gamma mix distance
            hi = host_vehicle.Param_opt.hi ; %% Time gap ( T )
            ri = host_vehicle.Param_opt.ri; % Minimum gap distance ( s0 )
            v_local_host = host_vehicle.observer.est_local_state_current(4);
            v_0 = host_vehicle.Param_opt.v0; % Desired velocity in free flow
            delta = host_vehicle.Param_opt.delta;
            if host_vehicle.scenarios_config.controller_type == "mix"
                s_CACC_expected = ri + hi * v_local_host/(host_id-1); % Expected spacing based on CACC model
                s_IDM_expected  = ri + hi * v_local_host / (sqrt(1 - (v_local_host/v_0)^delta));
                gamma_control = host_vehicle.gamma; % Control parameter for Mix controller
                d_expected = (1 - gamma_control)*s_CACC_expected + (gamma_control) * s_IDM_expected;

            elseif host_vehicle.scenarios_config.controller_type == "local"
                d_expected = (ri + hi*v_local_host) /(sqrt(1 - (v_local_host/v_0)^delta ));

            else %"coop"
                d_expected = ri + hi * v_local_host/(host_id-1); % Expected spacing based on CACC model
            end

            % Compute expected spacing using the mixing formula:

            scale_d_expected = d(1) /d_expected; % Normalize expected distance by current distance

            %%% NEW: Adjust d_add based on scenario flags



            delta_d = d(1) - d_expected;

            % Define scenario flags
            target_decelerating = a_y < 0;
            host_decelerating = a_host < 0;
            both_accelerating = a_y > 0 && a_host > 0;
            both_decelerating = a_y < 0 && a_host < 0;
            target_acc_host_dec = a_y > 0 && a_host < 0;

            % Initialize d_add_scale
            d_add_scale = 1;

            % Apply scenario-based adjustments to d_add_scale
            if target_decelerating
                d_add_scale = d_add_scale * 0.8; % Penalize when target is decelerating
            end
            if both_decelerating && delta_d < 0
                d_add_scale = d_add_scale * 0.7; % Stronger penalty when both are decelerating and distance is too small
            end
            if target_acc_host_dec && delta_d < 0
                d_add_scale = d_add_scale * 0.8; % Penalize when target is accelerating, host is decelerating, and distance is too small
            end
            if (both_accelerating || both_decelerating) && abs(delta_d) > 0.2 * d_expected
                d_add_scale = d_add_scale * 0.9; % Mild penalty for significant distance deviation
            end

            % Compute adjusted d_add
            d_add_adjusted = d_add * d_add_scale;


            %%% Calculate the acceleration score

            %% Enhanced implementation (your original with reduced sensitivity)
            delta_acc = a_host + expected_a_relative_diff - a_y; % Expected vs actual
            a_score_expected_diff_acc = max(1 - abs( d_add_adjusted * delta_acc), 0);

            %% Alternative implementations
            a_score_defaut = max(1 - abs(v_rel / ((self.buffer_size - 1)  * ts ) * a_recv_rel), 0); % Original Trip default score

            a_score_vrel_dis_adjusted =  max(1 - abs(v_rel /d_add_adjusted * a_recv_rel), 0); % Using adjusted additional distance

            a_score_vrel_dis_real = max(1 - abs(v_rel / (d(1) ) * a_recv_rel), 0);  % Using real distance without adjustment

            % Choose which acceleration score to use based on method selector:
            switch host_vehicle.scenarios_config.acceleration_trust_score_method
                case 'mathematical'
                    % Mathematical formula (most faithful to your equations)
                    a_score = a_score_mathematical;
                case 'enhanced'
                    % Enhanced implementation (less sensitive)
                    a_score = a_score_expected_diff_acc;
                case 'hybrid'
                    % Hybrid approach (combine both with weights)
                    a_score = 0.6 * a_score_mathematical + 0.4 * a_score_expected_diff_acc;
                case 'default'
                    a_score = a_score_defaut;
                case 'vrel_dis_adjusted'
                    a_score = a_score_vrel_dis_adjusted;
                case 'vrel_dis_real'
                    a_score = a_score_vrel_dis_real;
                otherwise
                    a_score = a_score_defaut;
                    % Default to enhanced
            end

            % Apply weighting based on proximity
            if is_nearby
                a_score = a_score^self.wa_nearby; % Tune wa_nearby
            else
                a_score = a_score^self.wa; % Tune wa
            end


            %log
            self.v_rel_log = [self.v_rel_log, v_rel]; % Log relative velocity
            self.distance_log = [self.distance_log, d(1)]; % Log distance
            self.d_add_log = [self.d_add_log, d_add_adjusted]; % Log additional distance
            self.acc_rel_log = [self.acc_rel_log, a_recv_rel]; % Log relative acceleration
            self.delta_acc_expected_log = [self.delta_acc_expected_log, delta_acc]; % Log expected acceleration difference
            self.scale_d_expected_log = [self.scale_d_expected_log, scale_d_expected]; % Log scale factor for expected distance
            self.delta_d_log = [self.delta_d_log, delta_d]; % Log delta distance (measured - expected)

            self.a_score_expected_diff_acc_log = [self.a_score_expected_diff_acc_log, a_score_expected_diff_acc];
            self.a_score_vrel_dis_adjusted_log = [self.a_score_vrel_dis_adjusted_log, a_score_vrel_dis_adjusted];
            self.a_score_vrel_dis_log = [self.a_score_vrel_dis_log, a_score_vrel_dis_real];
            self.a_score_defaut_log = [self.a_score_defaut_log, a_score_defaut];
            
            % Add logging for mathematical acceleration score
            self.a_score_mathematical_log = [self.a_score_mathematical_log, a_score_mathematical];
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



        function h_score = evaluate_heading(self, target_pos_X, target_pos_Y, reported_heading, instant_idx)
            % evaluate_heading - Computes trust score for target's reported heading
            %
            % Inputs:
            %   host_vehicle       - Host vehicle object
            %   target_vehicle     - Target vehicle object
            %   target_id          - Target vehicle ID
            %   host_id            - Host vehicle ID
            %   target_pos_X       - Target's current X position
            %   target_pos_Y       - Target's current Y position
            %   reported_heading   - Target's reported heading (radians)
            %   instant_idx        - Current simulation time step
            %
            % Output:
            %   h_score            - Heading trust score (0 to 1)

            % Retrieve previous position from target's state history
            if isequal(self.previous_state, zeros(4,1))
                h_score = 1; % Cannot compute heading without previous position
                self.previous_state = [target_pos_X, target_pos_Y, 0, 0]; % Initialize with current position
                return;
            else

                % Extract previous position
                prev_target_pos_X = self.previous_state(1);
                prev_target_pos_Y = self.previous_state(2);

                % Step 2: Calculate estimated heading (theta_est)
                delta_Y = target_pos_Y - prev_target_pos_Y;
                delta_X = target_pos_X - prev_target_pos_X;
                theta_est = atan2(delta_Y, delta_X); % Estimated heading in radians

                % Step 3: Compare with reported heading
                theta_diff = abs(reported_heading - theta_est);
                % Ensure smallest angle difference (handle circular nature of angles)
                theta_diff = min(theta_diff, 2 * pi - theta_diff);

                % Step 4: Compute trust value
                theta_max = pi / 18; % 10 degrees threshold
                h_score = max(1 - theta_diff / theta_max, 0);
                self.previous_state = [target_pos_X, target_pos_Y, 0, 0]; % Initialize with current position

            end
        end

        %%% Python-style local trust scoring helpers %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function value = clamp_unit(~, value)
            if ~isfinite(value)
                value = 0;
            end
            value = max(0, min(1, value));
        end

        function angle_wrapped = wrap_angle(~, angle_value)
            angle_wrapped = atan2(sin(angle_value), cos(angle_value));
        end

        function score = robust_score(self, error_value, tolerance)
            tol = max(abs(tolerance), 1e-3);
            ratio = abs(error_value) / tol;
            score = self.clamp_unit(1.0 / (1.0 + ratio * ratio));
        end

        function state = normalize_local_state(~, state)
            state = state(:);
            if length(state) < 5
                state = [state; zeros(5 - length(state), 1)];
            end
        end

        function update_local_host_score_state(self, host_state, instant_idx)
            host_state = self.normalize_local_state(host_state);
            if isempty(self.local_score_current_host_state) || self.local_score_host_instant_idx ~= instant_idx
                if ~isempty(self.local_score_current_host_state)
                    self.local_score_previous_host_state = self.local_score_current_host_state;
                end
                self.local_score_host_instant_idx = instant_idx;
            end
            self.local_score_current_host_state = struct( ...
                'state', host_state, ...
                'instant_idx', instant_idx);
        end

        function entry = make_local_score_entry(self, state, instant_idx, distance_from_host, relative_velocity_from_host, distance_is_measured)
            entry = struct( ...
                'state', self.normalize_local_state(state), ...
                'instant_idx', instant_idx, ...
                'distance_from_host', distance_from_host, ...
                'relative_velocity_from_host', relative_velocity_from_host, ...
                'distance_is_measured', logical(distance_is_measured));
        end

        function [entry, has_entry] = get_local_score_previous_target(self, target_id)
            key = int32(target_id);
            has_entry = isKey(self.local_score_previous_states_map, key);
            if has_entry
                entry = self.local_score_previous_states_map(key);
            else
                entry = [];
            end
        end

        function store_local_score_target_state(self, target_id, entry)
            self.local_score_previous_states_map(int32(target_id)) = entry;
        end

        function dt = local_entry_dt(~, previous_entry, current_entry, default_dt)
            dt = default_dt;
            if ~isempty(previous_entry) && ~isempty(current_entry) && ...
                    isfield(previous_entry, 'instant_idx') && isfield(current_entry, 'instant_idx')
                instant_delta = current_entry.instant_idx - previous_entry.instant_idx;
                if isfinite(instant_delta) && instant_delta > 0
                    dt = instant_delta * default_dt;
                end
            end
            dt = max(dt, 0.01);
        end

        function yaw_rate = yaw_rate_from_entries(self, previous_entry, current_entry, default_dt)
            yaw_rate = 0.0;
            if isempty(previous_entry) || isempty(current_entry)
                return;
            end
            prev_state = previous_entry.state;
            curr_state = current_entry.state;
            dt = self.local_entry_dt(previous_entry, current_entry, default_dt);
            theta_delta = self.wrap_angle(curr_state(3) - prev_state(3));
            yaw_rate = abs(theta_delta) / dt;
        end

        function v_rel = estimate_radial_relative_velocity(self, host_state, target_state)
            host_state = self.normalize_local_state(host_state);
            target_state = self.normalize_local_state(target_state);

            los = [target_state(1) - host_state(1); target_state(2) - host_state(2)];
            los_norm = norm(los);
            if los_norm <= 1e-6
                v_rel = target_state(4) - host_state(4);
                return;
            end

            los_hat = los / los_norm;
            target_velocity_xy = [target_state(4) * cos(target_state(3)); target_state(4) * sin(target_state(3))];
            host_velocity_xy = [host_state(4) * cos(host_state(3)); host_state(4) * sin(host_state(3))];
            v_rel = dot(target_velocity_xy - host_velocity_xy, los_hat);
        end

        function [distance_value, is_measured] = resolve_relative_distance(self, host_state, target_state, measured_distance)
            if isfinite(measured_distance) && measured_distance > 0
                distance_value = max(measured_distance, 0.1);
                is_measured = true;
                return;
            end

            host_state = self.normalize_local_state(host_state);
            target_state = self.normalize_local_state(target_state);
            distance_value = max(norm(target_state(1:2) - host_state(1:2)), 0.1);
            is_measured = false;
        end

        function relative_velocity = resolve_relative_velocity(self, host_state, target_state, measured_relative_velocity)
            if nargin >= 4 && isfinite(measured_relative_velocity)
                relative_velocity = measured_relative_velocity;
            else
                relative_velocity = self.estimate_radial_relative_velocity(host_state, target_state);
            end
        end

        function limit_value = relative_acceleration_limit(self)
            limit_value = 2.0 * max(abs(self.MAX_ACCEL), abs(self.MAX_DECEL));
        end

        function score = evaluate_velocity_python(self, host_state, target_entry, leader_state, previous_target, default_dt)
            target_state = target_entry.state;
            v_target = target_state(4);

            base_tolerance = max([self.velocity_tolerance, self.min_velocity_tolerance, 0.01]);
            tolerance_scale = max(1.0, self.active_trust_tolerance_scale);
            turn_bonus = 0.0;
            dt = max(default_dt, 0.01);
            if ~isempty(previous_target)
                dt = self.local_entry_dt(previous_target, target_entry, default_dt);
                heading_delta = self.wrap_angle(target_state(3) - previous_target.state(3));
                yaw_rate = abs(heading_delta) / dt;
                speed_scale = max([abs(v_target), abs(host_state(4)), 0.3]);
                turn_bonus = self.turn_velocity_tolerance_gain * yaw_rate * speed_scale;
            end
            accel_bonus = self.accel_velocity_tolerance_gain * max(abs(target_state(5)), abs(host_state(5)));
            v_tolerance = max(base_tolerance + turn_bonus + accel_bonus, 0.01) * tolerance_scale;

            context_refs = host_state(4);
            if ~isempty(leader_state) && length(leader_state) >= 4 && isfinite(leader_state(4))
                context_refs = [context_refs; leader_state(4)];
            end
            context_refs = context_refs(isfinite(context_refs));

            if ~isempty(previous_target)
                previous_velocity = previous_target.state(4);
                previous_acceleration = 0.0;
                if length(previous_target.state) >= 5 && isfinite(previous_target.state(5))
                    previous_acceleration = previous_target.state(5);
                end
                if isfinite(previous_velocity)
                    expected_velocity = previous_velocity + ...
                        0.5 * (previous_acceleration + target_state(5)) * dt;
                    temporal_score = self.robust_score(v_target - expected_velocity, v_tolerance);
                else
                    temporal_score = NaN;
                end
            else
                temporal_score = NaN;
            end

            if isempty(context_refs)
                score = 1.0;
                return;
            end

            v_ref = median(context_refs);

            if abs(v_target) < self.stationary_velocity_threshold && abs(v_ref) < self.stationary_velocity_threshold
                v_error = abs(v_target - v_ref);
                if v_error < self.stationary_noise_tolerance
                    score = 1.0;
                else
                    normalized_error = v_error / self.stationary_velocity_threshold;
                    score = self.clamp_unit((max(1.0 - normalized_error, 0.0)) ^ self.py_weight_velocity);
                end
                return;
            end

            context_tolerance = max(2.0 * v_tolerance, 0.01);
            context_score = self.robust_score(v_target - v_ref, context_tolerance);
            if isempty(previous_target) || ~isfinite(temporal_score)
                score = context_score;
                return;
            end

            % Follower targets can legitimately lag the host during transients.
            target_is_behind_host = target_state(1) < host_state(1);
            if target_is_behind_host
                temporal_weight = 0.75;
            else
                temporal_weight = 0.60;
            end
            score = temporal_weight * temporal_score + (1.0 - temporal_weight) * context_score;
            score = self.clamp_unit(score);
        end

        function [score, severe_mismatch] = evaluate_distance_python(self, host_state, target_entry, previous_target, previous_host, default_dt)
            severe_mismatch = false;
            if isempty(previous_target)
                score = 1.0;
                return;
            end

            d_current = target_entry.distance_from_host;
            current_is_measured = target_entry.distance_is_measured;
            prev_is_measured = previous_target.distance_is_measured;
            source_switched = current_is_measured ~= prev_is_measured;

            if prev_is_measured && isfinite(previous_target.distance_from_host) && previous_target.distance_from_host > 0
                d_prev = previous_target.distance_from_host;
            else
                if ~isempty(previous_host)
                    prev_host_state = previous_host.state;
                else
                    prev_host_state = host_state;
                end
                d_prev = norm(previous_target.state(1:2) - prev_host_state(1:2));
            end
            d_prev = max(d_prev, 0.1);

            v_rel = target_entry.relative_velocity_from_host;
            if isfinite(previous_target.relative_velocity_from_host)
                v_rel_prev = previous_target.relative_velocity_from_host;
            elseif ~isempty(previous_host)
                v_rel_prev = self.estimate_radial_relative_velocity(previous_host.state, previous_target.state);
            else
                v_rel_prev = self.estimate_radial_relative_velocity(host_state, previous_target.state);
            end

            dt = self.local_entry_dt(previous_target, target_entry, default_dt);
            v_rel_robust = 0.5 * (v_rel + v_rel_prev);
            d_expected = d_prev + v_rel_robust * dt;

            d_measured = max(d_current, 0.1);
            d_error = abs(d_current - d_expected);
            actual_change = abs(d_current - d_prev);
            tolerance_scale = max(1.0, self.active_trust_tolerance_scale);
            max_phys_change = abs(v_rel_prev) * dt + 0.5 * self.relative_acceleration_limit() * (dt ^ 2);
            max_phys_change = max(max_phys_change, 0.5 * self.stationary_noise_tolerance);
            max_phys_change = max_phys_change * tolerance_scale;
            if ~current_is_measured && ~prev_is_measured && ~source_switched
                % Non-nearby distance is inferred from exchanged pose estimates, not
                % directly measured. Use a wider per-step gate so normal estimator
                % noise does not collapse the distance score for a single sample.
                estimated_distance_noise_floor = max([ ...
                    self.local_pose_distance_tolerance, ...
                    2.0 * self.stationary_noise_tolerance, ...
                    0.1]);
                max_phys_change = max(max_phys_change, estimated_distance_noise_floor);
            end

            violation_ratio = self.distance_physical_violation_ratio;
            if source_switched
                violation_ratio = max(violation_ratio, self.distance_source_switch_grace_ratio);
            end

            violation_detected = actual_change > max_phys_change * violation_ratio;
            severe_v2v_violation = ~current_is_measured && ~prev_is_measured && ~source_switched && ...
                actual_change > max_phys_change * self.severe_v2v_distance_violation_ratio;

            if violation_detected
                if current_is_measured || prev_is_measured || source_switched
                    penalty_factor = 1.25;
                else
                    penalty_factor = 3.0;
                end
                d_error = max(d_error, actual_change * penalty_factor);
            end

            normalized_error = d_error / (d_measured * tolerance_scale);
            score = self.clamp_unit((max(1.0 - normalized_error, 0.0)) ^ self.py_weight_distance);
            if severe_v2v_violation
                score = min(score, self.severe_v2v_distance_score_cap);
            end
        end

        function [score, severe_mismatch] = evaluate_local_pose_distance_python(self, host_state, target_entry)
            severe_mismatch = false;
            if ~target_entry.distance_is_measured
                score = 1.0;
                return;
            end

            measured_distance = max(target_entry.distance_from_host, 0.1);
            reported_distance = norm(target_entry.state(1:2) - host_state(1:2));
            if ~isfinite(reported_distance)
                score = 1.0;
                return;
            end

            pose_error = abs(reported_distance - measured_distance);
            pose_tolerance = max([ ...
                self.local_pose_distance_tolerance, ...
                self.local_pose_distance_relative_tolerance * measured_distance, ...
                0.5 * self.stationary_noise_tolerance]);
            tolerance_scale = max(1.0, self.active_trust_tolerance_scale);
            pose_tolerance = pose_tolerance * tolerance_scale;
            excess_error = max(pose_error - pose_tolerance, 0.0);
            normalized_error = excess_error / max([measured_distance * tolerance_scale, pose_tolerance, 0.1]);
            score = self.clamp_unit((max(1.0 - normalized_error, 0.0)) ^ self.py_weight_distance);

            severe_mismatch = pose_error > max( ...
                pose_tolerance * self.severe_local_pose_distance_ratio, ...
                pose_tolerance + self.stationary_noise_tolerance);
            if severe_mismatch
                score = min(score, self.severe_v2v_distance_score_cap);
            end
        end

        function score = evaluate_acceleration_python(self, host_state, target_entry, previous_target, previous_host, current_host, default_dt)
            target_state = target_entry.state;
            if isempty(previous_target)
                score = 1.0;
                return;
            end

            a_target = target_state(5);
            a_host = host_state(5);
            v_target = target_state(4);
            v_host = host_state(4);
            dt = self.local_entry_dt(previous_target, target_entry, default_dt);
            tolerance_scale = max(1.0, self.active_trust_tolerance_scale);

            if abs(v_target) < self.stationary_velocity_threshold && abs(v_host) < self.stationary_velocity_threshold
                a_error = abs(a_target - a_host);
                noise_tolerance = max(0.35, 3.0 * self.stationary_noise_tolerance) * tolerance_scale;
                stationary_score = self.robust_score(a_error, noise_tolerance);
                score = self.clamp_unit(stationary_score ^ max(min(self.py_weight_acceleration, 3.0), 0.1));
                return;
            end

            target_yaw_rate = self.yaw_rate_from_entries(previous_target, target_entry, default_dt);
            host_yaw_rate = self.yaw_rate_from_entries(previous_host, current_host, default_dt);
            combined_yaw_rate = max(target_yaw_rate, host_yaw_rate);

            v_rel_now = target_entry.relative_velocity_from_host;
            if isfinite(previous_target.relative_velocity_from_host)
                v_rel_prev = previous_target.relative_velocity_from_host;
            elseif ~isempty(previous_host)
                v_rel_prev = self.estimate_radial_relative_velocity(previous_host.state, previous_target.state);
            else
                v_rel_prev = self.estimate_radial_relative_velocity(host_state, previous_target.state);
            end

            a_rel_reported = a_target - a_host;
            a_from_velocity = (v_target - previous_target.state(4)) / dt;
            a_error = a_target - a_from_velocity;
            a_tol = self.acceleration_base_tolerance + ...
                self.acceleration_speed_tolerance_gain * max([abs(v_target), abs(v_host), abs(v_rel_now), abs(v_rel_prev)]) + ...
                self.acceleration_host_tolerance_gain * abs(a_host) + ...
                self.acceleration_turn_tolerance_gain * combined_yaw_rate * max(abs(v_target), 0.2);
            a_tol = a_tol * tolerance_scale;
            score_temporal = self.robust_score(a_error, a_tol);

            if previous_target.distance_is_measured && isfinite(previous_target.distance_from_host) && previous_target.distance_from_host > 0
                d_prev = previous_target.distance_from_host;
            else
                if ~isempty(previous_host)
                    prev_host_state = previous_host.state;
                else
                    prev_host_state = host_state;
                end
                d_prev = norm(previous_target.state(1:2) - prev_host_state(1:2));
            end
            d_prev = max(d_prev, 0.1);
            d_curr = max(target_entry.distance_from_host, 0.1);

            d_pred = d_prev + v_rel_prev * dt + 0.5 * a_rel_reported * (dt ^ 2);
            d_error = d_curr - d_pred;
            d_tol = self.acceleration_distance_base_tolerance + ...
                0.25 * max(abs(v_rel_prev), abs(v_rel_now)) * dt + ...
                self.acceleration_distance_turn_gain * combined_yaw_rate * max(d_curr, 0.5) * dt;
            d_tol = d_tol * tolerance_scale;
            score_distance = self.robust_score(d_error, d_tol);

            v_rel_measured = (d_curr - d_prev) / dt;
            v_error = v_rel_now - v_rel_measured;
            v_tol = max(self.acceleration_rel_velocity_tolerance, 0.75 * self.velocity_tolerance) + ...
                0.2 * combined_yaw_rate * max(d_curr, 0.5);
            v_tol = v_tol * tolerance_scale;
            score_rel_velocity = self.robust_score(v_error, v_tol);

            combined = 0.45 * score_temporal + 0.35 * score_distance + 0.20 * score_rel_velocity;
            score = self.clamp_unit(combined ^ max(min(self.py_weight_acceleration, 3.0), 0.1));
        end

        function score = evaluate_heading_python(self, host_state, target_entry, previous_target, previous_host, current_host, default_dt)
            if isempty(previous_target)
                score = 1.0;
                return;
            end

            target_state = target_entry.state;
            theta_reported = target_state(3);
            theta_host = host_state(3);

            delta_x = target_state(1) - previous_target.state(1);
            delta_y = target_state(2) - previous_target.state(2);
            movement = hypot(delta_x, delta_y);

            heading_delta_deadband = 0.25 * self.heading_base_tolerance_rad;
            dt_target = self.local_entry_dt(previous_target, target_entry, default_dt);
            target_heading_delta = self.wrap_angle(target_state(3) - previous_target.state(3));
            target_yaw_rate = max(abs(target_heading_delta) - heading_delta_deadband, 0.0) / dt_target;
            host_yaw_rate = 0.0;
            if ~isempty(previous_host) && ~isempty(current_host)
                dt_host = self.local_entry_dt(previous_host, current_host, default_dt);
                host_heading_delta = self.wrap_angle(current_host.state(3) - previous_host.state(3));
                host_yaw_rate = max(abs(host_heading_delta) - heading_delta_deadband, 0.0) / dt_host;
            end
            turn_context = max(target_yaw_rate, host_yaw_rate);
            turn_factor = self.clamp_unit(turn_context / 0.8);
            tolerance_scale = max(1.0, self.active_trust_tolerance_scale);

            heading_tol = (self.heading_base_tolerance_rad + self.heading_turn_tolerance_gain * turn_factor) * tolerance_scale;
            score_abs = self.robust_score(self.wrap_angle(theta_reported - theta_host), heading_tol);

            motion_heading_min_distance = max(self.heading_min_movement_m, 1.0);
            if movement >= motion_heading_min_distance
                theta_from_motion = atan2(delta_y, delta_x);
                motion_tol = (self.heading_base_tolerance_rad + 0.5 * self.heading_turn_tolerance_gain * turn_factor) * tolerance_scale;
                score_motion = self.robust_score(self.wrap_angle(theta_reported - theta_from_motion), motion_tol);
            else
                score_motion = 1.0;
            end

            yaw_rate_tol = self.heading_yaw_rate_tolerance * (1.0 + 0.5 * turn_factor) * tolerance_scale;
            score_path = self.robust_score(target_yaw_rate - host_yaw_rate, yaw_rate_tol);

            w_abs = 0.45 - 0.30 * turn_factor;
            w_motion = 0.35;
            w_path = 1.0 - w_abs - w_motion;
            combined = w_abs * score_abs + w_motion * score_motion + w_path * score_path;
            score = self.clamp_unit(combined ^ max(min(self.py_weight_heading, 3.0), 0.2));
        end

        function [v_score, d_score, a_score, h_score, severe_local_pose_mismatch] = evaluate_python_local_scores(self, host_state, target_state, leader_state, target_id, measured_distance, instant_idx, default_dt)
            host_state = self.normalize_local_state(host_state);
            target_state = self.normalize_local_state(target_state);
            self.update_local_host_score_state(host_state, instant_idx);

            [previous_target, has_previous_target] = self.get_local_score_previous_target(target_id);
            if ~has_previous_target
                previous_target = [];
            end

            [distance_current, distance_is_measured] = self.resolve_relative_distance(host_state, target_state, measured_distance);
            relative_velocity_current = self.resolve_relative_velocity(host_state, target_state, NaN);
            current_target = self.make_local_score_entry( ...
                target_state, instant_idx, distance_current, relative_velocity_current, distance_is_measured);

            current_host = self.local_score_current_host_state;
            previous_host = self.local_score_previous_host_state;

            target_yaw_rate = self.yaw_rate_from_entries(previous_target, current_target, default_dt);
            host_yaw_rate = self.yaw_rate_from_entries(previous_host, current_host, default_dt);
            self.latest_target_turn_context = max(target_yaw_rate, host_yaw_rate);

            v_score = self.evaluate_velocity_python(host_state, current_target, leader_state, previous_target, default_dt);
            [distance_score, ~] = self.evaluate_distance_python(host_state, current_target, previous_target, previous_host, default_dt);
            [pose_distance_score, severe_local_pose_mismatch] = self.evaluate_local_pose_distance_python(host_state, current_target);
            d_score = min(distance_score, pose_distance_score);
            a_score = self.evaluate_acceleration_python(host_state, current_target, previous_target, previous_host, current_host, default_dt);
            h_score = self.evaluate_heading_python(host_state, current_target, previous_target, previous_host, current_host, default_dt);

            self.store_local_score_target_state(target_id, current_target);
        end

        %%% Python-style global trust scoring helpers %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function value = get_config_bool(~, cfg, field_name, default_value)
            value = default_value;
            if isempty(cfg)
                return;
            end

            has_value = false;
            raw_value = [];
            if isstruct(cfg) && isfield(cfg, field_name)
                raw_value = cfg.(field_name);
                has_value = true;
            elseif isobject(cfg) && isprop(cfg, field_name)
                raw_value = cfg.(field_name);
                has_value = true;
            end

            if ~has_value
                return;
            end

            if islogical(raw_value)
                value = raw_value;
            elseif isnumeric(raw_value)
                value = raw_value ~= 0;
            elseif isstring(raw_value) || ischar(raw_value)
                value = any(strcmpi(string(raw_value), ["true", "1", "yes", "on"]));
            end
        end

        function value = get_config_numeric(~, cfg, field_name, default_value)
            value = default_value;
            if isempty(cfg)
                return;
            end

            has_value = false;
            raw_value = [];
            if isstruct(cfg) && isfield(cfg, field_name)
                raw_value = cfg.(field_name);
                has_value = true;
            elseif isobject(cfg) && isprop(cfg, field_name)
                raw_value = cfg.(field_name);
                has_value = true;
            end

            if ~has_value || isempty(raw_value) || ~isnumeric(raw_value) || ~isfinite(raw_value(1))
                return;
            end

            value = raw_value(1);
        end

        function use_python = should_use_python_global_trust(self, host_vehicle)
            use_python = self.use_python_global_trust;
            if isprop(host_vehicle, 'scenarios_config')
                use_python = self.get_config_bool( ...
                    host_vehicle.scenarios_config, ...
                    'Use_python_global_trust', ...
                    use_python);
            end
        end

        function gamma = distance_to_gamma_python(self, distance_value, ~)
            if ~isfinite(distance_value)
                gamma = 0.0;
                return;
            end

            distance_value = max(double(distance_value), 0.0);
            gamma = exp(-distance_value);
            gamma = self.clamp_unit(gamma);
        end

        function state = state_to_vector_python(self, state)
            if isempty(state)
                state = [];
                return;
            end

            state = double(state(:));
            if isempty(state)
                return;
            end

            if length(state) < 5
                state = [state; zeros(5 - length(state), 1)];
            elseif length(state) > 5
                state = state(1:5);
            end
        end

        function valid = is_valid_state_vector(self, state)
            state = self.state_to_vector_python(state);
            valid = ~isempty(state) && length(state) >= 5 && all(isfinite(state));
        end

        function y = compute_relative_measurement_python(self, host_state, target_state, measured_distance, measured_relative_velocity)
            host_state = self.normalize_local_state(host_state);
            target_state = self.normalize_local_state(target_state);

            [relative_distance, ~] = self.resolve_relative_distance(host_state, target_state, measured_distance);
            if ~self.use_relative_velocity_in_relative_trust
                y = relative_distance;
                return;
            end

            relative_velocity = self.resolve_relative_velocity(host_state, target_state, measured_relative_velocity);
            y = [relative_distance; relative_velocity];
        end

        function y = compute_relative_from_estimates_python(self, est_host, est_target)
            est_host = self.state_to_vector_python(est_host);
            est_target = self.state_to_vector_python(est_target);

            relative_distance = max(norm(est_target(1:2) - est_host(1:2)), 0.1);
            if ~self.use_relative_velocity_in_relative_trust
                y = relative_distance;
                return;
            end

            relative_velocity = self.estimate_radial_relative_velocity(est_host, est_target);
            y = [relative_distance; relative_velocity];
        end

        function values = pad_or_trim_vector(~, values, n, default_value)
            values = double(values(:));
            if isempty(values)
                values = default_value * ones(n, 1);
                return;
            end

            if length(values) < n
                values = [values; values(end) * ones(n - length(values), 1)];
            elseif length(values) > n
                values = values(1:n);
            end
        end

        function [diff_vec, inv_diag] = prepare_mahalanobis_terms_python(self, x1, x2, yaw_rate)
            x1 = self.state_to_vector_python(x1);
            x2 = self.state_to_vector_python(x2);
            n = min(length(x1), length(x2));
            x1 = x1(1:n);
            x2 = x2(1:n);

            diff_vec = x1 - x2;
            if n >= 3
                diff_vec(3) = self.wrap_angle(diff_vec(3));
            end

            diag_values = self.pad_or_trim_vector(self.distributed_trust_covariance_diag, n, 1.0);

            if n >= 3
                xy_distance = norm(diff_vec(1:min(2, n)));
                if n >= 4
                    velocity_diff = abs(diff_vec(4));
                else
                    velocity_diff = 0.0;
                end

                d_scale = max(self.theta_similarity_distance_scale, 1e-3);
                v_scale = max(self.theta_similarity_velocity_scale, 1e-3);
                similarity = exp(-((xy_distance / d_scale)^2 + (velocity_diff / v_scale)^2));
                theta_gain = 1.0 ...
                    + self.theta_similarity_gain * similarity ...
                    + self.theta_turn_gain * max(yaw_rate, 0.0);
                diag_values(3) = max(diag_values(3) * theta_gain, 1e-6);
            end

            inv_diag = 1.0 ./ max(diag_values, 1e-6);
        end

        function [total_distance, contributions] = mahalanobis_components_python(self, x1, x2, yaw_rate)
            [diff_vec, inv_diag] = self.prepare_mahalanobis_terms_python(x1, x2, yaw_rate);
            n = length(diff_vec);
            contributions = zeros(n, 1);

            active_indices = self.distributed_trust_state_indices;
            if isempty(active_indices)
                active_indices = 1:n;
            end

            for idx = active_indices
                idx = round(idx);
                if idx >= 1 && idx <= n
                    contributions(idx) = (diff_vec(idx)^2) * inv_diag(idx);
                end
            end

            if n >= 5
                accel_weight = max(0.0, min(1.0, self.distributed_trust_accel_weight));
                contributions(5) = contributions(5) * accel_weight;
            end

            caps = self.pad_or_trim_vector(self.distributed_trust_contribution_caps, n, Inf);
            for idx = 1:n
                if isfinite(caps(idx)) && caps(idx) >= 0.0
                    contributions(idx) = min(contributions(idx), caps(idx));
                end
            end

            if n >= 3 && self.theta_contribution_cap > 0.0
                contributions(3) = min(contributions(3), self.theta_contribution_cap);
            end

            total_distance = sum(contributions);
        end

        function distance_value = relative_mahalanobis_python(self, y_measured, y_estimated, yaw_rate, distance_turn_gain, velocity_turn_gain, tau2_override, angle_indices, velocity_index)
            y_measured = double(y_measured(:));
            y_estimated = double(y_estimated(:));
            n = min(length(y_measured), length(y_estimated));
            if n <= 0
                distance_value = 0.0;
                return;
            end

            y_measured = y_measured(1:n);
            y_estimated = y_estimated(1:n);

            if nargin < 7 || isempty(tau2_override)
                tau2_diag = self.pad_or_trim_vector(self.distributed_local_tau2_diag, n, 1.0);
            else
                tau2_diag = self.pad_or_trim_vector(tau2_override, n, 1.0);
            end
            if nargin < 8 || isempty(angle_indices)
                angle_indices = [];
            end
            if nargin < 9
                if self.use_relative_velocity_in_relative_trust
                    velocity_index = 2;
                else
                    velocity_index = [];
                end
            end
            yaw_rate = max(yaw_rate, 0.0);
            if n >= 1 && distance_turn_gain > 0.0 && yaw_rate > 0.0
                tau2_diag(1) = tau2_diag(1) * (1.0 + distance_turn_gain * yaw_rate);
            end
            if ~isempty(velocity_index) && velocity_index >= 1 && velocity_index <= n && ...
                    velocity_turn_gain > 0.0 && yaw_rate > 0.0
                tau2_diag(velocity_index) = tau2_diag(velocity_index) * ...
                    (1.0 + velocity_turn_gain * yaw_rate);
            end

            residual = y_estimated - y_measured;
            for angle_idx = angle_indices(:)'
                if angle_idx >= 1 && angle_idx <= n
                    residual(angle_idx) = self.wrap_angle(residual(angle_idx));
                end
            end
            distance_value = sum((residual .^ 2) ./ max(tau2_diag, 1e-9));
        end

        function penalty = compute_gamma_self_penalty(self, gamma_self, threshold)
            gamma_self = self.clamp_unit(gamma_self);
            threshold = max(threshold, 1e-6);
            floor_value = self.clamp_unit(self.gamma_self_penalty_floor);
            exponent_value = max(self.gamma_self_penalty_exponent, 1e-6);

            if gamma_self >= threshold
                penalty = 1.0;
                return;
            end

            ratio = gamma_self / threshold;
            penalty = floor_value + (1.0 - floor_value) * (ratio ^ exponent_value);
            penalty = max(floor_value, min(1.0, penalty));
        end

        function turn_context = compute_turn_context_python(self, host_vehicle)
            turn_context = max(0.0, self.latest_target_turn_context);
            if isempty(self.local_score_previous_host_state) || isempty(self.local_score_current_host_state)
                return;
            end

            previous_host = self.local_score_previous_host_state;
            current_host = self.local_score_current_host_state;
            if ~isfield(previous_host, 'state') || ~isfield(current_host, 'state')
                return;
            end

            if isfield(previous_host, 'instant_idx') && isfield(current_host, 'instant_idx') && isprop(host_vehicle, 'dt')
                dt = (current_host.instant_idx - previous_host.instant_idx) * host_vehicle.dt;
            elseif isprop(host_vehicle, 'dt')
                dt = host_vehicle.dt;
            else
                dt = 0.01;
            end
            dt = max(dt, 0.01);

            theta_delta = self.wrap_angle(current_host.state(3) - previous_host.state(3));
            host_yaw_rate = abs(theta_delta) / dt;
            turn_context = max(turn_context, host_yaw_rate);
        end

        function [bearing, available, source_name] = resolve_clean_relative_bearing_python(self, host_vehicle, target_id, host_state)
            bearing = NaN;
            available = false;
            source_name = "";
            if isempty(host_vehicle) || ~isprop(host_vehicle, 'center_communication')
                return;
            end

            try
                clean_target = host_vehicle.center_communication.get_clean_local_state(target_id);
            catch
                clean_target = [];
            end
            clean_target = clean_target(:);
            host_state = self.normalize_local_state(host_state);
            if length(clean_target) < 2 || any(~isfinite(clean_target(1:2))) || ...
                    any(~isfinite(host_state(1:3)))
                return;
            end

            dx = clean_target(1) - host_state(1);
            dy = clean_target(2) - host_state(2);
            source_name = "v2v_clean";
            if hypot(dx, dy) <= 1e-9
                return;
            end
            if ~self.use_relative_bearing_in_gamma_self
                return;
            end
            bearing = self.wrap_angle(atan2(dy, dx) - host_state(3));
            available = true;
        end

        function tau2_diag = gamma_self_tau2_diag_python(self, include_bearing)
            if ~isempty(self.distributed_self_tau2_diag)
                tau2_diag = double(self.distributed_self_tau2_diag(:));
                return;
            end

            local_tau = double(self.distributed_local_tau2_diag(:));
            if isempty(local_tau)
                local_tau = ones(2, 1);
            end
            tau2_diag = local_tau(1);
            if self.use_relative_velocity_in_relative_trust
                tau2_diag(end + 1, 1) = local_tau(min(2, length(local_tau)));
            end
            if include_bearing
                tau2_diag(end + 1, 1) = max(double(self.gamma_self_bearing_tau2), 1e-9);
            end
        end

        function apply_penalty = should_apply_gamma_self_penalty(~, source_name)
            source_name = lower(strtrim(string(source_name)));
            apply_penalty = ~startsWith(source_name, "v2v_clean");
        end

        function [global_trust_sample, gamma_cross, gamma_local, gamma_local_our_self, D_pos, D_vel, D_acc, D_theta, D_total] = ...
                compute_global_trust_sample_python(self, host_vehicle, target_vehicle, target_state, host_state, measured_distance, target_global_state)
            fallback = self.clamp_unit(self.distributed_trust_fallback);
            gamma_cross = fallback;
            gamma_local = fallback;
            gamma_local_our_self = fallback;
            global_trust_sample = fallback * fallback;
            D_pos = NaN;
            D_vel = NaN;
            D_acc = NaN;
            D_theta = NaN;
            D_total = NaN;

            host_id = host_vehicle.vehicle_number;
            target_id = target_vehicle.vehicle_number;

            if nargin < 7 || isempty(target_global_state)
                target_global_state = host_vehicle.center_communication.get_global_state(target_id, host_id);
            end

            if isempty(target_global_state) || any(~isfinite(target_global_state(:)))
                gamma_cross = 0.0;
                gamma_local = 0.0;
                gamma_local_our_self = 0.0;
                global_trust_sample = 0.0;
                return;
            end

            host_fleet_estimates = host_vehicle.observer.est_global_state_current;
            if isempty(host_fleet_estimates) || any(~isfinite(host_fleet_estimates(:)))
                return;
            end

            host_state = self.normalize_local_state(host_state);
            target_state = self.normalize_local_state(target_state);
            y_local = self.compute_relative_measurement_python(host_state, target_state, measured_distance, NaN);
            local_relative_dof = max(1, length(y_local));
            turn_context = self.compute_turn_context_python(host_vehicle);
            relative_measurement_source = "";

            if target_id <= size(host_fleet_estimates, 2)
                host_target_estimate = host_fleet_estimates(:, target_id);
                if self.is_valid_state_vector(host_target_estimate)
                    % Python builds a separate gamma_self measurement vector.
                    % A clean bearing must not alter gamma_local's vector or DOF.
                    y_self_measured = y_local;
                    y_self_est = self.compute_relative_measurement_python(host_state, host_target_estimate, NaN, NaN);
                    [clean_bearing, has_clean_bearing, relative_measurement_source] = ...
                        self.resolve_clean_relative_bearing_python( ...
                        host_vehicle, target_id, host_state);
                    angle_indices = [];
                    if has_clean_bearing
                        estimated_bearing = self.wrap_angle( ...
                            atan2(host_target_estimate(2) - host_state(2), ...
                            host_target_estimate(1) - host_state(1)) - host_state(3));
                        y_self_measured(end + 1, 1) = clean_bearing;
                        y_self_est(end + 1, 1) = estimated_bearing;
                        angle_indices = length(y_self_measured);
                    end
                    self_relative_dof = max(1, length(y_self_measured));
                    self_tau2 = self.gamma_self_tau2_diag_python(has_clean_bearing);
                    if self.use_relative_velocity_in_relative_trust
                        velocity_index = 2;
                    else
                        velocity_index = [];
                    end
                    d_self = self.relative_mahalanobis_python( ...
                        y_self_measured, y_self_est, turn_context, ...
                        self.distributed_self_turn_distance_gain, ...
                        self.distributed_self_turn_velocity_gain, ...
                        self_tau2, angle_indices, velocity_index);
                    gamma_local_our_self = self.distance_to_gamma_python(d_self, self_relative_dof);
                end
            end

            d_host_total = 0.0;
            n_host_valid = 0;
            component_sum = zeros(5, 1);
            num_vehicles = min(size(host_fleet_estimates, 2), size(target_global_state, 2));
            for vehicle_idx = 1:num_vehicles
                if vehicle_idx == target_id
                    continue;
                end

                host_vec = self.state_to_vector_python(host_fleet_estimates(:, vehicle_idx));
                target_vec = self.state_to_vector_python(target_global_state(:, vehicle_idx));
                if isempty(host_vec) || isempty(target_vec) || any(~isfinite(host_vec)) || any(~isfinite(target_vec))
                    continue;
                end
                if all(abs(host_vec) < 1e-12)
                    continue;
                end

                [vehicle_distance, contributions] = self.mahalanobis_components_python(host_vec, target_vec, turn_context);
                d_host_total = d_host_total + vehicle_distance;
                n_host_valid = n_host_valid + 1;
                component_sum(1:length(contributions)) = component_sum(1:length(contributions)) + contributions;
            end

            if n_host_valid > 0
                D_total = d_host_total / n_host_valid;
                component_mean = component_sum / n_host_valid;
                D_pos = sum(component_mean(1:2));
                D_theta = component_mean(3);
                D_vel = component_mean(4);
                D_acc = component_mean(5);
                gamma_cross = self.distance_to_gamma_python(D_total, 5);
            end

            if host_id <= size(target_global_state, 2) && target_id <= size(target_global_state, 2)
                est_host = target_global_state(:, host_id);
                est_target = target_global_state(:, target_id);
                if self.is_valid_state_vector(est_host) && self.is_valid_state_vector(est_target)
                    y_target_relative = self.compute_relative_from_estimates_python(est_host, est_target);
                    d_local = self.relative_mahalanobis_python(y_local, y_target_relative, 0.0, 0.0, 0.0);
                    gamma_local = self.distance_to_gamma_python(d_local, local_relative_dof);
                end
            end

            global_trust_sample = gamma_cross * gamma_local;
            if self.should_apply_gamma_self_penalty(relative_measurement_source) && ...
                    gamma_local_our_self < self.self_trust_threshold
                global_trust_sample = global_trust_sample * self.compute_gamma_self_penalty( ...
                    gamma_local_our_self, self.self_trust_threshold);
            end

            global_trust_sample = self.clamp_unit(global_trust_sample);
        end

        %%% Score Filtering System %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   %%%%%%%%%%%%%%%%%%%
        function [filtered_v_score, filtered_d_score, filtered_a_score, filtered_beacon_score, filtered_h_score] = filter_all_scores(self, v_score, d_score, a_score, beacon_score, h_score)
            % filter_all_scores - Apply configurable filters to all trust scores
            %
            % Inputs:
            %   v_score: Velocity trust score
            %   d_score: Distance trust score  
            %   a_score: Acceleration trust score
            %   beacon_score: Beacon reception score
            %   h_score: Heading trust score
            %
            % Outputs:
            %   filtered_v_score: Filtered velocity score
            %   filtered_d_score: Filtered distance score
            %   filtered_a_score: Filtered acceleration score
            %   filtered_beacon_score: Filtered beacon score
            %   filtered_h_score: Filtered heading score
            
            % Check master filtering switch
            if ~self.enable_score_filtering
                % No filtering - return original scores
                filtered_v_score = v_score;
                filtered_d_score = d_score;
                filtered_a_score = a_score;
                filtered_beacon_score = beacon_score;
                filtered_h_score = h_score;
                return;
            end
            
            % Apply filtering to each score type
            filtered_v_score = self.apply_single_score_filter(v_score, 'velocity');
            filtered_d_score = self.apply_single_score_filter(d_score, 'distance');
            filtered_a_score = self.apply_single_score_filter(a_score, 'acceleration');
            filtered_beacon_score = self.apply_single_score_filter(beacon_score, 'beacon');
            filtered_h_score = self.apply_single_score_filter(h_score, 'heading');
            
            % Log filtered scores
            self.filtered_v_score_log = [self.filtered_v_score_log, filtered_v_score];
            self.filtered_d_score_log = [self.filtered_d_score_log, filtered_d_score];
            self.filtered_a_score_log = [self.filtered_a_score_log, filtered_a_score];
            self.filtered_beacon_score_log = [self.filtered_beacon_score_log, filtered_beacon_score];
            self.filtered_h_score_log = [self.filtered_h_score_log, filtered_h_score];
        end
        
        function filtered_score = apply_single_score_filter(self, raw_score, score_type)
            % apply_single_score_filter - Apply filter to a single score type
            %
            % Inputs:
            %   raw_score: Original unfiltered score
            %   score_type: Type of score ('velocity', 'distance', 'acceleration', 'beacon', 'heading')
            %
            % Output:
            %   filtered_score: Filtered score value
            
            % Get filter configuration for this score type
            [enable_filter, filter_type, buffer] = self.get_filter_config(score_type);
            
            % If filtering is disabled for this score type, return original
            if ~enable_filter
                filtered_score = raw_score;
                return;
            end
            
            % Update buffer with new score
            updated_buffer = self.update_score_buffer(buffer, raw_score, score_type);
            
            % Apply the specified filter
            switch filter_type
                case 'moving_average'
                    filtered_score = self.apply_moving_average_filter(updated_buffer);
                case 'median'
                    filtered_score = self.apply_median_filter(updated_buffer);
                case 'exponential'
                    filtered_score = self.apply_exponential_filter(raw_score, score_type);
                case 'threshold'
                    filtered_score = self.apply_threshold_filter(raw_score);
                case 'adaptive_weighted'
                    filtered_score = self.apply_adaptive_weighted_filter(updated_buffer);
                case 'outlier_rejection'
                    filtered_score = self.apply_outlier_rejection_filter(updated_buffer, raw_score);
                otherwise
                    % Default to no filtering
                    filtered_score = raw_score;
            end
            
            % Ensure filtered score is within valid bounds [0, 1]
            filtered_score = max(0, min(1, filtered_score));
        end
        
        function [enable_filter, filter_type, buffer] = get_filter_config(self, score_type)
            % get_filter_config - Get filter configuration for a specific score type
            
            switch score_type
                case 'velocity'
                    enable_filter = self.enable_velocity_filter;
                    filter_type = self.velocity_filter_type;
                    buffer = self.velocity_score_buffer;
                case 'distance'
                    enable_filter = self.enable_distance_filter;
                    filter_type = self.distance_filter_type;
                    buffer = self.distance_score_buffer;
                case 'acceleration'
                    enable_filter = self.enable_acceleration_filter;
                    filter_type = self.acceleration_filter_type;
                    buffer = self.acceleration_score_buffer;
                case 'beacon'
                    enable_filter = self.enable_beacon_filter;
                    filter_type = self.beacon_filter_type;
                    buffer = self.beacon_score_buffer;
                case 'heading'
                    enable_filter = self.enable_heading_filter;
                    filter_type = self.heading_filter_type;
                    buffer = self.heading_score_buffer;
                otherwise
                    enable_filter = false;
                    filter_type = 'none';
                    buffer = [];
            end
        end
        
        function updated_buffer = update_score_buffer(self, buffer, new_score, score_type)
            % update_score_buffer - Update the score buffer and store it back to the object
            
            % Add new score to buffer
            updated_buffer = [new_score, buffer];
            
            % Limit buffer size
            if length(updated_buffer) > self.filter_window_size
                updated_buffer = updated_buffer(1:self.filter_window_size);
            end
            
            % Store updated buffer back to the object
            switch score_type
                case 'velocity'
                    self.velocity_score_buffer = updated_buffer;
                case 'distance'
                    self.distance_score_buffer = updated_buffer;
                case 'acceleration'
                    self.acceleration_score_buffer = updated_buffer;
                case 'beacon'
                    self.beacon_score_buffer = updated_buffer;
                case 'heading'
                    self.heading_score_buffer = updated_buffer;
            end
        end
        
        function filtered_score = apply_moving_average_filter(self, buffer)
            % apply_moving_average_filter - Apply moving average filter
            if isempty(buffer)
                filtered_score = 1.0; % Default high trust
            else
                filtered_score = mean(buffer);
            end
        end
        
        function filtered_score = apply_median_filter(self, buffer)
            % apply_median_filter - Apply median filter (good for outlier rejection)
            if isempty(buffer)
                filtered_score = 1.0; % Default high trust
            else
                filtered_score = median(buffer);
            end
        end
        
        function filtered_score = apply_exponential_filter(self, raw_score, score_type)
            % apply_exponential_filter - Apply exponential moving average filter
            % filtered_score = alpha * raw_score + (1 - alpha) * previous_filtered_score
            
            % Get previous filtered value
            switch score_type
                case 'velocity'
                    prev_filtered = self.prev_filtered_v_score;
                case 'distance'
                    prev_filtered = self.prev_filtered_d_score;
                case 'acceleration'
                    prev_filtered = self.prev_filtered_a_score;
                case 'beacon'
                    prev_filtered = self.prev_filtered_beacon_score;
                case 'heading'
                    prev_filtered = self.prev_filtered_h_score;
                otherwise
                    prev_filtered = 1.0;
            end
            
            % Apply exponential filter
            filtered_score = self.filter_alpha * raw_score + (1 - self.filter_alpha) * prev_filtered;
            
            % Store filtered value for next iteration
            switch score_type
                case 'velocity'
                    self.prev_filtered_v_score = filtered_score;
                case 'distance'
                    self.prev_filtered_d_score = filtered_score;
                case 'acceleration'
                    self.prev_filtered_a_score = filtered_score;
                case 'beacon'
                    self.prev_filtered_beacon_score = filtered_score;
                case 'heading'
                    self.prev_filtered_h_score = filtered_score;
            end
        end
        
        function filtered_score = apply_threshold_filter(self, raw_score)
            % apply_threshold_filter - Apply threshold-based filter
            % Clips values outside the threshold range
            if raw_score < self.filter_threshold_min
                filtered_score = self.filter_threshold_min;
            elseif raw_score > self.filter_threshold_max
                filtered_score = self.filter_threshold_max;
            else
                filtered_score = raw_score;
            end
        end
        
        function filtered_score = apply_adaptive_weighted_filter(self, buffer)
            % apply_adaptive_weighted_filter - Apply adaptive weighted average
            % Recent values have higher weights
            if isempty(buffer)
                filtered_score = 1.0; % Default high trust
                return;
            end
            
            % Create exponentially decaying weights (most recent = highest weight)
            n = length(buffer);
            weights = exp(-0.3 * (0:(n-1))); % Exponential decay
            weights = weights / sum(weights); % Normalize
            
            % Apply weighted average
            filtered_score = sum(buffer .* weights);
        end
        
        function filtered_score = apply_outlier_rejection_filter(self, buffer, raw_score)
            % apply_outlier_rejection_filter - Filter that rejects outliers based on statistical analysis
            if length(buffer) < 3
                filtered_score = raw_score; % Not enough data for outlier detection
                return;
            end
            
            % Calculate statistics
            buffer_mean = mean(buffer);
            buffer_std = std(buffer);
            
            % Check if current score is an outlier (> 2 standard deviations)
            if abs(raw_score - buffer_mean) > 2 * buffer_std
                % It's an outlier, use median of buffer instead
                filtered_score = median(buffer);
            else
                % Not an outlier, use moving average
                filtered_score = mean(buffer);
            end
        end
        
        %%% Filter Configuration Methods %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   %%%%%%%%%%%%%%%%%%%
        function configure_filtering(self, enable_master, filter_configs)
            % configure_filtering - Configure the filtering system
            %
            % Inputs:
            %   enable_master: Boolean to enable/disable all filtering
            %   filter_configs: Struct with filter configurations for each score type
            %
            % Example usage:
            %   filter_configs.velocity = struct('enable', true, 'type', 'moving_average');
            %   filter_configs.distance = struct('enable', true, 'type', 'median');
            %   trust_model.configure_filtering(true, filter_configs);
            
            self.enable_score_filtering = enable_master;
            
            if nargin > 2 && isstruct(filter_configs)
                % Configure velocity filter
                if isfield(filter_configs, 'velocity')
                    if isfield(filter_configs.velocity, 'enable')
                        self.enable_velocity_filter = filter_configs.velocity.enable;
                    end
                    if isfield(filter_configs.velocity, 'type')
                        self.velocity_filter_type = filter_configs.velocity.type;
                    end
                end
                
                % Configure distance filter
                if isfield(filter_configs, 'distance')
                    if isfield(filter_configs.distance, 'enable')
                        self.enable_distance_filter = filter_configs.distance.enable;
                    end
                    if isfield(filter_configs.distance, 'type')
                        self.distance_filter_type = filter_configs.distance.type;
                    end
                end
                
                % Configure acceleration filter
                if isfield(filter_configs, 'acceleration')
                    if isfield(filter_configs.acceleration, 'enable')
                        self.enable_acceleration_filter = filter_configs.acceleration.enable;
                    end
                    if isfield(filter_configs.acceleration, 'type')
                        self.acceleration_filter_type = filter_configs.acceleration.type;
                    end
                end
                
                % Configure beacon filter
                if isfield(filter_configs, 'beacon')
                    if isfield(filter_configs.beacon, 'enable')
                        self.enable_beacon_filter = filter_configs.beacon.enable;
                    end
                    if isfield(filter_configs.beacon, 'type')
                        self.beacon_filter_type = filter_configs.beacon.type;
                    end
                end
                
                % Configure heading filter
                if isfield(filter_configs, 'heading')
                    if isfield(filter_configs.heading, 'enable')
                        self.enable_heading_filter = filter_configs.heading.enable;
                    end
                    if isfield(filter_configs.heading, 'type')
                        self.heading_filter_type = filter_configs.heading.type;
                    end
                end
            end
        end
        
        function reset_filters(self)
            % reset_filters - Reset all filter buffers and previous values
            self.velocity_score_buffer = [];
            self.distance_score_buffer = [];
            self.acceleration_score_buffer = [];
            self.beacon_score_buffer = [];
            self.heading_score_buffer = [];
            
            self.prev_filtered_v_score = 1.0;
            self.prev_filtered_d_score = 1.0;
            self.prev_filtered_a_score = 1.0;
            self.prev_filtered_beacon_score = 1.0;
            self.prev_filtered_h_score = 1.0;
            
            % Clear filtered score logs
            self.filtered_v_score_log = [];
            self.filtered_d_score_log = [];
            self.filtered_a_score_log = [];
            self.filtered_beacon_score_log = [];
            self.filtered_h_score_log = [];
        end

        %%% Trust sample calculation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   %%%%%%%%%%%%%%%%%%%
        function trust_sample = calculate_trust_sample_wo_Acc(self, v_score, d_score, a_score, beacon_score,h_score,is_nearby)
            trust_sample = beacon_score * (v_score) * (d_score) ;
        end

        function trust_sample = calculate_trust_sample_normal(self, v_score, d_score, a_score, beacon_score,h_score,is_nearby)
            if (is_nearby)
                trust_sample = beacon_score * (v_score^self.wv_nearby) * (d_score^self.wd_nearby) * h_score^self.wh_nearby;
            else
                trust_sample = beacon_score * (v_score^self.wv)* (d_score^self.wd) ;
            end
        end

        function trust_sample = calculate_trust_sample_python(self, v_score, d_score, a_score, beacon_score, h_score, quality_factor)
            if nargin < 7
                quality_factor = beacon_score;
            end

            scores = [v_score, d_score, a_score, h_score, beacon_score, quality_factor];
            clamped_scores = zeros(size(scores));
            for idx = 1:length(scores)
                clamped_scores(idx) = self.clamp_unit(scores(idx));
            end

            fusion_mode = char(lower(strtrim(string(self.local_trust_fusion_mode))));
            switch fusion_mode
                case {'product', 'direct_product', 'multiply'}
                    trust_sample = prod(clamped_scores);

                case {'equal', 'equal_geometric', 'equal_geomean'}
                    safe_scores = max(clamped_scores, 0.01);
                    trust_sample = prod(safe_scores) ^ (1.0 / length(safe_scores));

                otherwise
                    weights = [ ...
                        self.local_weight_velocity, ...
                        self.local_weight_distance, ...
                        self.local_weight_acceleration, ...
                        self.local_weight_heading, ...
                        self.local_weight_beacon, ...
                        self.local_weight_quality];

                    weighted_product = 1.0;
                    total_weight = 0.0;
                    for idx = 1:length(clamped_scores)
                        safe_score = max(clamped_scores(idx), 0.01);
                        weighted_product = weighted_product * (safe_score ^ weights(idx));
                        total_weight = total_weight + weights(idx);
                    end

                    if total_weight > 0
                        trust_sample = weighted_product ^ (1.0 / total_weight);
                    else
                        trust_sample = 0.5;
                    end
            end
            trust_sample = self.clamp_unit(trust_sample);
        end

        function trust_sample = calculate_trust_sample(self, v_score, d_score, a_score, beacon_score,h_score,is_nearby)
            % Local beacon quality has no separate MATLAB channel, so it is
            % reused for both beacon and quality terms.
            trust_sample = self.calculate_trust_sample_python(v_score, d_score, a_score, beacon_score, h_score, beacon_score);
        end
        function trust_sample = calculate_trust_sample_weighted_based(self, v_score, d_score, a_score, beacon_score,h_score,is_nearby)
            % Kept for configuration compatibility; the live local trust formula
            % is selected by local_trust_fusion_mode.
            trust_sample = self.calculate_trust_sample_python(v_score, d_score, a_score, beacon_score, h_score, beacon_score);
        end
        
        function trust_sample = calculate_trust_sample_with_filtering(self, v_score, d_score, a_score, beacon_score, h_score, is_nearby)
            % calculate_trust_sample_with_filtering - Calculate trust sample with optional filtering
            % This method applies filtering if enabled, then calculates the trust sample
            %
            % Inputs:
            %   v_score: Velocity trust score
            %   d_score: Distance trust score  
            %   a_score: Acceleration trust score
            %   beacon_score: Beacon reception score
            %   h_score: Heading trust score
            %   is_nearby: Boolean indicating if target is nearby
            %
            % Output:
            %   trust_sample: Final trust sample value
            
            % Apply filtering to all scores
            [filtered_v_score, filtered_d_score, filtered_a_score, filtered_beacon_score, filtered_h_score] = ...
                self.filter_all_scores(v_score, d_score, a_score, beacon_score, h_score);
            
            trust_sample = self.calculate_trust_sample_python( ...
                filtered_v_score, filtered_d_score, filtered_a_score, ...
                filtered_beacon_score, filtered_h_score, filtered_beacon_score);
        end
        
        function trust_sample = calculate_trust_sample_filtered_weighted(self, v_score, d_score, a_score, beacon_score, h_score, ~)
            % calculate_trust_sample_filtered_weighted - Weighted-based trust sample with filtering
            % Uses linear combination of filtered scores
            %
            % Inputs:
            %   v_score: Velocity trust score
            %   d_score: Distance trust score  
            %   a_score: Acceleration trust score
            %   beacon_score: Beacon reception score (multiplier)
            %   h_score: Heading trust score
            %   is_nearby: Not used in weighted calculation
            %
            % Output:
            %   trust_sample: Final trust sample value
            
            % Apply filtering to all scores
            [filtered_v_score, filtered_d_score, filtered_a_score, filtered_beacon_score, filtered_h_score] = ...
                self.filter_all_scores(v_score, d_score, a_score, beacon_score, h_score);
            
            trust_sample = self.calculate_trust_sample_python( ...
                filtered_v_score, filtered_d_score, filtered_a_score, ...
                filtered_beacon_score, filtered_h_score, filtered_beacon_score);
        end
        
        %%% Trust decay implementation (Separate Local and Global) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   %%%%%%%%%%%%%%%%%%%
        function trust_score = apply_trust_decay(self, target_id, current_trust, beacon_received, trust_type)
            % Apply trust decay separately for local and global trust
            % LT_local(t) = (1 - λ_h) * LT_local(t-1) when local beacon not received
            % LT_global(t) = (1 - λ_h) * LT_global(t-1) when global beacon not received
            % 
            % Inputs:
            %   target_id: ID of target vehicle
            %   current_trust: Current trust score
            %   beacon_received: Boolean indicating if beacon was received
            %   trust_type: 'local' or 'global' (required)
            % Output:
            %   trust_score: Decayed trust score
            
            if nargin < 5
                error('trust_type parameter is required: "local" or "global"');
            end
            
            % Validate trust_type
            if ~ismember(trust_type, {'local', 'global'})
                error('trust_type must be "local" or "global"');
            end
            
            if beacon_received
                % Beacon received, use current trust score directly
                trust_score = self.clamp_unit(current_trust);
            else
                % No beacon received, apply decay to previous trust score
                if trust_type == "local"
                    previous_scores = self.previous_trust_scores.local;
                else % global
                    previous_scores = self.previous_trust_scores.global;
                end
                
                if target_id <= length(previous_scores) && isfinite(previous_scores(target_id))
                    previous_trust = self.clamp_unit(previous_scores(target_id));
                else
                    previous_trust = 1.0; % Default high trust for new vehicles
                end
                
                % Apply decay formula: trust_new = (1 - λ_h) * trust_old
                trust_score = (1 - self.lambda_h) * previous_trust;
                trust_floor = max(self.clamp_unit(self.distributed_trust_fallback) * 0.1, 0.01);
                trust_score = max(trust_score, trust_floor);
            end
            
            % Store the trust score in the appropriate array
            if trust_type == "local"
                self.previous_trust_scores.local(target_id) = trust_score;
            else % global
                self.previous_trust_scores.global(target_id) = trust_score;
            end
        end
        
        %%% Physical constraints validation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   %%%%%%%%%%%%%%%%%%%
        function is_valid = check_physical_constraints(self, reported_state, previous_state, dt)
            % Check if reported state violates physical constraints
            % Inputs:
            %   reported_state: [pos_x, pos_y, heading, velocity, acceleration]
            %   previous_state: previous state vector
            %   dt: time step
            % Output:
            %   is_valid: boolean indicating if constraints are satisfied
            
            is_valid = true;
            
            % Extract current values
            current_velocity = reported_state(4);
            current_acceleration = reported_state(5);
            
            % Check velocity limits
            if abs(current_velocity) > self.MAX_VELOCITY
                is_valid = false;
                return;
            end
            
            % Check acceleration limits
            if current_acceleration > self.MAX_ACCEL || current_acceleration < self.MAX_DECEL
                is_valid = false;
                return;
            end
            
            % Check for reasonable acceleration change (jerk limits)
            if ~isempty(previous_state) && length(previous_state) >= 5
                previous_acceleration = previous_state(5);
                jerk = abs(current_acceleration - previous_acceleration) / dt;
                if jerk > self.MAX_JERK
                    is_valid = false;
                    return;
                end
            end
        end
        
        %%% Temporal consistency evaluation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   %%%%%%%%%%%%%%%%%%%
        function temporal_score = evaluate_temporal_consistency(self, target_id, current_state, dt, tolerance_scale)
            % Evaluate temporal consistency of reported state with previous state
            % Inputs:
            %   target_id: ID of target vehicle
            %   current_state: [pos_x, pos_y, heading, velocity, acceleration]
            %   dt: time step
            %   tolerance_scale: warm-up multiplier for transient tolerance
            % Output:
            %   temporal_score: consistency score (0 to 1)
            if nargin < 5
                tolerance_scale = 1.0;
            end
            tolerance_scale = max(1.0, tolerance_scale);
            
            % Check if we have previous state for this vehicle
            if ~isKey(self.previous_states_map, target_id)
                % First time seeing this vehicle, store state and return max score
                self.previous_states_map(target_id) = current_state;
                temporal_score = 1.0;
                return;
            end
            
            % Get previous state
            prev_state = self.previous_states_map(target_id);
            
            % Extract values
            current_pos_x = current_state(1);
            current_pos_y = current_state(2);
            current_velocity = current_state(4);
            
            previous_pos_x = prev_state(1);
            previous_pos_y = prev_state(2);
            previous_velocity = prev_state(4);
            previous_acceleration = prev_state(5);
            
            % Check position-velocity consistency
            expected_pos_x = previous_pos_x + previous_velocity * cos(prev_state(3)) * dt;
            expected_pos_y = previous_pos_y + previous_velocity * sin(prev_state(3)) * dt;
            pos_error = sqrt((current_pos_x - expected_pos_x)^2 + (current_pos_y - expected_pos_y)^2);
            
            % Check velocity-acceleration consistency
            expected_velocity = previous_velocity + previous_acceleration * dt;
            vel_error = abs(current_velocity - expected_velocity);
            
            % Normalize errors and compute score
            pos_tolerance = max(self.temporal_pos_tolerance_m, 1e-3) * tolerance_scale;
            vel_tolerance = max(self.temporal_vel_tolerance, 1e-3) * tolerance_scale;
            
            pos_score = max(1 - pos_error / pos_tolerance, 0);
            vel_score = max(1 - vel_error / vel_tolerance, 0);
            
            % Combine scores (weighted average)
            temporal_score = 0.6 * pos_score + 0.4 * vel_score;
            
            % Store current state for next iteration
            self.previous_states_map(target_id) = current_state;
        end
        
        %%% Trust rating vector update %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   %%%%%%%%%%%%%%%%%%%

        function update_rating_vector(self, trust_sample , type)
            %  Map trust sample to a specific trust level in the rating vector

            zero_based_level = self.python_round_nonnegative( ...
                self.clamp_unit(trust_sample) * (self.k - 1));
            trust_level = min(zero_based_level + 1, self.k);
            trust_vector = zeros(1, self.k); % trust_vector = r_y^x
            trust_vector(trust_level) = 1;

            % Compute current trust score (sigma_y) to use in lambda_y calculation

            if (type == "local")
                current_trust_score = self.calculate_trust_score( self.rating_vector);
                lambda_y = current_trust_score * self.wt;
            else
                current_trust_score = self.calculate_trust_score(self.rating_vector_global);
                lambda_y = current_trust_score * self.wt_global;
            end

            % Define lambda_y as per equation (9): lambda_y = sigma_y * w_t

            % Update the rating vector (R_y) using the aging factor lambda_y
            if type == "local"
                self.rating_vector = (1 - lambda_y) * self.rating_vector + trust_vector;
            else
                self.rating_vector_global = (1 - lambda_y) * self.rating_vector_global + trust_vector;
            end
            % self.rating_vector = (1 - lambda_y) * self.rating_vector + trust_vector;
            % self.rating_vector
            % lambda_y
        end


        function trust_score = calculate_trust_score(self,rating_vector)
            %Explain : https://discord.com/channels/1123389035713400902/1327310432381435914/1327403771663224873

            % Normalize the rating vector to ensure it represents probabilities
            % a = (1/self.k) = 1/5 = 0.2 in  self.C / self.k
            S_y = (rating_vector + self.C / self.k) / (self.C + sum(rating_vector));

            % Define weights with a small epsilon to avoid zero weight for the lowest level
            epsilon = 0.01; % Small positive constant
            weights = ((0:(self.k-1)) + epsilon) / (self.k-1 + epsilon);

            % Calculate the trust score as a weighted average
            trust_score = sum(weights .* S_y);
        end

        function value = python_round_nonnegative(~, value)
            % Python round() uses ties-to-even; MATLAB round() uses ties
            % away from zero. Trust bin boundaries must follow Python.
            lower_value = floor(value);
            fraction = value - lower_value;
            if abs(fraction - 0.5) <= 16 * eps(max(abs(value), 1))
                if mod(lower_value, 2) == 0
                    value = lower_value;
                else
                    value = lower_value + 1;
                end
            else
                value = floor(value + 0.5);
            end
        end



        %%% Main %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   %%%%%%%%%%%%%%%%%%%
        function [final_score,local_trust_sample,gamma_cross, v_score ,d_score,a_score,beacon_score_local, beacon_score_global] = calculateTrust(self , host_vehicle, target_vehicle, leader_vehicle, neighbors, is_nearby, instant_idx)
            % calculateTrust - Computes trust scores for a specific vehicle
            % Updated to include gamma_local_our_self for self-consistency evaluation
            %
            % Inputs:
            %   host_id       - ID of the current vehicle
            %   x_local          - Local state of the current vehicle (3x1 vector)
            %   neighbors_states - A matrix where each column is the state of a neighbor vehicle (3xN matrix)
            %   neighbors_ids    - IDs of the neighbor vehicles (1xN vector)
            %   instant_idx        - Current simulation time step (scalar)
            %
            % Outputs:
            %   trust_scores     - Trust scores for each neighbor (1xN vector)

            host_id = host_vehicle.vehicle_number;
            target_id = target_vehicle.vehicle_number;
            self.apply_scenario_config(host_vehicle.scenarios_config);
            self.latest_target_turn_context = 0.0;
            trust_warmup_time = max(0, self.get_config_numeric( ...
                host_vehicle.scenarios_config, 'trust_warmup_time', 0));
            trust_warmup_tolerance_scale = max(1.0, self.get_config_numeric( ...
                host_vehicle.scenarios_config, 'trust_warmup_tolerance_scale', 1.0));
            local_flag_threshold = self.clamp_unit(self.get_config_numeric( ...
                host_vehicle.scenarios_config, 'local_trust_flag_threshold', 0.5));
            local_flag_required_samples = max(1, round(self.get_config_numeric( ...
                host_vehicle.scenarios_config, 'local_trust_flag_required_samples', 1)));
            is_trust_warmup = instant_idx * host_vehicle.dt < trust_warmup_time;
            if is_trust_warmup
                self.active_trust_tolerance_scale = trust_warmup_tolerance_scale;
            else
                self.active_trust_tolerance_scale = 1.0;
            end
            if isprop(host_vehicle.scenarios_config, 'local_trust_fusion_mode')
                self.local_trust_fusion_mode = string(host_vehicle.scenarios_config.local_trust_fusion_mode);
            end
             

             
            % Reported data for trust evaluation

            % half_lenght_vehicle = target_vehicle.param.l_r; % distance between vehicle's c.g. and rear axle
            vehicle_length = target_vehicle.param.l_r + target_vehicle.param.l_f ; % total length of a vehicle

            leader_state = host_vehicle.center_communication.get_local_state(leader_vehicle.vehicle_number , host_id);
            if any(isnan(leader_state(:)))
                % Use the lastest state of leader vehicle
                leader_state = self.lead_state_lastest ;
                leader_beacon_interval = (instant_idx - self.lead_state_lastest_timestamp)*host_vehicle.dt;
            else
                % save the lastest state of leader vehicle
                self.lead_state_lastest = leader_state;
                self.lead_state_lastest_timestamp = instant_idx;
                leader_beacon_interval = 0;
            end
            leader_velocity = leader_state(4);
            leader_acceleration = leader_state(5);

            % Measurement part host (radar, lidar  )
            host_pos_X = host_vehicle.observer.est_local_state_current(1);
            host_pos_Y = host_vehicle.observer.est_local_state_current(2);
            host_velocity = host_vehicle.observer.est_local_state_current(4);
            host_acceleration = host_vehicle.observer.est_local_state_current(5);
            host_state_for_local_trust = host_vehicle.observer.est_local_state_current;

            %%

            % if (host_id - neighbors_ids) < 0  =>  host is Front , else = Behind

            % host_distance_measurement = (host_id - target_id)*(target_vehicle.observer.est_local_state_current(1) - host_vehicle.observer.est_local_state_current(1)) - half_lenght_vehicle;

            % Why target_vehicle.state(1) : because is the host_distance_measurement , its in the point view of Host
            % So need to be acurate , not disturb by attack like "target_pos_X" (below )

            if is_nearby
                delta_X = target_vehicle.state(1) - host_pos_X;  % center-to-center distance

                if delta_X >= 0
                    % Host is behind target → rear-to-bumper (e.g., vehicle 1 to 4)
                    host_distance_measurement = delta_X + vehicle_length;
                    % host_distance_measurement = abs(delta_X);
                else
                    % Host is in front of target → bumper-to-rear (e.g., vehicle 4 to 1)
                    host_distance_measurement = abs(delta_X) - vehicle_length;
                    % host_distance_measurement = abs(delta_X);
                end
                % host_distance_measurement = (host_id - target_id)*(target_vehicle.state(1) - host_pos_X) - vehicle_length;
            else
                %% Oracle mode for test-only studies where non-nearby distance is assumed known.
                if host_vehicle.scenarios_config.is_know_data_not_nearby == true
                    delta_X = target_vehicle.state(1) - host_pos_X;  % center-to-center distance

                    if delta_X >= 0
                        % Host is behind target → rear-to-bumper (e.g., vehicle 1 to 4)
                        host_distance_measurement = delta_X + vehicle_length;
                        % host_distance_measurement = abs(delta_X);
                    else
                        % Host is in front of target → bumper-to-rear (e.g., vehicle 4 to 1)
                        host_distance_measurement = abs(delta_X) - vehicle_length;
                        % host_distance_measurement = abs(delta_X);
                    end
                else
                    %% TODO : If we don't know the distance between host and target (dont have the real measurement)
                    % Use the estimated distance based on the host's position and target's position

                    nb_space = abs(host_id - target_id); % sign of the host and target id
                    T = host_vehicle.Param_opt.hi ; %% Time gap , time headway (s)
                    s0 = host_vehicle.Param_opt.ri; % Minimum gap distance
                    s_acc_expected = nb_space*(s0 + T*host_velocity); % h_base = T

                    host_distance_measurement = s_acc_expected - vehicle_length;
                end
            end

            if is_nearby || host_vehicle.scenarios_config.is_know_data_not_nearby == true
                local_measured_distance = hypot(target_vehicle.state(1) - host_pos_X, target_vehicle.state(2) - host_pos_Y);
            else
                local_measured_distance = NaN;
            end






            %% TODO : need to get real distance between host and target

            target_state = host_vehicle.center_communication.get_local_state(target_id,host_id);
            if any(isnan(target_state(:)))
                beacon_score_local = 0;  % Local channel beacon not received
                v_score = 0;
                d_score = 0;
                a_score = 0;
                h_score = 0;
                quality_factor = 0;
                local_trust_sample = 0;
            else
                beacon_score_local = 1;  % Local channel beacon received
                quality_factor = beacon_score_local; % Match Python q_factor with local beacon quality in MATLAB
                % target_input = host_vehicle.center_communication.get_input(target_id);

                target_pos_X = target_state(1);
                target_pos_Y = target_state(2);

                %% distance reporting
                delta_X = target_pos_X - host_pos_X;  % center-to-center distance

                if delta_X >= 0
                    % Host is behind target → rear-to-bumper (e.g., vehicle 1 to 4)
                    target_reported_distance = delta_X + vehicle_length;
                    % target_reported_distance = abs(delta_X) ;
                else
                    % Host is in front of target → bumper-to-rear (e.g., vehicle 4 to 1)
                    target_reported_distance = abs(delta_X) - vehicle_length;
                    % target_reported_distance = abs(delta_X);
                end

                target_reported_velocity = target_state(4);
                target_reported_acceleration = target_state(5);



                % Python-style local component trust evaluation
                [v_score, d_score, a_score, h_score, severe_local_pose_mismatch] = ...
                    self.evaluate_python_local_scores( ...
                    host_state_for_local_trust, target_state, leader_state, ...
                    target_id, local_measured_distance, instant_idx, host_vehicle.dt);

                % if instant_idx - self.last_time_d == self.Period_a_score_distane
                %     self.last_time_d = instant_idx;
                % end
                self.last_d = host_distance_measurement;



                %% New validation checks (Option - Controlled by Scenarios_config)
                if host_vehicle.scenarios_config.Use_physical_constraints_check
                    % Physical constraints validation
                    physical_valid = self.check_physical_constraints(target_state, [], host_vehicle.dt);
                    self.physical_valid_log = [self.physical_valid_log, physical_valid];
                    
                    % Apply physical validation penalty to trust scores
                    if ~physical_valid
                        % Severely penalize physically impossible states
                        v_score = v_score * 0.1;
                        d_score = d_score * 0.1;
                        a_score = a_score * 0.1;
                    end
                else
                    % Log default valid state when check is disabled
                    self.physical_valid_log = [self.physical_valid_log, true];
                end
                
                if host_vehicle.scenarios_config.Use_temporal_consistency_check
                    % Temporal consistency evaluation
                    temporal_score = self.evaluate_temporal_consistency( ...
                        target_id, target_state, host_vehicle.dt, ...
                        self.active_trust_tolerance_scale);
                    self.temporal_score_log = [self.temporal_score_log, temporal_score];
                    
                    % Apply temporal consistency penalty
                    temporal_penalty = temporal_score; % temporal_score is already 0-1
                    v_score = v_score * temporal_penalty;
                    d_score = d_score * temporal_penalty;
                    a_score = a_score * temporal_penalty;
                else
                    % Log default perfect score when check is disabled
                    self.temporal_score_log = [self.temporal_score_log, 1.0];
                end

                % Compute Python-style local trust sample using local beacon as quality.
                local_trust_sample = self.calculate_trust_sample_python( ...
                    v_score, d_score, a_score, beacon_score_local, h_score, quality_factor);
                if severe_local_pose_mismatch
                    local_trust_sample = min(local_trust_sample, d_score);
                end
               
            end
            local_trust_sample_raw = local_trust_sample;

            % Evaluate self-consistency of distributed estimation first - how trustworthy is our own global state?
            gamma_local_our_self = self.compute_self_consistency_factor(host_vehicle, neighbors);
            
            % Check if observer is in prediction-only mode for this target (using convenient method)
            is_in_prediction_mode = false;
            if host_vehicle.scenarios_config.Use_predict_observer
                is_in_prediction_mode = host_vehicle.observer.is_vehicle_in_prediction_mode(target_id);
            end


            %% Flag Check with self-consistency consideration
            self.flag_taget_attk = false;
            self.flag_glob_est_check = false;
            self.flag_local_est_check = false;


            %% ---------- Global channel evaluation
            D_pos = NaN;
            D_vel = NaN;
            D_acc = NaN;
            D_theta = NaN;
            D_total = NaN;

            target_global_state = target_vehicle.center_communication.get_global_state(target_id,host_id);
            if any(isnan(target_global_state(:)))
                % Global channel beacon not received
                beacon_score_global = 0;
                gamma_cross = 0;
                gamma_local = 0;
                global_trust_sample = 0;
            else
                % Global channel beacon received
                beacon_score_global = 1;
                if self.should_use_python_global_trust(host_vehicle)
                    [global_trust_sample, gamma_cross, gamma_local, gamma_local_our_self, ...
                        D_pos, D_vel, D_acc, D_theta, D_total] = ...
                        self.compute_global_trust_sample_python( ...
                        host_vehicle, target_vehicle, target_state, ...
                        host_state_for_local_trust, local_measured_distance, ...
                        target_global_state);
                else
                % Compute trust factors using the legacy MATLAB calculation
                [gamma_cross, D_pos, D_vel, D_acc, D_theta, D_total] = self.compute_cross_host_target_factor(host_id,host_vehicle, target_id,target_vehicle);
                gamma_local = self.compute_local_consistency_factor(host_vehicle, target_vehicle, neighbors);
                
                if is_in_prediction_mode
                    % SELF-AWARE PREDICTION-ONLY MODE ADJUSTMENT
                    % Use gamma_local_our_self to determine if WE are the problem or OTHERS are the problem
                    
                    
                    if gamma_local_our_self >= self.self_trust_threshold
                        self.flag_glob_est_check = true;
                        % HIGH SELF-CONSISTENCY: Our distributed estimation is still good
                        % → The problem is likely with external data (attack/corruption)
                        % → Justifiably reduce trust in external estimates
                        gamma_cross = gamma_cross * 0.4; % Penalize cross-validation (others might be bad)
                        global_trust_sample = gamma_cross * gamma_local * 0.8; % Mild penalty
                        
                    else
                        self.flag_taget_attk = true;
                        % LOW SELF-CONSISTENCY: Our distributed estimation has degraded in prediction-only mode
                        % → The problem might be US, not others
                        % → Don't heavily penalize external data, but acknowledge our uncertainty
                        gamma_cross = gamma_cross * 0.8; % Light penalty on cross-validation
                        gamma_local = gamma_local * gamma_local_our_self; % Scale local consistency by self-trust
                        global_trust_sample = gamma_cross * gamma_local * 0.9; % Acknowledge our degraded state
                    end
                else
                    % Normal mode: Full trust evaluation
                    self.flag_glob_est_check = true;
                    global_trust_sample = gamma_cross * gamma_local;
                    if gamma_local_our_self < self.self_trust_threshold
                        % If we don't trust our own global state, reduce reliance on global estimates
                        global_trust_sample = global_trust_sample * gamma_local_our_self;
                        % Also reduce cross-validation trust as it depends on our global estimate
                        gamma_cross = gamma_cross * gamma_local_our_self;
                    end
                end
                end
            end


            if (host_vehicle.scenarios_config.Monitor_sudden_change == true)
                beta = self.monitor_sudden(gamma_cross , D_pos,D_vel,D_acc);
            else
                beta = 1; % Default value
                self.D_pos_log = [self.D_pos_log, D_pos];
                self.D_vel_log = [self.D_vel_log, D_vel];
                self.D_acc_log = [self.D_acc_log, D_acc];
            end
            self.D_theta_log = [self.D_theta_log, D_theta];
            self.D_total_log = [self.D_total_log, D_total];

            % %% Apply self-consistency factor before trust calculation
            % % If our own global state is not trustworthy, reduce confidence in global estimates
            % self_trust_threshold = 0.7; % Threshold for considering our own state trustworthy
            
            % if gamma_local_our_self < self_trust_threshold
            %     % If we don't trust our own global state, reduce reliance on global estimates
            %     global_trust_sample = global_trust_sample * gamma_local_our_self;
            %     % Also reduce cross-validation trust as it depends on our global estimate
            %     gamma_cross = gamma_cross * gamma_local_our_self;
            % end

            %% Apply trust decay to local and global trust separately
            % Implement equation: LT_{i,l}(t) = (1 - λ_h) * LT_{i,l}(t-1) when δ_{i,l}(t) = 0
            beacon_received_local = (beacon_score_local == 1);
            beacon_received_global = (beacon_score_global == 1);
            
            %% Separate trust sample for local and global estimates
            if (host_vehicle.scenarios_config.Dichiret_type == "Single")
                % Apply trust decay to the combined sample before rating vector update
                local_trust_sample_decayed = self.apply_trust_decay(target_id, local_trust_sample, beacon_received_local, 'local');
                global_trust_sample_decayed = self.apply_trust_decay(target_id, global_trust_sample, beacon_received_global, 'global');
                
                % Log decayed trust values
                self.local_trust_decayed_log = [self.local_trust_decayed_log, local_trust_sample_decayed];
                self.global_trust_decayed_log = [self.global_trust_decayed_log, global_trust_sample_decayed];
                
                trust_sample_ext = local_trust_sample_decayed * global_trust_sample_decayed;
                self.update_rating_vector(trust_sample_ext , "local");
                final_score = self.calculate_trust_score(self.rating_vector);
            else % "Dual"
                % Apply trust decay to local trust
                local_trust_sample_decayed = self.apply_trust_decay(target_id, local_trust_sample, beacon_received_local, 'local');
                
                % Apply trust decay to global trust
                global_trust_sample_decayed = self.apply_trust_decay(target_id, global_trust_sample, beacon_received_global, 'global');

                % Log decayed trust values
                self.local_trust_decayed_log = [self.local_trust_decayed_log, local_trust_sample_decayed];
                self.global_trust_decayed_log = [self.global_trust_decayed_log, global_trust_sample_decayed];

                self.update_rating_vector(local_trust_sample_decayed , "local");
                local_trust_sample = self.calculate_trust_score(self.rating_vector);

                self.update_rating_vector(global_trust_sample_decayed , "global");
                global_trust_sample = self.calculate_trust_score(self.rating_vector_global);

                final_score = local_trust_sample * global_trust_sample;
            end
            final_score = final_score * beta;
            % Python applies a final-score EMA after the Dirichlet update.
            if isfinite(self.previous_final_score)
                alpha_final = min(1.0, max(0.0, self.ema_alpha));
                final_score = alpha_final * final_score + ...
                    (1.0 - alpha_final) * self.previous_final_score;
            end
            final_score = self.clamp_unit(final_score);


            
            % Only trust attack detection if our own state is trustworthy
            if ~is_trust_warmup
                if (gamma_local_our_self > 0.6)
                    if (gamma_local > 0.5 && gamma_cross < 0.5)
                        self.flag_taget_attk = true;
                    end
                    if (gamma_local < 0.5 && gamma_cross > 0.5)
                        self.flag_local_est_check = true;
                    end
                else
                    % If our own global state is not trustworthy, flag it
                    self.flag_glob_est_check = true; % Our global estimate needs checking
                end

                %% importance
                current_log_idx = length(self.local_trust_decayed_log);
                first_flag_idx = max(1, floor(trust_warmup_time / host_vehicle.dt) + 1);
                recent_start_idx = max([1, current_log_idx - local_flag_required_samples + 1, first_flag_idx]);
                recent_local_trust = self.local_trust_decayed_log(recent_start_idx:current_log_idx);
                if numel(recent_local_trust) >= local_flag_required_samples && ...
                        all(recent_local_trust < local_flag_threshold)
                    self.flag_local_est_check = true;
                end
            end

            if self.should_use_python_global_trust(host_vehicle)
                % Exact TrustScore flag semantics from trust_model.py.  Use
                % the decayed local/global samples, not the Dirichlet final
                % score, so each flag identifies the failing channel.
                flag_threshold = self.clamp_unit(self.get_config_numeric( ...
                    host_vehicle.scenarios_config, 'trust_threshold', 0.5));
                self.set_python_attack_flags( ...
                    local_trust_sample_decayed, global_trust_sample_decayed, flag_threshold);
            end

            self.previous_final_score = final_score;


            % final_score = trust_sample_ext ;

            % Log data for analysis
            self.trust_sample_log = [self.trust_sample_log, local_trust_sample_raw];
            self.gamma_cross_log = [self.gamma_cross_log, gamma_cross];
            self.gamma_local_log = [self.gamma_local_log, gamma_local];
            self.gamma_local_our_self_log = [self.gamma_local_our_self_log, gamma_local_our_self];
            % self.gamma_expected_log = [self.gamma_expected_log, gamma_expected];

            self.v_score_log = [self.v_score_log, v_score];
            self.d_score_log = [self.d_score_log, d_score];
            self.a_score_log = [self.a_score_log, a_score];
            self.h_score_log = [self.h_score_log, h_score];

            % Log separate beacon scores
            self.beacon_score_local_log = [self.beacon_score_local_log, beacon_score_local];
            self.beacon_score_global_log = [self.beacon_score_global_log, beacon_score_global];
            
            % Maintain compatibility - log combined beacon score (for existing code)
            beacon_score_combined = beacon_score_local * beacon_score_global; % Both channels must work
            self.beacon_score_log = [self.beacon_score_log, beacon_score_combined];

            self.final_score_log = [self.final_score_log, final_score];

            self.flag_taget_attk_log = [self.flag_taget_attk_log, self.flag_taget_attk];
            self.flag_glob_est_check_log = [self.flag_glob_est_check_log, self.flag_glob_est_check];
            self.flag_local_est_check_log = [self.flag_local_est_check_log, self.flag_local_est_check];

        end

        function set_python_attack_flags(self, local_trust, global_trust, threshold)
            threshold = self.clamp_unit(threshold);
            local_bad = self.clamp_unit(local_trust) < threshold;
            global_bad = self.clamp_unit(global_trust) < threshold;
            self.flag_taget_attk = local_bad && global_bad;
            self.flag_glob_est_check = ~local_bad && global_bad;
            self.flag_local_est_check = local_bad;
        end

        function beta = monitor_sudden(self,gamma_cross,D_pos,D_vel,D_acc)
            % --- New Code: Anomaly Detection and Trust Adjustment ---

            %% TODO in the case DOS ,so need to forget about the anomaly pos and velocity
            % Anomaly detection for gamma_cross
            anomaly_gamma = 0;  % Default value
            if length(self.gamma_cross_log) >= self.w
                window = self.gamma_cross_log(end-self.w+1:end);
                mu_gamma = mean(window);
                sigma_gamma = std(window);
                if sigma_gamma > 0 && abs(gamma_cross - mu_gamma) > 2 * sigma_gamma
                    anomaly_gamma = 1;
                end
            end
            self.anomaly_gamma_log = [self.anomaly_gamma_log, anomaly_gamma];

            % Log discrepancies
            self.D_pos_log = [self.D_pos_log, D_pos];
            self.D_vel_log = [self.D_vel_log, D_vel];
            self.D_acc_log = [self.D_acc_log, D_acc];

            % Anomaly detection for position
            anomaly_pos = 0;
            if length(self.D_pos_log) >= self.w
                window = self.D_pos_log(end-self.w+1:end);
                mu_pos = mean(window);
                sigma_pos = std(window);
                if sigma_pos > 0 && abs(D_pos - mu_pos) > 2 * sigma_pos
                    anomaly_pos = 1;
                end
            end
            self.anomaly_pos_log = [self.anomaly_pos_log, anomaly_pos];

            % Anomaly detection for velocity
            anomaly_vel = 0;
            if length(self.D_vel_log) >= self.w
                window = self.D_vel_log(end-self.w+1:end);
                mu_vel = mean(window);
                sigma_vel = std(window);
                if sigma_vel > 0 && abs(D_vel - mu_vel) > 2 * sigma_vel
                    anomaly_vel = 1;
                end
            end
            % Log the anomaly velocity

            self.anomaly_vel_log = [self.anomaly_vel_log, anomaly_vel];


            % Anomaly detection for acceleration
            anomaly_acc = 0;
            if length(self.D_acc_log) >= self.w
                window = self.D_acc_log(end-self.w+1:end);
                mu_acc = mean(window);
                sigma_acc = std(window);
                if sigma_acc > 0 && abs(D_acc - mu_acc) > 2 * sigma_acc
                    anomaly_acc = 1;
                end
            end
            % Log the anomaly acceleration
            self.anomaly_acc_log = [self.anomaly_acc_log, anomaly_acc];



            beta = 1; % Default value
            % Cumulative check for trust adjustment
            if length(self.anomaly_gamma_log) >= self.w
                % self.w - 1 because length(self.beacon_score_log) only have w-1 elements
                anomaly_drop_packet = (self.w - 1) - sum(self.beacon_score_log(end-self.w+2:end));
                anomaly_count_gamma = sum(self.anomaly_gamma_log(end-self.w+1:end));
                anomaly_count_pos = sum(self.anomaly_pos_log(end-self.w+1:end));
                anomaly_count_vel = sum(self.anomaly_vel_log(end-self.w+1:end));
                anomaly_count_acc = sum(self.anomaly_acc_log(end-self.w+1:end));
                min_anomali = min([anomaly_count_gamma, anomaly_count_pos, anomaly_count_vel, anomaly_count_acc, anomaly_drop_packet]);
                if min_anomali > self.Threshold_anomalie
                    beta = (1-self.reduce_factor*min_anomali / self.w);
                end
            end
        end



        % Only compare with the directed neighbor (not all neighbors) , or more specific is the target vehicle
        function [gamma_cross, D_pos, D_vel, D_acc, D_theta, D_total] = compute_cross_host_target_factor(self, host_id,host_vehicle, target_id,target_vehicle)
            % Inputs:
            %   host_vehicle: The vehicle evaluating trust
            %   target_vehicle: The neighbor whose global estimate is being evaluated
            %   neighbors: Array of neighbor vehicle objects
            %
            % Output:
            %   gamma_cross: Trust factor based on cross-validation (0 to 1)

            host_global_estimate = host_vehicle.observer.est_global_state_current;

            % Get target vehicle's global estimate
            target_global_estimate = host_vehicle.center_communication.get_global_state(target_id,host_id);


            % Compute total discrepancy D_i,l(k)
            D = 0;
            D_pos = 0;
            D_vel = 0;
            D_acc = 0;
            D_theta = 0;
            num_vehicles = size(target_global_estimate,2);
            for j = 1:num_vehicles

                % Position difference
                pos_diff = target_global_estimate(1:2, j) - host_global_estimate(1:2, j);
                D_pos = D_pos + pos_diff' * inv(self.sigma2_matrix_gamma_cross(1:2,1:2)) * pos_diff;

                % Velocity difference
                vel_diff = target_global_estimate(4, j) - host_global_estimate(4, j);
                D_vel = D_vel + vel_diff' * inv(self.sigma2_matrix_gamma_cross(4,4)) * vel_diff;

                % Acceleration difference
                acc_diff = target_global_estimate(5, j) - host_global_estimate(5, j);
                D_acc = D_acc + acc_diff' * inv(self.sigma2_matrix_gamma_cross(5,5)) * acc_diff;

                % Heading difference
                theta_diff = target_global_estimate(3, j) - host_global_estimate(3, j);
                D_theta = D_theta + theta_diff' * inv(self.sigma2_matrix_gamma_cross(3,3)) * theta_diff;


                % that is in the paper
                x_diff = target_global_estimate(:, j) - host_global_estimate(:, j);
                D = D + x_diff' * inv(self.sigma2_matrix_gamma_cross) * x_diff; % Mahalanobis distance
            end

            % Compute trust factor
            D_total = D;
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
            target_global_estimate = host_vehicle.center_communication.get_global_state(target_vehicle.vehicle_number , host_id);

            x_l_i = target_global_estimate([1,4], host_id); % In L target vehicle , get the estimate of the host vehicle (i is host)
            % If position errors are around 2m and velocity errors around 1m/s

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
                velocity_diff_measurement = abs(predecessor.state(4) - host_vehicle.state(4));
                y_i_predecessor = [host_distance_measurement; velocity_diff_measurement];

                pred_id = predecessor.vehicle_number;

                x_l_pred = target_global_estimate([1,4], pred_id); % In L car , get the estimate of the predceding of host vehicle (i + 1 is pred)

                % Compute  relative state from global estimate received
                rel_state_est = abs(x_l_pred - x_l_i - [half_lenght_vehicle;0]);

                % Compute consistency error e_i,l^(j)(k)
                e = rel_state_est - y_i_predecessor;

                E =  e' * inv(self.tau2_matrix_gamma_local) * e + E;

            end

            if (~isempty(successor))
                % Get local measurement (e.g., relative position and velocity to successor)
                % Assume local_measurements.successor is a vector [rel_pos; rel_vel]
                host_distance_measurement = (host_vehicle.state(1) - successor.state(1) ) - half_lenght_vehicle;
                velocity_diff_measurement = abs(successor.state(4) - host_vehicle.state(4));
                y_i_successor = [host_distance_measurement; velocity_diff_measurement]; % Host measurement

                successor_id = successor.vehicle_number;

                x_l_successor = target_global_estimate([1,4], successor_id); % In L car , get the estimate of the successor of host vehicle (i - 1 is succ)

                % Compute expected relative state from global estimate
                rel_state_est = abs(x_l_i - x_l_successor - [half_lenght_vehicle;0]);

                % Compute consistency error e_i,l^(j)(k)
                e = (rel_state_est - y_i_successor);
                E = e' * inv(self.tau2_matrix_gamma_local) * e + E;

            end

            % Compute trust factor
            gamma_local = exp(-E);
        end

        function gamma_local_our_self = compute_self_consistency_factor(self, host_vehicle, neighbors)
            % Inputs:
            %   host_vehicle: The vehicle evaluating its own trust
            %   neighbors: Array of neighbor vehicle objects
            %
            % Output:
            %   gamma_local_our_self: Self-consistency trust factor (0 to 1)
            %
            % This method evaluates how well the host vehicle's own global estimate
            % is consistent with its local sensor measurements

            half_lenght_vehicle = host_vehicle.param.l_r; % distance between vehicle's c.g. and rear axle
            host_id = host_vehicle.vehicle_number;
            
            % Get host vehicle's own global estimate
            host_global_estimate = host_vehicle.observer.est_global_state_current;
            x_l_host_self = host_global_estimate([1,4], host_id); % Host's estimate of itself
            
            E = 0; % Total error
            M_i = 0; % Count of neighbors used for comparison
            
            % Find predecessor and successor for consistency check
            predecessor = [];
            successor = [];
            
            for m = 1:length(neighbors)
                car_idx = neighbors(m).vehicle_number;
                if abs(car_idx - host_id) == 1
                    if (car_idx > host_id)
                        M_i = M_i + 1;
                        successor = neighbors(m);
                    end
                    if (car_idx < host_id)
                        M_i = M_i + 1;
                        predecessor = neighbors(m); % Following vehicle
                    end
                end
            end

            % If no predecessor or successor is found, return maximum trust
            if isempty(predecessor) && isempty(successor)
                gamma_local_our_self = 1;
                return;
            end

            % Check consistency with predecessor (if exists)
            if (~isempty(predecessor))
                % Get local measurement to predecessor
                host_distance_measurement = (predecessor.state(1) - host_vehicle.state(1)) - half_lenght_vehicle;
                velocity_diff_measurement = abs(predecessor.state(4) - host_vehicle.state(4));
                y_i_predecessor = [host_distance_measurement; velocity_diff_measurement];

                pred_id = predecessor.vehicle_number;
                x_l_pred = host_global_estimate([1,4], pred_id); % Host's global estimate of predecessor

                % Compute relative state from host's own global estimate
                rel_state_est = abs(x_l_pred - x_l_host_self - [half_lenght_vehicle;0]);

                % Compute consistency error
                e = rel_state_est - y_i_predecessor;
                E = e' * inv(self.tau2_matrix_gamma_local) * e + E;
            end

            % Check consistency with successor (if exists)
            if (~isempty(successor))
                % Get local measurement to successor
                host_distance_measurement = (host_vehicle.state(1) - successor.state(1)) - half_lenght_vehicle;
                velocity_diff_measurement = abs(successor.state(4) - host_vehicle.state(4));
                y_i_successor = [host_distance_measurement; velocity_diff_measurement];

                successor_id = successor.vehicle_number;
                x_l_successor = host_global_estimate([1,4], successor_id); % Host's global estimate of successor

                % Compute relative state from host's own global estimate
                rel_state_est = abs(x_l_host_self - x_l_successor - [half_lenght_vehicle;0]);

                % Compute consistency error
                e = rel_state_est - y_i_successor;
                E = e' * inv(self.tau2_matrix_gamma_local) * e + E;
            end

            % Compute self-consistency trust factor
            gamma_local_our_self = exp(-E);
        end



        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Plotting function %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   %%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function plot_filter_comparison(self, host_vehicle_number, target_vehicle_number)
            % plot_filter_comparison - Plot comparison between raw and filtered scores
            %
            % Inputs:
            %   host_vehicle_number: ID of host vehicle for plot title
            %   target_vehicle_number: ID of target vehicle for plot title
            
            figure('Name', sprintf('Score Filtering Comparison - Host %d, Target %d', host_vehicle_number, target_vehicle_number), ...
                   'Position', [100, 100, 1400, 900]);
            
            % Check if we have filtered data
            if isempty(self.filtered_v_score_log)
                warning('No filtered score data available. Make sure filtering is enabled and trust evaluation has been run.');
                return;
            end
            
            % Create time vector
            time_steps = 1:length(self.v_score_log);
            time_steps_filtered = 1:length(self.filtered_v_score_log);
            
            % Subplot 1: Velocity Score Comparison
            subplot(2, 3, 1);
            if ~isempty(self.v_score_log)
                plot(time_steps, self.v_score_log, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Raw Velocity Score');
                hold on;
            end
            if ~isempty(self.filtered_v_score_log)
                plot(time_steps_filtered, self.filtered_v_score_log, 'r--', 'LineWidth', 2, 'DisplayName', 'Filtered Velocity Score');
            end
            xlabel('Time Step');
            ylabel('Velocity Score');
            title('Velocity Score Filtering');
            legend('show', 'Location', 'best');
            grid on;
            ylim([0, 1]);
            
            % Subplot 2: Distance Score Comparison  
            subplot(2, 3, 2);
            if ~isempty(self.d_score_log)
                plot(time_steps, self.d_score_log, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Raw Distance Score');
                hold on;
            end
            if ~isempty(self.filtered_d_score_log)
                plot(time_steps_filtered, self.filtered_d_score_log, 'r--', 'LineWidth', 2, 'DisplayName', 'Filtered Distance Score');
            end
            xlabel('Time Step');
            ylabel('Distance Score');
            title('Distance Score Filtering');
            legend('show', 'Location', 'best');
            grid on;
            ylim([0, 1]);
            
            % Subplot 3: Acceleration Score Comparison
            subplot(2, 3, 3);
            if ~isempty(self.a_score_log)
                plot(time_steps, self.a_score_log, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Raw Acceleration Score');
                hold on;
            end
            if ~isempty(self.filtered_a_score_log)
                plot(time_steps_filtered, self.filtered_a_score_log, 'r--', 'LineWidth', 2, 'DisplayName', 'Filtered Acceleration Score');
            end
            xlabel('Time Step');
            ylabel('Acceleration Score');
            title('Acceleration Score Filtering');
            legend('show', 'Location', 'best');
            grid on;
            ylim([0, 1]);
            
            % Subplot 4: Beacon Score Comparison
            subplot(2, 3, 4);
            if ~isempty(self.beacon_score_log)
                plot(time_steps, self.beacon_score_log, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Raw Beacon Score');
                hold on;
            end
            if ~isempty(self.filtered_beacon_score_log)
                plot(time_steps_filtered, self.filtered_beacon_score_log, 'r--', 'LineWidth', 2, 'DisplayName', 'Filtered Beacon Score');
            end
            xlabel('Time Step');
            ylabel('Beacon Score');
            title('Beacon Score Filtering');
            legend('show', 'Location', 'best');
            grid on;
            ylim([0, 1]);
            
            % Subplot 5: Heading Score Comparison
            subplot(2, 3, 5);
            if ~isempty(self.h_score_log)
                plot(time_steps, self.h_score_log, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Raw Heading Score');
                hold on;
            end
            if ~isempty(self.filtered_h_score_log)
                plot(time_steps_filtered, self.filtered_h_score_log, 'r--', 'LineWidth', 2, 'DisplayName', 'Filtered Heading Score');
            end
            xlabel('Time Step');
            ylabel('Heading Score');
            title('Heading Score Filtering');
            legend('show', 'Location', 'best');
            grid on;
            ylim([0, 1]);
            
            % Subplot 6: Filter Configuration Summary
            subplot(2, 3, 6);
            axis off;
            
            % Create text summary of filter configuration
            config_text = {
                'Filter Configuration:',
                sprintf('Master Filter: %s', string(self.enable_score_filtering)),
                '',
                sprintf('Velocity Filter: %s (%s)', string(self.enable_velocity_filter), self.velocity_filter_type),
                sprintf('Distance Filter: %s (%s)', string(self.enable_distance_filter), self.distance_filter_type),
                sprintf('Acceleration Filter: %s (%s)', string(self.enable_acceleration_filter), self.acceleration_filter_type),
                sprintf('Beacon Filter: %s (%s)', string(self.enable_beacon_filter), self.beacon_filter_type),
                sprintf('Heading Filter: %s (%s)', string(self.enable_heading_filter), self.heading_filter_type),
                '',
                sprintf('Filter Window Size: %d', self.filter_window_size),
                sprintf('Exponential Alpha: %.2f', self.filter_alpha),
                sprintf('Threshold Min: %.2f', self.filter_threshold_min),
                sprintf('Threshold Max: %.2f', self.filter_threshold_max)
            };
            
            text(0.05, 0.95, config_text, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
                 'FontSize', 10, 'FontName', 'FixedWidth', 'Interpreter', 'none');
            
            sgtitle(sprintf('Trust Score Filtering Analysis - Host Vehicle %d vs Target Vehicle %d', ...
                           host_vehicle_number, target_vehicle_number), 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        function plot_filter_effectiveness(self, host_vehicle_number, target_vehicle_number)
            % plot_filter_effectiveness - Plot metrics showing filter effectiveness
            %
            % Shows noise reduction, smoothness improvement, and response characteristics
            
            figure('Name', sprintf('Filter Effectiveness Metrics - Host %d, Target %d', host_vehicle_number, target_vehicle_number), ...
                   'Position', [150, 150, 1200, 800]);
            
            % Check if we have data
            if isempty(self.filtered_v_score_log) || isempty(self.v_score_log)
                warning('Insufficient data for effectiveness analysis.');
                return;
            end
            
            % Align data lengths (take minimum length)
            min_length = min([length(self.v_score_log), length(self.filtered_v_score_log), ...
                             length(self.d_score_log), length(self.filtered_d_score_log), ...
                             length(self.a_score_log), length(self.filtered_a_score_log)]);
                         
            time_steps = 1:min_length;
            
            % Calculate metrics for each score type
            score_types = {'Velocity', 'Distance', 'Acceleration'};
            raw_scores = {self.v_score_log(1:min_length), self.d_score_log(1:min_length), self.a_score_log(1:min_length)};
            filtered_scores = {self.filtered_v_score_log(1:min_length), self.filtered_d_score_log(1:min_length), self.filtered_a_score_log(1:min_length)};
            
            for i = 1:3
                subplot(2, 3, i);
                
                raw_data = raw_scores{i};
                filtered_data = filtered_scores{i};
                
                % Plot raw vs filtered
                plot(time_steps, raw_data, 'b-', 'LineWidth', 1, 'DisplayName', 'Raw');
                hold on;
                plot(time_steps, filtered_data, 'r-', 'LineWidth', 2, 'DisplayName', 'Filtered');
                
                xlabel('Time Step');
                ylabel('Score Value');
                title(sprintf('%s Score Comparison', score_types{i}));
                legend('show', 'Location', 'best');
                grid on;
                ylim([0, 1]);
            end
            
            % Calculate and display effectiveness metrics
            subplot(2, 3, 4);
            axis off;
            
            % Calculate variance reduction for each score type
            variance_reduction = zeros(1, 3);
            smoothness_improvement = zeros(1, 3);
            
            for i = 1:3
                if length(raw_scores{i}) > 1 && length(filtered_scores{i}) > 1
                    raw_variance = var(raw_scores{i});
                    filtered_variance = var(filtered_scores{i});
                    variance_reduction(i) = (raw_variance - filtered_variance) / raw_variance * 100;
                    
                    % Calculate smoothness (inverse of derivative variance)
                    raw_smoothness = var(diff(raw_scores{i}));
                    filtered_smoothness = var(diff(filtered_scores{i}));
                    if raw_smoothness > 0
                        smoothness_improvement(i) = (raw_smoothness - filtered_smoothness) / raw_smoothness * 100;
                    end
                end
            end
            
            % Display metrics
            metrics_text = {
                'Filter Effectiveness Metrics:',
                '',
                'Variance Reduction (%):',
                sprintf('  Velocity: %.1f%%', variance_reduction(1)),
                sprintf('  Distance: %.1f%%', variance_reduction(2)),
                sprintf('  Acceleration: %.1f%%', variance_reduction(3)),
                '',
                'Smoothness Improvement (%):',
                sprintf('  Velocity: %.1f%%', smoothness_improvement(1)),
                sprintf('  Distance: %.1f%%', smoothness_improvement(2)),
                sprintf('  Acceleration: %.1f%%', smoothness_improvement(3))
            };
            
            text(0.05, 0.95, metrics_text, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
                 'FontSize', 10, 'FontName', 'FixedWidth');
            
            % Plot histograms of score distributions
            subplot(2, 3, 5);
            if ~isempty(raw_scores{1}) && ~isempty(filtered_scores{1})
                histogram(raw_scores{1}, 20, 'Alpha', 0.6, 'DisplayName', 'Raw Velocity', 'Normalization', 'probability');
                hold on;
                histogram(filtered_scores{1}, 20, 'Alpha', 0.6, 'DisplayName', 'Filtered Velocity', 'Normalization', 'probability');
                xlabel('Score Value');
                ylabel('Probability');
                title('Score Distribution Comparison');
                legend('show');
            end
            
            % Plot correlation analysis
            subplot(2, 3, 6);
            if length(raw_scores{1}) == length(filtered_scores{1}) && length(raw_scores{1}) > 1
                scatter(raw_scores{1}, filtered_scores{1}, 'filled', 'Alpha', 0.6);
                hold on;
                plot([0, 1], [0, 1], 'r--', 'LineWidth', 2);
                xlabel('Raw Score');
                ylabel('Filtered Score');
                title('Raw vs Filtered Correlation');
                grid on;
                axis equal;
                xlim([0, 1]);
                ylim([0, 1]);
                
                % Calculate and display correlation coefficient
                if length(raw_scores{1}) > 1
                    correlation = corrcoef(raw_scores{1}, filtered_scores{1});
                    text(0.05, 0.95, sprintf('R = %.3f', correlation(1,2)), 'Units', 'normalized');
                end
            end
            
            sgtitle(sprintf('Filter Effectiveness Analysis - Host %d vs Target %d', host_vehicle_number, target_vehicle_number), ...
                    'FontSize', 14, 'FontWeight', 'bold');
        end

        function plot_details_acc_score(self, host_vehicle_number, target_vehicle_number)
            % Plot acceleration score over time in a 3x3 grid layout
            figure("Name", "Details Acc Score " + host_vehicle_number + "->" + target_vehicle_number);

            tiledlayout(3, 3, 'TileSpacing', 'compact'); % use compact spacing

            % Create time vector for plotting
            num_time_steps = length(self.v_rel_log);
            dt_config = 0.01; % Default dt from Config.m
            time_vector = 0:dt_config:(num_time_steps-1)*dt_config;

            % Subplot 1: Relative Velocity
            nexttile;
            plot(time_vector, self.v_rel_log, 'DisplayName', 'Relative Velocity', 'LineWidth', 1.5);
            title('$v_{rel}$', 'Interpreter', 'latex');
            ylim([-2 2]);
            grid on;

            % Subplot 2: Additional Distance
            nexttile;
            plot(time_vector, self.d_add_log, 'DisplayName', 'Additional Distance', 'LineWidth', 1.5);
            title('$d_{add}$', 'Interpreter', 'latex');
            grid on;

            % Subplot 3: Relative Acceleration
            nexttile;
            plot(time_vector, self.acc_rel_log, 'DisplayName', 'Relative Acceleration', 'LineWidth', 1.5);
            title('$acc_{rel}$', 'Interpreter', 'latex');
            grid on;

            % Subplot 4: Delta Acc Expected
            nexttile;
            plot(time_vector, self.delta_acc_expected_log, 'DisplayName', 'Expected Acceleration Difference', 'LineWidth', 1.5);
            title('$\Delta acc_{expec}$', 'Interpreter', 'latex');
            grid on;

            % Subplot 5: Distance
            nexttile;
            plot(time_vector, self.distance_log, 'DisplayName', 'Distance Log', 'LineWidth', 1.5, 'Color', 'magenta');
            title('Distance');
            ylabel('Meters');
            grid on;

            % Subplot 6: Scale d Expected
            nexttile;
            plot(time_vector, self.scale_d_expected_log, 'DisplayName', 'Scaled Expected Distance', 'LineWidth', 1.5);
            xlabel('Time (s)');
            title(['Scale $$d_{expected}$$ ' num2str(host_vehicle_number) ' $->$ ' num2str(target_vehicle_number)], 'Interpreter', 'latex');
            grid on;

            % Subplot 7: Delta d
            nexttile;
            plot(time_vector, self.delta_d_log, 'DisplayName', 'Delta Distance', 'LineWidth', 1.5);
            title('$\Delta d$', 'Interpreter', 'latex');
            ylabel('Meters');
            grid on;

            % If you want, you can leave the last two tiles blank or use them for legends, summary text, etc.
        end


        function plot_diff_acc_score(self , host_vehicle_number, target_vehicle_number)

            figure("Name", "Diff Acc Score"+ host_vehicle_number + "->" + target_vehicle_number);
            
            % Create time vector for plotting
            num_time_steps = length(self.a_score_defaut_log);
            dt_config = 0.01; % Default dt from Config.m
            time_vector = 0:dt_config:(num_time_steps-1)*dt_config;
            
            subplot(5,1,1);
            plot(time_vector, self.a_score_defaut_log, 'DisplayName', 'Default Acc Score', 'LineWidth', 1.5);
            title('Default Acc Score TrIP $\frac{v_{rel}}{T_s}$', 'Interpreter', 'latex');
            grid on;
            subplot(5,1,2);
            plot(time_vector, self.a_score_vrel_dis_adjusted_log , 'DisplayName', 'Diff Acc Score', 'LineWidth', 1.5);
            title('New Acc Score with d_add for smothing');
            grid on;
            subplot(5,1,3);
            plot(time_vector, self.a_score_expected_diff_acc_log, 'DisplayName', 'Expected Acceleration', 'LineWidth', 1.5);
            title('Expected Diff Acceleration $\Delta acc_{expec}$', 'Interpreter', 'latex');
            grid on;
            subplot(5,1,4);
            plot(time_vector, self.a_score_vrel_dis_log, 'DisplayName', 'Vrel Dist', 'LineWidth', 1.5);
            title('Acc use true host Dist $\frac{v_{rel}}{d(1)}$', 'Interpreter', 'latex');
            subplot(5,1,5);
            plot(time_vector, self.a_score_mathematical_log, 'DisplayName', 'Vrel Dist', 'LineWidth', 1.5);
            title('Paper now', 'Interpreter', 'latex');
            xlabel('Time (s)');

            


        end


        function plot_trust_log(self,nb_host_car , nb_target_car, dt_config)
            if nargin < 4
                dt_config = 0.01; % Default dt from Config.m
            end

            [time_vector, local_most_impacted, local_impact_idx, ...
                global_most_impacted, global_impact_idx, local_trust, ...
                global_trust, local_component_names, global_component_names] = ...
                self.get_most_impacted_score_traces(dt_config);

            if isempty(time_vector)
                warning('No trust logs available for %d -> %d.', nb_host_car, nb_target_car);
                return;
            end

            num_time_steps = length(time_vector);
            gamma_cross = self.plot_log_row(self.gamma_cross_log, num_time_steps);
            gamma_local = self.plot_log_row(self.gamma_local_log, num_time_steps);
            gamma_self = self.plot_log_row(self.gamma_local_our_self_log, num_time_steps);
            v_score = self.plot_log_row(self.v_score_log, num_time_steps);
            d_score = self.plot_log_row(self.d_score_log, num_time_steps);
            a_score = self.plot_log_row(self.a_score_log, num_time_steps);
            h_score = self.plot_log_row(self.h_score_log, num_time_steps);
            final_score = self.plot_log_row(self.final_score_log, num_time_steps);

            figure("Name", num2str(nb_host_car) +  " Trust for " + num2str(nb_target_car), ...
                "NumberTitle", "off", "Position", [120, 60, 1250, 950]);

            subplot(5,1,1);
            plot(time_vector, gamma_cross, 'DisplayName', 'Gamma Cross', 'LineWidth', 1);
            hold on;
            plot(time_vector, gamma_local, 'DisplayName', 'Gamma Local','LineWidth', 1);
            plot(time_vector, gamma_self, 'DisplayName', 'Gamma Self','LineWidth', 1, 'LineStyle', '--');
            plot(time_vector, global_trust, 'DisplayName', 'Global Trust','LineWidth', 1.5);
            grid on;
            legend show;

            %% Not use
            % plot(time_vector, self.gamma_expected_log, 'DisplayName', 'Gamma expect', 'LineWidth', 1);


            subplot(5,1,2);
            plot(time_vector, a_score, 'DisplayName', 'A Score', 'LineWidth', 1);
            hold on;
            plot(time_vector, v_score, 'DisplayName', 'V Score', 'LineWidth', 1);
            plot(time_vector, d_score, 'DisplayName', 'D Score', 'LineWidth', 1);
            plot(time_vector, h_score, 'DisplayName', 'H Score', 'LineWidth', 1);
            plot(time_vector, local_trust, 'DisplayName', 'Local Trust', 'LineWidth', 1.5);
            grid on;

            legend show;

            subplot(5,1,3);
            self.plot_most_impacted_subplot(time_vector, local_trust, local_most_impacted, ...
                local_impact_idx, local_component_names, 'Local Trust', ...
                'Most impacted local trust score');

            subplot(5,1,4);
            self.plot_most_impacted_subplot(time_vector, global_trust, global_most_impacted, ...
                global_impact_idx, global_component_names, 'Global Trust', ...
                'Most impacted global trust score');

            subplot(5,1,5);
            plot(time_vector, final_score, 'DisplayName', 'Final Score' , 'LineWidth', 1.5);

            % plot(time_vector, self.beacon_score_log, 'DisplayName', 'Beacon Score');

            xlabel('Time (s)');
            ylabel('Value');
            title([num2str(nb_host_car) '-> Trust and Score Logs Over Time for car '  num2str(nb_target_car)],"LineWidth",1);
            legend show;
            grid on;

            hold off;
        end

        function [time_vector, local_most_impacted, local_impact_idx, ...
                global_most_impacted, global_impact_idx, local_trust, ...
                global_trust, local_component_names, global_component_names] = ...
                get_most_impacted_score_traces(self, dt_config)
            if nargin < 2
                dt_config = 0.01;
            end

            local_component_names = {'Velocity', 'Distance', 'Acceleration', 'Heading', 'Local Beacon'};
            global_component_names = {'Gamma Cross', 'Gamma Local', 'Gamma Self', 'Global Beacon'};
            fusion_mode = char(lower(strtrim(string(self.local_trust_fusion_mode))));
            if any(strcmp(fusion_mode, {'product', 'direct_product', 'multiply', 'equal', 'equal_geometric', 'equal_geomean'}))
                local_component_weights = [1, 1, 1, 1, 2];
            else
                local_component_weights = [ ...
                    self.local_weight_velocity, ...
                    self.local_weight_distance, ...
                    self.local_weight_acceleration, ...
                    self.local_weight_heading, ...
                    self.local_weight_beacon + self.local_weight_quality];
            end
            global_component_weights = ones(1, numel(global_component_names));

            num_time_steps = max([ ...
                length(self.v_score_log), ...
                length(self.d_score_log), ...
                length(self.a_score_log), ...
                length(self.h_score_log), ...
                length(self.beacon_score_local_log), ...
                length(self.gamma_cross_log), ...
                length(self.gamma_local_log), ...
                length(self.gamma_local_our_self_log), ...
                length(self.beacon_score_global_log), ...
                length(self.trust_sample_log), ...
                length(self.final_score_log)]);

            if num_time_steps == 0
                time_vector = [];
                local_most_impacted = [];
                local_impact_idx = [];
                global_most_impacted = [];
                global_impact_idx = [];
                local_trust = [];
                global_trust = [];
                return;
            end

            time_vector = (0:(num_time_steps - 1)) * dt_config;
            local_trust = self.plot_log_row(self.trust_sample_log, num_time_steps);

            gamma_cross = self.plot_log_row(self.gamma_cross_log, num_time_steps);
            gamma_local = self.plot_log_row(self.gamma_local_log, num_time_steps);
            gamma_self = self.plot_log_row(self.gamma_local_our_self_log, num_time_steps);
            global_trust = gamma_cross .* gamma_local;

            local_components = [
                self.plot_log_row(self.v_score_log, num_time_steps);
                self.plot_log_row(self.d_score_log, num_time_steps);
                self.plot_log_row(self.a_score_log, num_time_steps);
                self.plot_log_row(self.h_score_log, num_time_steps);
                self.plot_log_row(self.beacon_score_local_log, num_time_steps)];

            global_components = [
                gamma_cross;
                gamma_local;
                gamma_self;
                self.plot_log_row(self.beacon_score_global_log, num_time_steps)];

            [local_most_impacted, local_impact_idx] = ...
                self.most_impacted_from_components(local_components, local_component_weights);
            [global_most_impacted, global_impact_idx] = ...
                self.most_impacted_from_components(global_components, global_component_weights);
        end

        function row = plot_log_row(~, values, n)
            row = NaN(1, n);
            if isempty(values) || n == 0
                return;
            end
            values = double(values(:))';
            m = min(numel(values), n);
            row(1:m) = values(1:m);
        end

        function [impact_score, impact_idx] = most_impacted_from_components(~, component_matrix, component_weights)
            if nargin < 3 || isempty(component_weights) || numel(component_weights) ~= size(component_matrix, 1)
                component_weights = ones(1, size(component_matrix, 1));
            end

            finite_scores = isfinite(component_matrix);
            safe_scores = max(min(component_matrix, 1), 0.01);
            component_impact = -log(safe_scores) .* component_weights(:);
            component_impact(~finite_scores) = -Inf;

            [~, impact_idx] = max(component_impact, [], 1);
            impact_score = NaN(1, size(component_matrix, 2));

            valid_columns = any(finite_scores, 1);
            if any(valid_columns)
                columns = find(valid_columns);
                selected_rows = impact_idx(columns);
                selected_indices = sub2ind(size(component_matrix), selected_rows, columns);
                impact_score(columns) = component_matrix(selected_indices);
            end
            impact_idx(~valid_columns) = NaN;
        end

        function plot_most_impacted_subplot(~, time_vector, trust_trace, impact_score, impact_idx, ...
                component_names, trust_label, plot_title)
            plot(time_vector, trust_trace, 'k-', 'DisplayName', trust_label, 'LineWidth', 1.5);
            hold on;
            plot(time_vector, impact_score, 'Color', [0.80, 0.15, 0.10], ...
                'DisplayName', 'Most impacted score', 'LineWidth', 1.1);

            colors = lines(max(1, numel(component_names)));
            for component_idx = 1:numel(component_names)
                marker_idx = impact_idx == component_idx & isfinite(impact_score);
                if any(marker_idx)
                    scatter(time_vector(marker_idx), impact_score(marker_idx), 12, ...
                        colors(component_idx, :), 'filled', ...
                        'DisplayName', component_names{component_idx});
                end
            end

            hold off;
            title(plot_title);
            ylabel('Score');
            ylim([0, 1.05]);
            grid on;
            legend('show', 'Location', 'eastoutside');
        end

        function plot_validation_metrics(self, host_vehicle_number, target_vehicle_number)
            % Plot the new validation metrics: physical constraints and temporal consistency
            figure("Name", "Validation Metrics " + host_vehicle_number + "->" + target_vehicle_number);
            
            subplot(2,1,1);
            plot(self.physical_valid_log, 'DisplayName', 'Physical Valid', 'LineWidth', 1.5, 'LineStyle', '-', 'Marker', 'o');
            title('Physical Constraints Validation');
            ylabel('Valid (1) / Invalid (0)');
            ylim([-0.1, 1.1]);
            grid on;
            legend show;
            
            subplot(2,1,2);
            plot(self.temporal_score_log, 'DisplayName', 'Temporal Consistency Score', 'LineWidth', 1.5, 'Color', 'red');
            title('Temporal Consistency Score');
            xlabel('Time Step');
            ylabel('Consistency Score (0-1)');
            ylim([0, 1]);
            grid on;
            legend show;
        end

        function plot_trust_decay_effects(self, host_vehicle_number, target_vehicle_number)
            % Plot trust decay effects on local and global trust
            figure("Name", "Trust Decay Effects " + host_vehicle_number + "->" + target_vehicle_number);
            
            % Check if we have decay data
            if isempty(self.local_trust_decayed_log) || isempty(self.global_trust_decayed_log)
                warning('No trust decay data available for plotting');
                return;
            end
            
            subplot(4,1,1);
            % Compare original vs decayed local trust
            plot(self.trust_sample_log, 'DisplayName', 'Original Local Trust', 'LineWidth', 1.5, 'LineStyle', '-');
            hold on;
            plot(self.local_trust_decayed_log, 'DisplayName', 'Decayed Local Trust', 'LineWidth', 1.5, 'LineStyle', '--');
            title('Local Trust: Original vs Decayed');
            ylabel('Trust Score');
            ylim([0, 1]);
            grid on;
            legend show;
            
            subplot(4,1,2);
            % Compare original vs decayed global trust (using gamma_cross * gamma_local as proxy for original)
            if length(self.gamma_cross_log) == length(self.gamma_local_log) && ...
               length(self.gamma_cross_log) == length(self.global_trust_decayed_log)
                original_global = self.gamma_cross_log .* self.gamma_local_log;
                plot(original_global, 'DisplayName', 'Original Global Trust', 'LineWidth', 1.5, 'LineStyle', '-');
                hold on;
                plot(self.global_trust_decayed_log, 'DisplayName', 'Decayed Global Trust', 'LineWidth', 1.5, 'LineStyle', '--');
                title('Global Trust: Original vs Decayed');
                ylabel('Trust Score');
                ylim([0, 1]);
                grid on;
                legend show;
            end
            
            subplot(4,1,3);
            % Show separate beacon reception patterns
            plot(self.beacon_score_local_log, 'DisplayName', 'Local Channel Beacon', 'LineWidth', 1.5, 'Marker', 'o', 'MarkerSize', 3);
            hold on;
            plot(self.beacon_score_global_log, 'DisplayName', 'Global Channel Beacon', 'LineWidth', 1.5, 'Marker', 's', 'MarkerSize', 3);
            title('Separate Channel Beacon Reception (δ_{i,l}^{local}(t) & δ_{i,l}^{global}(t))');
            ylabel('Beacon Received (1/0)');
            ylim([-0.1, 1.1]);
            grid on;
            legend show;
            
            subplot(4,1,4);
            % Show combined beacon pattern (both channels must work)
            plot(self.beacon_score_log, 'DisplayName', 'Combined Beacon (Local & Global)', 'LineWidth', 1.5, 'Marker', 'd', 'MarkerSize', 3, 'Color', 'red');
            title('Combined Beacon Reception (Both Channels Required)');
            xlabel('Time Step');
            ylabel('Both Beacons (1/0)');
            ylim([-0.1, 1.1]);
            grid on;
            legend show;
        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Not Use %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function gamma_cross_expected = compute_cross_host_expected_factor(self, host_id, host_vehicle, target_id, target_vehicle)
            % Get target vehicle's global estimate
            target_global_estimate = target_vehicle.center_communication.get_global_state(target_id, host_id);

            % Define variances for covariance matrix
            var_x = 3;         % Variance for X-difference
            var_y = 1;         % Variance for Y-difference
            var_angle = 0.01;  % Variance for angle-difference
            var_velocity = 1;  % Variance for velocity-difference
            sigma2_matrix = diag([var_x, var_y, var_angle, var_velocity]);

            % Retrieve control parameters (assumed available as properties or from the vehicle)
            s0      = 8;       % Minimum spacing (m)
            h_base  = 0.5;   % Base time headway (s) for the first follower (vehicle 2)
            delta_h = 0.1;  % Headway reduction per position in the platoon (s)
            if ~isempty(host_vehicle.gamma_log)
                gamma_control = host_vehicle.gamma_log(end);
            else
                gamma_control = 1;
            end
            % Initialize total discrepancy
            D = 0;
            num_vehicles = size(target_global_estimate, 2);

            % Loop over consecutive vehicle pairs in the global estimate
            for j = 1:num_vehicles - 1
                % Difference between vehicle j and j+1 in the global state vector
                state_diff = target_global_estimate(:, j) - target_global_estimate(:, j+1);

                % Compute adaptive expected spacing for the follower (vehicle n = j+1)
                n = j + 1; % Vehicle index in platoon (leader is n=1)
                % Adaptive headway for this vehicle
                h_n = h_base - delta_h * (n - 2);

                % Assume state vector: [x; y; angle; velocity]
                % Use the velocity of the follower vehicle (j+1) for the spacing calculation
                v = target_global_estimate(4, j+1);

                % Compute expected spacing using the mixing formula:
                % s_expected = s0 + (gamma*h_base + (1-gamma)*h_n)*v
                s_expected = s0 + ((1 - gamma_control) * h_base + gamma_control * h_n) * v;

                % Define the expected difference vector for this gap. Only the X-component is adaptive.
                mu_diff_gap = [s_expected; 0; 0; 0];

                % Calculate difference from expected value for this pair
                diff_from_expected = state_diff - mu_diff_gap;

                % Accumulate the discrepancy using the Mahalanobis distance
                D = D + diff_from_expected' * inv(sigma2_matrix) * diff_from_expected;
            end

            % Compute the cross expected trust factor as an exponential decay with the total discrepancy
            gamma_cross_expected = exp(-D);
        end

        function gamma_cross_expected = compute_cross_host_expected_2_factor(self, host_id, host_vehicle, target_id, target_vehicle)
            % Get target vehicle's global estimate
            target_global_estimate = target_vehicle.center_communication.get_global_state(target_id, host_id);

            % Define variances for covariance matrix
            var_x = 3;         % Variance for X-difference
            var_y = 1;         % Variance for Y-difference
            var_angle = 0.01;  % Variance for angle-difference
            var_velocity = 0.5;  % Variance for velocity-difference
            sigma2_matrix = diag([var_x, var_y, var_angle, var_velocity]);

            % Retrieve control parameters (assumed available as properties or from the vehicle)
            s0      = 8;       % Minimum spacing (m)
            h_base  = 0.4;   % Base time headway (s) for the first follower (vehicle 2)
            T = 0.5;
            delta_h = 0.1;  % Headway reduction per position in the platoon (s)
            if ~isempty(host_vehicle.gamma_log)
                gamma_control = host_vehicle.gamma_log(end);
            else
                gamma_control = 1;
            end
            % Initialize total discrepancy
            D = 0;
            num_vehicles = size(target_global_estimate, 2);

            % Loop over consecutive vehicle pairs in the global estimate
            for j = 1:num_vehicles - 1
                % Difference between vehicle j and j+1 in the global state vector
                state_diff = target_global_estimate(:, j) - target_global_estimate(:, j+1);

                % Assume state vector: [x; y; angle; velocity]
                % Use the velocity of the follower vehicle (j+1) for the spacing calculation
                v = target_global_estimate(4, j+1);

                s_acc = s0 + T*v; % h_base = T
                s_i = s0 + h_base*v/j;
                % Compute expected spacing using the mixing formula:
                % s_expected = s0 + (gamma*h_base + (1-gamma)*h_n)*v
                s_expected =  ((1 - gamma_control) * s_acc + gamma_control * s_i) ;
                % U_final = self.input + tau_filter*(U_target - self.input) ; % self.input is last input

                % Define the expected difference vector for this gap. Only the X-component is adaptive.
                mu_diff_gap = [s_expected; 0; 0; 0];

                % Calculate difference from expected value for this pair
                diff_from_expected = state_diff - mu_diff_gap;

                % Accumulate the discrepancy using the Mahalanobis distance
                D = D + diff_from_expected' * inv(sigma2_matrix) * diff_from_expected;
            end

            % Compute the cross expected trust factor as an exponential decay with the total discrepancy
            gamma_cross_expected = exp(-D);
        end



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
        
        %%% Filter Usage Examples %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   %%%%%%%%%%%%%%%%%%%
        function demonstrate_filter_usage(self)
            % demonstrate_filter_usage - Example of how to configure and use the filtering system
            %
            % This method shows different ways to configure the filtering system
            
            fprintf('\n=== Trust Score Filtering System Demo ===\n\n');
            
            % Example 1: Enable all filters with moving average
            fprintf('1. Enabling all filters with moving average:\n');
            self.enable_score_filtering = true;
            self.enable_velocity_filter = true;
            self.enable_distance_filter = true;
            self.enable_acceleration_filter = true;
            self.enable_beacon_filter = true;
            self.enable_heading_filter = true;
            
            self.velocity_filter_type = 'moving_average';
            self.distance_filter_type = 'moving_average';
            self.acceleration_filter_type = 'moving_average';
            self.beacon_filter_type = 'moving_average';
            self.heading_filter_type = 'moving_average';
            
            self.filter_window_size = 5;
            fprintf('   - All filters enabled with moving average (window size = %d)\n', self.filter_window_size);
            
            % Example 2: Configure specific filter types for different scores
            fprintf('\n2. Configuring specific filter types:\n');
            self.velocity_filter_type = 'exponential';      % Smooth velocity changes
            self.distance_filter_type = 'median';           % Reject distance outliers  
            self.acceleration_filter_type = 'outlier_rejection'; % Handle acceleration spikes
            self.beacon_filter_type = 'threshold';          % Binary beacon filtering
            self.heading_filter_type = 'adaptive_weighted'; % Recent heading emphasis
            
            fprintf('   - Velocity: exponential filter (alpha = %.2f)\n', self.filter_alpha);
            fprintf('   - Distance: median filter\n');
            fprintf('   - Acceleration: outlier rejection filter\n');
            fprintf('   - Beacon: threshold filter (min = %.1f, max = %.1f)\n', self.filter_threshold_min, self.filter_threshold_max);
            fprintf('   - Heading: adaptive weighted filter\n');
            
            % Example 3: Using the structured configuration method
            fprintf('\n3. Using structured configuration:\n');
            filter_configs = struct();
            filter_configs.velocity = struct('enable', true, 'type', 'moving_average');
            filter_configs.distance = struct('enable', true, 'type', 'median');
            filter_configs.acceleration = struct('enable', false, 'type', 'none');
            filter_configs.beacon = struct('enable', true, 'type', 'threshold');
            filter_configs.heading = struct('enable', true, 'type', 'exponential');
            
            self.configure_filtering(true, filter_configs);
            fprintf('   - Configured via struct: velocity, distance, beacon, heading filtered\n');
            fprintf('   - Acceleration filtering disabled\n');
            
            % Example 4: Disable all filtering
            fprintf('\n4. Disabling all filtering:\n');
            self.enable_score_filtering = false;
            fprintf('   - Master filter switch disabled\n');
            fprintf('   - All scores will pass through unfiltered\n');
            
            % Example 5: Show how to use filtered trust calculation
            fprintf('\n5. Usage in trust calculation:\n');
            fprintf('   Replace:\n');
            fprintf('     trust_sample = self.calculate_trust_sample(v_score, d_score, a_score, beacon_score, h_score, is_nearby);\n');
            fprintf('   With:\n');
            fprintf('     trust_sample = self.calculate_trust_sample_with_filtering(v_score, d_score, a_score, beacon_score, h_score, is_nearby);\n');
            
            % Example 6: Plotting filtered results
            fprintf('\n6. Visualizing filter effects:\n');
            fprintf('   Use these methods after simulation:\n');
            fprintf('     trust_model.plot_filter_comparison(host_id, target_id);\n');
            fprintf('     trust_model.plot_filter_effectiveness(host_id, target_id);\n');
            
            fprintf('\n=== Filter Configuration Complete ===\n\n');
        end
    end
end
