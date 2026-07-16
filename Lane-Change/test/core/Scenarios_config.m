classdef Scenarios_config < handle
    properties
        dt ;
        simulation_time ;
        where ; % 1 indicates highway, 2 indicates urban road
        model_vehicle_type = "normal"; % "delay_v" , "delay_a" , "normal"

        lead_senario = "constant"; % "constant" , "Acceleration" , "Deceleration" , "Lane_change"
        
        lead_input = 0; % lead input
        debug_mode = false; % debug mode

        %%%% Observer related

        Use_predict_observer = true; % if use predict observer
        predict_controller_type = "true_other"; % "self" , "true_other" , "predict_other"
        Local_observer_type = "kalman"; % "mesurement" , "kalman" , "observer"
        Is_noise_mesurement = false; % if the noise is in the mesurement
        noise_probability = 0.3; % Probability of adding measurement noise
        Use_smooth_filter = true; % if using smooth filtering for noise mesurement 
        Use_smooth_filter_in_local_observer = true; % if using smooth filtering in local observer
        measurement_noise_variance = [0.15, 0.005, 0.003, 0.01, 0.0003]; % R diagonal for noisy measurement [x, y, theta, v, a]
        process_noise_variance = [0.01, 0.001, 0.0005, 0.02, 0.0005]; % Q diagonal for noisy process [x, y, theta, v, a]
        no_noise_measurement_variance = [0.01, 0.005, 0.0001, 0.005, 0.0002]; % R diagonal when Is_noise_mesurement is false
        no_noise_process_variance = [0.01, 0.001, 0.005, 0.02, 0.0005]; % Q diagonal when Is_noise_mesurement is false
        measurement_noise_correlation = 0.8; % Temporal correlation for smooth measurement noise
        noise_filter_alpha = 0.7; % Measurement-noise smoothing alpha
        local_observer_output_filter_alpha = 0.3; % Local observer output smoothing alpha

        Dichiret_type = "Dual"; % Python config uses separate local/global rating vectors
        Monitor_sudden_change = false; % if the sudden change is monitored
        use_local_data_from_other = true; % if the local data from other vehicles is used

        % Canonical Python-compatible fleet-estimator/weight configuration.
        % Keep these values on the scenario object so every batch case can
        % construct an independent numerical Weight_Trust_module from the
        % same immutable settings.
        fleet_estimator_parity_mode = true;
        trust_threshold = 0.5;
        weight_type = "trust_based";
        w0_fixed = 0.4;
        w_self_base = 0.2;
        w_cap = 0.4;
        kappa = 3;
        eta = 0.15;
        enable_smoothing = false;
        startup_fixed_duration_s = 0.5;
        use_gamma_self_weight_adaptation = true;
        gamma_self_weight_floor = 0.25;
        include_target_self_fleet_estimate = true;
        local_bad_zero_w0_neighbor_total_cap = 0.05;
        flag_w0_target_attack_factor = 0.25;
        flag_w0_global_est_check_factor = 1.25;
        flag_w0_local_est_check_factor = 0.5;
        use_distance_weighting = false;
        use_generalized_trust_vector = true;
        trust_vector_theta_min = 0.4;

        rollback_enabled = false; % Enable/disable rollback functionality
        trust_warmup_time = 0; % Seconds to relax scoring and suppress trust flags/rollback
        trust_warmup_tolerance_scale = 1.0; % Multiplier for local trust tolerances during warm-up
        local_trust_flag_required_samples = 1; % Consecutive low local samples needed before flagging
        local_trust_flag_threshold = 0.5; % Local trust threshold for local-estimate check flag
        rollback_start_time = 5.0; % MATLAB gate corresponding to Python startup rollback suppression
        rollback_startup_suppress_duration_s = 5.0;
        rollback_trigger_delay_steps = 0;
        rollback_required_bad_steps = 1; % Consecutive bad trust/flag steps before rollback
        rollback_window_size = 16;
        rollback_trusted_state_history_size = 60;
        rollback_trusted_state_guard_steps = 8;
        rollback_rewrite_history_log = false; % Keep paper plots causal by default.
        rollback_on_final_trust = true;
        rollback_on_local_est_check = true;
        rollback_on_global_est_check = true;
        rollback_recovery_good_steps = 1; % Clean steps required before a flagged target can leave rollback-active state
        rollback_cooldown_steps = 0; % Minimum steps between actual rollback replays

        % Correction-then-prediction, attack anchors, and output filtering.
        % The physical MATLAB plant remains selected separately through
        % model_vehicle_type ("delay_a" in Config.m).
        dynamics_prediction_mode = "mixed_clean_data";
        force_clean_pose_anchor = false;
        post_rollback_anchor_enabled = true;
        relative_host_anchor_anchor_position_weight = 0.8;
        relative_host_anchor_estimate_position_weight = 0.2;
        relative_host_anchor_clean_theta_weight = 1.0;
        relative_host_anchor_host_theta_weight = 0.0;
        relative_host_anchor_target_velocity_weight = 0.1;
        relative_host_anchor_host_velocity_weight = 0.9;
        relative_host_anchor_target_acceleration_weight = 0.1;
        relative_host_anchor_host_acceleration_weight = 0.9;
        relative_host_anchor_use_bearing = true;
        enable_output_low_pass = true;
        output_low_pass_alpha = 1.0;
        attack_output_low_pass_alpha = 1.0;

        % Direct-channel application/recovery experiment controls.
        direct_recovery_enabled = false;
        direct_recovery_hold_steps = 10;
        direct_recovery_required_good_steps = 8;
        direct_recovery_ramp_steps = 20;
        direct_recovery_min_local_trust = 0.5;
        direct_trust_application_delay_steps = 4;

        % Timestamp/control alignment used by the numerical prediction path.
        timestamp_alignment_enabled = true;
        timestamp_alignment_max_extrapolation_s = 0.25;
        control_timeout_s = 1.0;
        prediction_max_velocity = Inf;
        prediction_max_acceleration = Inf;

        %%%% Attack related

        attacker_update_locally = true; % if the attacker is not updated from the others , only use local data


        %% ------------- Trust related
        using_weight_trust_observer = true; % if using weight trust
        Use_weight_local_trust = true; % if using weight trust for local data
        Use_weight_global_trust = true; % if using weight trust for global data
        Use_python_global_trust = true; % Use Python-compatible global trust calculation in TriPTrustModel
        local_trust_fusion_mode = "weighted_geometric"; % fixed Python component weights

        % Active trust-model values mirrored from config_trust_estimator.yaml.
        % State indices below are MATLAB one-based equivalents of YAML [0..4].
        py_weight_velocity = 1.0;
        py_weight_distance = 2.0;
        py_weight_acceleration = 0.3;
        py_weight_heading = 0.3;
        local_weight_velocity = 0.30;
        local_weight_distance = 0.20;
        local_weight_acceleration = 0.15;
        local_weight_heading = 0.15;
        local_weight_beacon = 0.10;
        local_weight_quality = 0.10;
        stationary_velocity_threshold = 0.2;
        stationary_noise_tolerance = 0.15;
        velocity_tolerance = 0.2;
        min_velocity_tolerance = 0.05;
        turn_velocity_tolerance_gain = 0.3;
        accel_velocity_tolerance_gain = 0.25;
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
        theta_similarity_distance_scale = 1.5;
        theta_similarity_velocity_scale = 1.0;
        theta_similarity_gain = 2.5;
        theta_turn_gain = 2.0;
        theta_contribution_cap = 3.0;
        num_trust_levels = 5;
        dirichlet_C = 0.2;
        dirichlet_wt_local = 0.4;
        dirichlet_wt_global = 0.5;
        ema_alpha = 0.5;
        trust_decay_lambda = 0.2;
        max_message_age_s = 1.0;
        distributed_trust_fallback = 0.2;
        distributed_trust_state_indices = 1:5;
        distributed_trust_contribution_caps = [4.0, 4.0, 2.0, 2.0, 0.2];
        distributed_trust_accel_weight = 0.1;
        distributed_trust_covariance_diag = [2.0, 2.0, 3.5, 2.0, 6.0];
        distributed_local_tau2_diag = [1.5, 0.6];
        distributed_self_turn_distance_gain = 1.5;
        distributed_self_turn_velocity_gain = 1.0;
        use_relative_velocity_in_relative_trust = false;
        use_relative_bearing_in_gamma_self = true;
        gamma_self_bearing_tau2 = 0.25;
        distributed_self_tau2_diag = [];
        gamma_self_penalty_floor = 0.4;
        gamma_self_penalty_exponent = 0.9;
        max_velocity = 5.0;
        max_acceleration = 4.0;
        max_deceleration = -5.0;
        max_jerk = 8.0;
        temporal_pos_tolerance_m = 0.5;
        temporal_vel_tolerance = 0.5;
        
        is_know_data_not_nearby = true ; % just for test purpose, Use that we have better Trust score , meaning that we know the data all of the other vehicles
        
        % New validation controls
        Use_physical_constraints_check = false; % Enable/disable physical constraints validation
        Use_temporal_consistency_check = false; % Enable/disable temporal consistency evaluation 
        

        acceleration_trust_score_method = "vrel_dis_adjusted"; % 'mathematical' - Use exact mathematical formula from paper
                                        % 'enhanced' - Use enhanced implementation with reduced sensitivity
                                        % 'default' - Use default implementation
                                        % vrel_dis_adjusted - Use adjusted relative distance
                                        % 'vrel_dis_real' - Use real relative distance
                                        % 'hybrid' - Combine both methods
        opinion_type = "trust"; % opinion type " distance" , " trust" , " both"

        %%% Controller related
        gamma_type = "min"; % type gamma for switching control = " min" , " max " , " mean "
        controller_type = "local"; % "local" , "coop" , "mix"
        data_type_for_u2 = "true"; % "est" , "true"

        control_use_accel = false; % Will override distributed observer with a prediction model
        CACC_bidirectional = false; % If true, the CACC controller will consider both leading and following vehicles in the control law
    

        %% ---- PREDICTION in observer PARAMETERS
        MAX_PREDICT_ONLY_TIME = 5; % seconds
        N_good = 3; % Number of consecutive good steps to exit predict_only
        blend_thresh = 10; % You can tune this threshold

    end
    methods
        function self = Scenarios_config(dt , simulation_time , scenario_where  )
            self.dt = dt;
            self.simulation_time = simulation_time;
            self.where = scenario_where;

        end

        function [ulim,llim] = getLimitSpeed(self)

            if self.where == "Highway"
                ulim = 33.33;
                llim = 16.67;
            elseif self.where == "Urban"
                ulim = 16.67;
                llim = 12;
            end
        end

        function lane_width = getLaneWidth(self)
            if self.where == "Highway"
                lane_width = 3.6; % Highway lane width in meters
            elseif self.where == "Urban"
                lane_width = 3;
            end
        end

        function set_usecontrol_accel(self, control_use_accel)
            self.control_use_accel = control_use_accel;
        end

        function set_Trip_Dichiret(self, dichiret_type)
            self.Dichiret_type = dichiret_type;
        end

        function set_Lead_Senarios(self , lead_senario )
            self.lead_senario = lead_senario;
        end

        function Is_attacker_not_update(self , attacker_update_locally)
            self.attacker_update_locally = attacker_update_locally;
        end

        function lead_input = get_LeadInput(self , instant_index)
            lead_input = 0;

            if self.lead_senario == "constant"
                return;
            end

            time = self.dt * instant_index;

            % Use a short smooth pulse instead of a long constant command.
            % This keeps the lead vehicle in a realistic highway speed range.
            maneuver_start = 10;
            maneuver_duration = 2;
            maneuver_end = maneuver_start + maneuver_duration;

            if time >= maneuver_start && time < maneuver_end
                phase = (time - maneuver_start) / maneuver_duration;
                smooth_pulse = sin(pi * phase);

                if self.lead_senario == "Acceleration"
                    lead_input = 1.5 * smooth_pulse;
                elseif self.lead_senario == "Deceleration"
                    lead_input = -3.0 * smooth_pulse;
                elseif self.lead_senario == "Lane_change"
                    lead_input = 0;
                end
            end

        end

        function set_CACC_bidirectional(self, CACC_bidirectional)
            self.CACC_bidirectional = CACC_bidirectional;
        end




        function set_Test_better_trust(self , is_know_data_not_nearby)
            self.is_know_data_not_nearby = is_know_data_not_nearby;
        end


        function set_Use_local_data_from_other(self , use_local_data_from_other)
            self.use_local_data_from_other = use_local_data_from_other;
        end

        function set_Use_predict_observer(self, use_predict_observer)
            self.Use_predict_observer = use_predict_observer;
        end

        function  set_predict_controller_type(self ,predict_controller_type )
            self.predict_controller_type = predict_controller_type;
        end

        function  set_Local_observer_type(self ,Local_observer_type )
            self.Local_observer_type = Local_observer_type;
        end
        function set_Is_noise_mesurement(self , Is_noise_mesurement)
            self.Is_noise_mesurement = Is_noise_mesurement;
        end

        function set_Use_smooth_filter(self, Use_smooth_filter)
            self.Use_smooth_filter = Use_smooth_filter;
        end

        function set_monitor_sudden_change(self, monitor_sudden_change)
            self.Monitor_sudden_change = monitor_sudden_change;
        end

        function set_parmeter_prediction_switch_observer(self, MAX_PREDICT_ONLY_TIME, N_good , blend_thresh)
            % Set parameters for prediction in observer
            self.MAX_PREDICT_ONLY_TIME = MAX_PREDICT_ONLY_TIME; % seconds
            self.N_good = N_good; % Number of consecutive good steps to exit predict_only
            self.blend_thresh = blend_thresh; % Blending threshold
        end
    end
end
