clc
close all
clear

addpath ../test/core/

addpath('Function');
addpath('core/observer');
addpath('core/Controller');
addpath('core/Trust');
addpath('core/communication');


%% Params affect the simulation
param_sys = ParamVeh();

%% Simulation and Senarios related  
dt = 0.01; % time step
simulation_time = 25; % simulation time
Road_type = "Highway"; % "Highway" , "Urban"
Scenarios_config = Scenarios_config(dt, simulation_time,  Road_type  );


% is_lead_input_change = false; % if the lead input is changing (for different senarios)
Scenarios_config.lead_senario = "constant"; % "constant" , "Acceleration" , "Deceleration" , "Lane_change"

%%%% Observer related
Scenarios_config.Use_predict_observer = false; % Will override distributed observer with a prediction model

Scenarios_config.predict_controller_type = "true_other"; % "self" , "true_other" , "predict_other"

Scenarios_config.Local_observer_type = "kalman"; % "mesurement" , "kalman" , "observer"

Scenarios_config.Is_noise_mesurement = true; % if the measurement is noisy
Scenarios_config.noise_probability = 0.1; % Probability of adding measurement noise
Scenarios_config.Use_smooth_filter = true; % Enable/disable smooth filtering for noise and observer output
Scenarios_config.Use_smooth_filter_in_local_observer = false; % Enable/disable smooth filtering in local observer
% Noise vectors use state order [x, y, theta, v, a]. Values are variances; std = sqrt(variance).
Scenarios_config.measurement_noise_variance = [0.15, 0.005, 0.003, 0.01, 0.0003]; % R when measurement noise is enabled
Scenarios_config.process_noise_variance = [0.01, 0.001, 0.0005, 0.02, 0.0005]; % Q when measurement noise is enabled
Scenarios_config.no_noise_measurement_variance = [0.01, 0.005, 0.0001, 0.005, 0.0002]; % R used by Kalman filter when measurement noise is disabled
Scenarios_config.no_noise_process_variance = [0.01, 0.001, 0.005, 0.02, 0.0005]; % Q used by Kalman filter when measurement noise is disabled
Scenarios_config.measurement_noise_correlation = 0.8; % 0 = white noise, closer to 1 = smoother correlated noise
Scenarios_config.noise_filter_alpha = 0.7; % 0 = no smoothing, closer to 1 = smoother injected noise
Scenarios_config.local_observer_output_filter_alpha = 0.3; % 0 = raw local observer output, closer to 1 = smoother output

Scenarios_config.use_local_data_from_other = true; % if the local data from other vehicles is used (true = ourpaper , false = another paper)
% - PREDICTION in observer PARAMETERS
Scenarios_config.MAX_PREDICT_ONLY_TIME = 3; % seconds
Scenarios_config.N_good = 3; % Number of consecutive good steps to exit predict_only
Scenarios_config.blend_thresh = 3; % You can tune this threshold

% Canonical numerical parity mode for the Python TrustBasedFleetEstimator.
Scenarios_config.fleet_estimator_parity_mode = true; % true = Python parity, false = MATLAB canonical
Scenarios_config.dynamics_prediction_mode = "mixed_clean_data";
Scenarios_config.force_clean_pose_anchor = false;
Scenarios_config.post_rollback_anchor_enabled = true;
Scenarios_config.relative_host_anchor_anchor_position_weight = 0.8;
Scenarios_config.relative_host_anchor_estimate_position_weight = 0.2;
Scenarios_config.relative_host_anchor_clean_theta_weight = 1.0;
Scenarios_config.relative_host_anchor_host_theta_weight = 0.0;
Scenarios_config.relative_host_anchor_target_velocity_weight = 0.1;
Scenarios_config.relative_host_anchor_host_velocity_weight = 0.9;
Scenarios_config.relative_host_anchor_target_acceleration_weight = 0.1;
Scenarios_config.relative_host_anchor_host_acceleration_weight = 0.9;
Scenarios_config.relative_host_anchor_use_bearing = true;
Scenarios_config.enable_output_low_pass = true;
Scenarios_config.output_low_pass_alpha = 1.0;
Scenarios_config.attack_output_low_pass_alpha = 1.0;

Scenarios_config.direct_recovery_enabled = true;
Scenarios_config.direct_recovery_hold_steps = 10;
Scenarios_config.direct_recovery_required_good_steps = 8;
Scenarios_config.direct_recovery_ramp_steps = 20;
Scenarios_config.direct_recovery_min_local_trust = 0.5;
Scenarios_config.direct_trust_application_delay_steps = 4;

Scenarios_config.timestamp_alignment_enabled = true;
Scenarios_config.timestamp_alignment_max_extrapolation_s = 0.25;
Scenarios_config.control_timeout_s = 1.0;

Scenarios_config.rollback_enabled = true; % Current Python YAML default for batch analysis
Scenarios_config.trust_warmup_time = 0.0; % Trust still logs during fixed-weight startup
Scenarios_config.trust_warmup_tolerance_scale = 1.0; % Keep local trust scoring normal during warm-up
Scenarios_config.local_trust_flag_required_samples = 1;
Scenarios_config.local_trust_flag_threshold = 0.5; % Local trust threshold for rollback flagging
Scenarios_config.rollback_startup_suppress_duration_s = 5.0;
Scenarios_config.rollback_start_time = 5.0; % MATLAB execution gate for startup suppression
Scenarios_config.rollback_trigger_delay_steps = 0;
Scenarios_config.rollback_required_bad_steps = 1;
Scenarios_config.rollback_window_size = 16;
Scenarios_config.rollback_trusted_state_guard_steps = 8;
Scenarios_config.rollback_trusted_state_history_size = 60;
Scenarios_config.rollback_rewrite_history_log = false; % Keep estimation-error plots causal for paper figures
Scenarios_config.rollback_on_final_trust = true;
Scenarios_config.rollback_on_local_est_check = true;
Scenarios_config.rollback_on_global_est_check = true;
Scenarios_config.rollback_recovery_good_steps = 1;
Scenarios_config.rollback_cooldown_steps = 0;




%%%% Attack related

attack_module = Attack_module(Scenarios_config.dt);
% Define a time-based attack scenario
% Attack from t_star to t_end
% a vehicle with attacker_vehicle_id will be the attacker

t_star = 10;
t_end = 15;
attacker_vehicle_id = 1;
victim_id = -1;                 % -1 mean every vehicle
case_nb_attack = 5;             % case number of attack scenario
data_type_attack = "global"; % "local" , "global", "none"
attack_type = "Mix_test"; % "DoS"  , "Collusion" ,"Bogus", "None" , "POS" , "VEL" , "ACC"


Scenarios_config.attacker_update_locally = true; % if the attacker does update observer by only using the local data   



%%% Controller related
Scenarios_config.control_use_accel = true; % if using acceleration control

Scenarios_config.gamma_type = "min"; % type gamma for switching control = " min" , " max " , " mean " , "self_belief"
Scenarios_config.controller_type = "coop"; % type of controller for the ego vehicle: "local" , "coop" , "mix"
Scenarios_config.CACC_bidirectional = false; % If true, the CACC controller will consider both leading and following vehicles in the control law

Scenarios_config.data_type_for_u2 = "true"; % "est" , "true" using the estimated or true data for u2 (CACC)

%%% Trust related

Scenarios_config.using_weight_trust_observer = true; % if using weight trust for observer

Scenarios_config.opinion_type = "mix_non_nearby"; % opinion type " distance" , " trust" , " both" , "mix_non_nearby"
Scenarios_config.Dichiret_type = "Dual" ; % Python parity: separate local/global Dirichlet vectors
Scenarios_config.Use_weight_local_trust = false; % if using weight trust for local data
Scenarios_config.Use_weight_global_trust = false; % if using weight trust for global data
Scenarios_config.local_trust_fusion_mode = "weighted_geometric"; % Python fixed component-weight fusion
Scenarios_config.py_weight_velocity = 1.0;
Scenarios_config.py_weight_distance = 2.0;
Scenarios_config.py_weight_acceleration = 0.3;
Scenarios_config.py_weight_heading = 0.3;
Scenarios_config.velocity_tolerance = 0.2;
Scenarios_config.turn_velocity_tolerance_gain = 0.3;
Scenarios_config.accel_velocity_tolerance_gain = 0.25;
Scenarios_config.acceleration_base_tolerance = 1.0;
Scenarios_config.acceleration_speed_tolerance_gain = 0.15;
Scenarios_config.acceleration_rel_velocity_tolerance = 0.2;
Scenarios_config.dirichlet_C = 0.2;
Scenarios_config.dirichlet_wt_local = 0.4;
Scenarios_config.dirichlet_wt_global = 0.5;
Scenarios_config.ema_alpha = 0.5;
Scenarios_config.trust_decay_lambda = 0.2;
Scenarios_config.max_message_age_s = 1.0;
Scenarios_config.distributed_trust_fallback = 0.2;
Scenarios_config.distributed_trust_state_indices = 1:5; % YAML [0,1,2,3,4]
Scenarios_config.distributed_trust_contribution_caps = [4.0 4.0 2.0 2.0 0.2];
Scenarios_config.distributed_trust_accel_weight = 0.1;
Scenarios_config.distributed_trust_covariance_diag = [2.0 2.0 3.5 2.0 6.0];
Scenarios_config.distributed_local_tau2_diag = [1.5 0.6];
Scenarios_config.distributed_self_turn_distance_gain = 1.5;
Scenarios_config.distributed_self_turn_velocity_gain = 1.0;
Scenarios_config.use_relative_velocity_in_relative_trust = false;
Scenarios_config.use_relative_bearing_in_gamma_self = true;
Scenarios_config.gamma_self_bearing_tau2 = 0.25;
Scenarios_config.distributed_self_tau2_diag = [];
Scenarios_config.gamma_self_penalty_floor = 0.4;
Scenarios_config.gamma_self_penalty_exponent = 0.9;

Scenarios_config.acceleration_trust_score_method = "vrel_dis_adjusted"; % 'mathematical' - Use exact mathematical formula from paper
                                                                        % 'enhanced' - Use enhanced implementation with reduced sensitivity
                                                                        % 'default' - Use default implementation
                                                                        % 'vrel_dis_adjusted' - Use adjusted relative distance
                                                                        % 'vrel_dis_real' - Use real relative distance
                                                                        % 'hybrid' - Combine both methods

% -- option trust (no imporant yet)
Scenarios_config.Monitor_sudden_change = false; % if the sudden change is monitored

Scenarios_config.is_know_data_not_nearby = false ; % false avoids oracle non-nearby distance data in trust evaluation

% New validation controls
Scenarios_config.Use_physical_constraints_check = false; % Disable simple physical gate during startup observer transients
Scenarios_config.Use_temporal_consistency_check = false; % Enable/disable temporal consistency evaluation (recommended: true)

%%% Model related
Scenarios_config.model_vehicle_type = "delay_a"; % "delay_v" , "delay_a" , "normal","paper"

%% ------------------  Graph related
graph = [0 1 1 1;  % Adjacency matrix
        1 0 1 1;
        1 1 0 1;
        1 1 1 0];

% One configured template is created here. Batch scripts clone its public
% settings into a fresh module for every host/case; the handle is never shared.
Scenarios_config.trust_threshold = 0.5;
Scenarios_config.weight_type = "trust_based";
Scenarios_config.w0_fixed = 0.4;
Scenarios_config.w_self_base = 0.2;
Scenarios_config.w_cap = 0.4;
Scenarios_config.kappa = 3;
Scenarios_config.eta = 0.15;
Scenarios_config.enable_smoothing = false;
Scenarios_config.startup_fixed_duration_s = 0.5;
Scenarios_config.use_gamma_self_weight_adaptation = true;
Scenarios_config.gamma_self_weight_floor = 0.25;
Scenarios_config.include_target_self_fleet_estimate = true;
Scenarios_config.local_bad_zero_w0_neighbor_total_cap = 0.05;
Scenarios_config.flag_w0_target_attack_factor = 0.25;
Scenarios_config.flag_w0_global_est_check_factor = 1.25;
Scenarios_config.flag_w0_local_est_check_factor = 0.5;
Scenarios_config.use_generalized_trust_vector = true;
Scenarios_config.trust_vector_theta_min = 0.4;

trust_threshold = Scenarios_config.trust_threshold; % communication/trust cutoff
kappa = Scenarios_config.kappa; % maximum trusted fleet sources
Weight_Trust_module = Weight_Trust_module(graph, trust_threshold, kappa);
Weight_Trust_module.weight_type = Scenarios_config.weight_type;
Weight_Trust_module.w0_fixed = Scenarios_config.w0_fixed;
Weight_Trust_module.w_self_base = Scenarios_config.w_self_base;
Weight_Trust_module.w_cap = Scenarios_config.w_cap;
Weight_Trust_module.eta = Scenarios_config.eta;
Weight_Trust_module.enable_smoothing = Scenarios_config.enable_smoothing;
Weight_Trust_module.startup_fixed_duration_s = Scenarios_config.startup_fixed_duration_s;
Weight_Trust_module.use_gamma_self_weight_adaptation = Scenarios_config.use_gamma_self_weight_adaptation;
Weight_Trust_module.gamma_self_weight_floor = Scenarios_config.gamma_self_weight_floor;
Weight_Trust_module.include_target_self_fleet_estimate = Scenarios_config.include_target_self_fleet_estimate;
Weight_Trust_module.local_bad_zero_w0_neighbor_total_cap = Scenarios_config.local_bad_zero_w0_neighbor_total_cap;
Weight_Trust_module.flag_w0_target_attack_factor = Scenarios_config.flag_w0_target_attack_factor;
Weight_Trust_module.flag_w0_global_est_check_factor = Scenarios_config.flag_w0_global_est_check_factor;
Weight_Trust_module.flag_w0_local_est_check_factor = Scenarios_config.flag_w0_local_est_check_factor;



% ------------------ Define driving Senarios lanes
% Create a straight lane with specified width and length
lane_width = Scenarios_config.getLaneWidth();% width of each single lane

num_lanes = 3; % number of the lanes
max_length = 750; % maximum length of the lanes
straightLanes = StraightLane(num_lanes, lane_width, max_length);
