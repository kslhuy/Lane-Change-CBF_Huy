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

Scenarios_config.rollback_enabled = true; % Enable/disable rollback functionality
Scenarios_config.trust_warmup_time = 0.5; % Avoid trust/rollback decisions during startup-weight warm-up
Scenarios_config.trust_warmup_tolerance_scale = 1.0; % Keep local trust scoring normal during warm-up
Scenarios_config.local_trust_flag_required_samples = 3; % Require persistent low local trust before local-estimate rollback flag
Scenarios_config.local_trust_flag_threshold = 0.5; % Local trust threshold for rollback flagging
Scenarios_config.rollback_start_time = 5.0; % Do not replay until startup fixed weights are finished
Scenarios_config.rollback_required_bad_steps = 3; % Avoid one-step noise spikes triggering rollback
Scenarios_config.rollback_window_size = 100; % 1 s replay horizon at dt=0.01
Scenarios_config.rollback_trusted_state_history_size = 150; % Keep trusted roots for 1.5 s at dt=0.01
Scenarios_config.rollback_rewrite_history_log = false; % Keep estimation-error plots causal for paper figures

% Scenarios_config.rollback_on_global_est_check = true;
% Scenarios_config.rollback_on_local_est_check = false;

Scenarios_config.rollback_recovery_good_steps = 30; % Python-parity: leave active set after one clean step
Scenarios_config.rollback_cooldown_steps = 10; % Python-parity: no replay cooldown




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
Scenarios_config.Dichiret_type = "Single" ; % "Single" , "Dual"
Scenarios_config.Use_weight_local_trust = false; % if using weight trust for local data
Scenarios_config.Use_weight_global_trust = false; % if using weight trust for global data
Scenarios_config.local_trust_fusion_mode = "product"; % "product" multiplies all local components; use "equal_geometric" or "weighted_geometric" to soften

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

trust_threshold = 0.5; % for cut the communication in the graph
kappa = 3; % parameter in the design weigts matrix
Weight_Trust_module = Weight_Trust_module(graph, trust_threshold, kappa);
Weight_Trust_module.w0_fixed = 0.4;
Weight_Trust_module.w_self_base = 0.2;
Weight_Trust_module.w_cap = 0.4;
Weight_Trust_module.enable_smoothing = false;
Weight_Trust_module.startup_fixed_duration_s = 0.0;



% ------------------ Define driving Senarios lanes
% Create a straight lane with specified width and length
lane_width = Scenarios_config.getLaneWidth();% width of each single lane

num_lanes = 3; % number of the lanes
max_length = 750; % maximum length of the lanes
straightLanes = StraightLane(num_lanes, lane_width, max_length);
