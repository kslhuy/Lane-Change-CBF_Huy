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
% is_lead_input_change = false; % if the lead input is changing (for different senarios)
lead_senario = "constant"; % "constant" , "Acceleration" , "Deceleration" , "Lane_change"

%%%% Observer related
use_predict_observer = true; % Will override distributed observer with a prediction model
predict_controller_type = "true_other"; % "self" , "true_other" , "predict_other"
Local_observer_type = "kalman"; % "mesurement" , "kalman" , "observer"
set_Is_noise_mesurement = false; % if the measurement is noisy
use_local_data_from_other = true; % if the local data from other vehicles is used (true = ourpaper , false = another paper)

attacker_update_locally = true; % if the attacker does update observer by only using the local data   

% ---- PREDICTION in observer PARAMETERS
MAX_PREDICT_ONLY_TIME = 4; % seconds
N_good = 3; % Number of consecutive good steps to exit predict_only
blend_thresh = 3; % You can tune this threshold


%%% Controller related
control_use_accel = true; % if using acceleration control

gamma_type = "min"; % type gamma for switching control = " min" , " max " , " mean " , "self_belief"
controller_type = "local"; % type of controller for the ego vehicle: "local" , "coop" , "mix" 
CACC_bidirectional = false; % If true, the CACC controller will consider both leading and following vehicles in the control law

data_type_for_u2 = "est"; % "est" , "true" using the estimated or true data for u2 (CACC)

%%% Trust related

using_weight_trust = true; % if using weight trust

opinion_type = "mix_non_nearby"; % opinion type " distance" , " trust" , " both" , "mix_non_nearby"
Dichiret_type = "Single" ; % "Single" , "Dual"
monitor_sudden_change = false; % if the sudden change is monitored

is_know_data_not_nearby = true ; % just for test purpose, Use that we have better Trust score , meaning that we know the data all of the other vehicles

%%% Model related
model_vehicle_type = "delay_a"; % "delay_v" , "delay_a" , "normal","paper"

%% Graph related
graph = [0 1 1 1;  % Adjacency matrix
        1 0 1 1;
        1 1 0 1;
        1 1 1 0];

trust_threshold = 0.5; % for cut the communication in the graph
kappa = 1; % parameter in the design weigts matrix
Weight_Trust_module = Weight_Trust_module(graph, trust_threshold, kappa);




Scenarios_config = Scenarios_config(dt, simulation_time,  Road_type , controller_type, data_type_for_u2 , gamma_type , opinion_type,model_vehicle_type );
Scenarios_config.set_Lead_Senarios(lead_senario); % For different senarios

% Observer related
Scenarios_config.set_predict_controller_type(predict_controller_type);
Scenarios_config.set_Use_predict_observer(use_predict_observer);
Scenarios_config.set_Local_observer_type(Local_observer_type);
Scenarios_config.set_Is_noise_mesurement(set_Is_noise_mesurement); % if the measurement is noisy

Scenarios_config.set_parmeter_prediction_switch_observer(MAX_PREDICT_ONLY_TIME, N_good,blend_thresh); % Set parameters for prediction in observer

Scenarios_config.set_Use_local_data_from_other( use_local_data_from_other)
Scenarios_config.Is_attacker_not_update(attacker_update_locally);


Scenarios_config.set_Trip_Dichiret(Dichiret_type); % "Single" , "Dual"
Scenarios_config.set_monitor_sudden_change(monitor_sudden_change); % if the sudden change is monitored

Scenarios_config.set_Test_better_trust(is_know_data_not_nearby);


% Define driving Senarios lanes
% Create a straight lane with specified width and length
lane_width = Scenarios_config.getLaneWidth();% width of each single lane

num_lanes = 3; % number of the lanes
max_length = 750; % maximum length of the lanes
straightLanes = StraightLane(num_lanes, lane_width, max_length);
