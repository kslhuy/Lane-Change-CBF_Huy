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
is_lead_input_change = false; % if the lead input is changing (for different senarios)


% Observer related
use_predict_observer = true;
predict_controller_type = "true_other"; % "self" , "true_other" , "predict_other"
Local_observer_type = "kalman"; % "mesurement" , "kalman" , "observer"
set_Is_noise_mesurement = false; % if the measurement is noisy

% controller related
gamma_type = "min"; % type gamma for switching control = " min" , " max " , " mean "
controller_type = "mix"; % type of controller for the ego vehicle: "local" , "coop" , "mix" 
data_type_for_u2 = "est"; % "est" , "true"

% trust related
opinion_type = "both"; % opinion type " distance" , " trust" , " both"

% model related
model_vehicle_type = "normal"; % "delay_v" , "delay_a" , "normal"

%% Graph related
graph = [0 1 1 1;  % Adjacency matrix
        1 0 1 1;
        1 1 0 1;
        1 1 1 0];

trust_threshold = 0.5; % for cut the communication in the graph
kappa = 1; % parameter in the design weigts matrix
Weight_Trust_module = Weight_Trust_module(graph, trust_threshold, kappa);



% Log and Debug related
IsShowAnimation = true;
debug_mode = false;
if (debug_mode )
    % dbstop in Vehicle at 132;
    % dbstop in Observer at 126 if instant_index>=999;
    % dbstop in Observer at 80 if instant_index>=1000;
    dbstop in Observer at 75 if instant_index>=1000;
    % dbstop in Observer at 80 if instant_index>=1000;
end

Scenarios_config = Scenarios_config(dt, simulation_time,  Road_type , controller_type, data_type_for_u2 , gamma_type , opinion_type,model_vehicle_type,debug_mode );
Scenarios_config.set_LeadInput_change(is_lead_input_change); % For different senarios
% Observer related
Scenarios_config.set_predict_controller_type(predict_controller_type);
Scenarios_config.set_Use_predict_observer(use_predict_observer);
Scenarios_config.set_Local_observer_type(Local_observer_type);
Scenarios_config.set_Is_noise_mesurement(set_Is_noise_mesurement); % if the measurement is noisy


% Define driving Senarios lanes
% Create a straight lane with specified width and length
lane_width = Scenarios_config.getLaneWidth();% width of each single lane
 
num_lanes = 3; % number of the lanes
max_length = 750; % maximum length of the lanes
straightLanes = StraightLane(num_lanes, lane_width, max_length);



%% Set Attack senario
attack_module = Attack_module(Scenarios_config.dt);
% Define a time-based attack scenario
% Attack with 'bias' type between 5 and 10 seconds with intensity 0.5


t_star = 10;
t_end = 15;
attacker_vehicle_id = 1;
victim_id = -1;
case_nb_attack = 7;
data_type_attack = "global"; % "local" , "global",
attack_type = "DoS"; % "DoS" , "faulty" , "scaling" , "Collusion" ,"Bogus"
% scenario_Attack_params = struct('attacker_id', attacker_id, ...
%                         'victim_id', victim_id, ...
%                         'start_time', t_star, ...
%                         'end_time', t_end, ...
%                         'attack_type', 'Bogus', ...
%                         'fault_intensity', -0.5, ... % 50% reduction
%                         'data_type',  "local", ... % "local" , "global",
%                         'attack_row', 'velocity');

attack_module = Atk_Scenarios(attack_module , attack_type ,data_type_attack,case_nb_attack , t_star, t_end, attacker_vehicle_id,victim_id );
% scenario_Attack_params = struct('attacker_vehicle_id',2, ...
%                         'start_time', 5, ...      % Start at 5 seconds
%                          'end_time', 10, ...       % End at 10 seconds
%                          'attack_type', 'bias', ... % Bias attack
%                          'fault_intensity', 0.5, ... % Add 0.5 to data
%                          'data_type', 'local', ...     % Local attack
%                          'attack_row' , 'velocity');   % 'velocity' , 'X' , 'Y' , 'all' data

% attack_module.setScenario('time_based', scenario_Attack_params);


% % Scenario 2: Attack global state with faulty data from 15 to 20 seconds
% scenario2 = struct('start_time', 15, ...
%                    'end_time', 20, ...
%                    'attack_type', 'faulty', ...
%                    'fault_intensity', 0.2, ...
%                    'target', 'global');
% attack_module.setScenario('time_based', scenario2);
%% END Attack senario





% car1 is in the same lane with ego vehicle , lane lowest
initial_lane_id = 1;
direction_flag = 0; % 1 stands for changing to the left adjacent lane, 0 stands for keeping the current lane, -1 stands for changing

%% Comunication Module
center_communication = CenterCommunication(attack_module);


car1 = Vehicle(1, "None", param_sys, [80; 0.5 * lane_width; 0; 23], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

% car2 , car3 is in the middle lane, lane middle
car2 = Vehicle(2, "IDM", param_sys, [60; 0.5 * lane_width; 0; 26], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

car3 = Vehicle(3, "IDM", param_sys, [40; 0.5 * lane_width; 0; 26], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

% car4 is in the lane highest
car4 = Vehicle(4, "IDM", param_sys, [20; 0.5 * lane_width; 0; 26], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);



platton_vehicles = [car1; car2; car3; car4];

car1.assign_neighbor_vehicle(platton_vehicles, [],"None", center_communication , graph);
car2.assign_neighbor_vehicle(platton_vehicles, [],"CACC", center_communication,  graph);
car3.assign_neighbor_vehicle(platton_vehicles, [],"CACC", center_communication, graph );
car4.assign_neighbor_vehicle(platton_vehicles, [],"CACC", center_communication, graph);



%% define a simulator and start simulation
simulator0 = Simulator(straightLanes, [] , platton_vehicles, Scenarios_config.dt , IsShowAnimation );
[state_log, input_log] = simulator0.startSimulation(Scenarios_config.simulation_time);
%% Plot the movement of the vehicles
num_vehicles = length(platton_vehicles);
simulator0.plot_movement_log(platton_vehicles, Scenarios_config, num_vehicles);
simulator0.plot_relative_movement_log(platton_vehicles, Scenarios_config, num_vehicles);

%% Plot the global state log
car1.observer.plot_global_state_log()
car2.observer.plot_global_state_log()
car3.observer.plot_global_state_log()
car4.observer.plot_global_state_log()

car1.observer.plot_error_local_estimated()
car2.observer.plot_error_local_estimated()
car3.observer.plot_error_local_estimated()
car4.observer.plot_error_local_estimated()




%% Compare the ground truth and estimated states
% car1.plot_ground_truth_vs_estimated()
% car2.plot_ground_truth_vs_estimated()
% car3.plot_ground_truth_vs_estimated()
% car4.plot_ground_truth_vs_estimated()

car1.plot_ground_error_global_est(platton_vehicles)
car2.plot_ground_error_global_est(platton_vehicles)
car3.plot_ground_error_global_est(platton_vehicles)
car4.plot_ground_error_global_est(platton_vehicles)
%%
car1.plot_u1_u2_gamma()

car2.plot_u1_u2_gamma()
car3.plot_u1_u2_gamma()
car4.plot_u1_u2_gamma()

%% Plot the trust log

car1.plot_trust_log()
car2.plot_trust_log()
car3.plot_trust_log()
car4.plot_trust_log()



%% Plot the trust log for each trip model
car1.trip_models{2}.plot_trust_log(1 , 2)

car2.trip_models{1}.plot_trust_log(2 ,1)
car2.trip_models{3}.plot_trust_log(2,3)
car2.trip_models{3}.plot_trust_log(2,4)



car3.trip_models{2}.plot_trust_log(3,2)
car3.trip_models{4}.plot_trust_log(3,4)

car4.trip_models{3}.plot_trust_log(4,3)


attack_module.plotAttackValues('data_plot' , 'local');
attack_module.plotAttackValues('data_plot' , 'global');



