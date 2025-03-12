clc
close all
clear 

addpath ../test/core/

addpath('Function');
addpath('core/observer');
addpath('core/Trust');
addpath('core/communication');


% Params = dt , simulation_time , scenario
% "Highway" , "Urban"
Scenarios_config = Scenarios_config( 0.01, 25,  "Highway");


%% Graph 
graph = [0 1 1 0;  % Adjacency matrix
         1 0 1 1;
         1 1 0 1;
         0 1 1 0];


% %% Not use this part
% % Generate virtual graphs for each vehicle
% num_vehicles = size(graph, 1);
% virtual_graphs = cell(num_vehicles, 1);
% weights = cell(num_vehicles, 1);

% for j = 1:num_vehicles
%     virtual_graphs{j} = generate_virtual_graph(graph, j);
%     % fprintf('Virtual graph matrix for vehicle %d:\n', j);
%     % disp(virtual_graphs{j});

%     weights{j} = calculate_weights_Defaut(virtual_graphs{j});
%     % fprintf('Consensus weights matrix for vehicle %d:\n', j);
%     % disp(weights{j});
% end

% weights_Dis_1 = weights{1,1}(1+1,:); 
% weights_Dis_2 = weights{2,1}(2+1,:); 
% weights_Dis_3 = weights{3,1}(3+1,:); 
% weights_Dis_4 = weights{4,1}(4+1,:); 
% %--------- Not use this part

Weight_Trust_module = Weight_Trust_module(graph, 0.5, 1);


%% define driving lanes
lane_width = Scenarios_config.getLaneWidth();
% Create a straight lane with specified width and length
% Params
% num_lanes; % number of the lanes
% lane_width; % width of each single lane
% max_length; % maximum length of the lanes

straightLanes = StraightLane(3, lane_width, 750);



%% define driving scenario
% car1 is in the same lane with ego vehicle , lane lowest

initial_lane_id = 1;
direction_flag = 0; % 1 stands for changing to the left adjacent lane, 0 stands for keeping the current lane, -1 stands for changing

param_sys = ParamVeh();

%% Set Attack senario
attack_module = Attack_module(Scenarios_config.dt);
% Define a time-based attack scenario
% Attack with 'bias' type between 5 and 10 seconds with intensity 0.5
t_star = 10;
t_end = 18;
target_vehicle_id = 1;
case_nb_attack = 3;
attack_module = Atk_Scenarios(attack_module , "Bogus" ,case_nb_attack , t_star, t_end, target_vehicle_id );
% scenario_Attack_params = struct('target_vehicle_id',2, ...
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

center_communication = CenterCommunication(attack_module);


car1 = Vehicle(1, "None", param_sys, [80; 0.5 * lane_width; 0; 23], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

% car2 , car3 is in the middle lane, lane middle 
car2 = Vehicle(2, "look_ahead", param_sys, [60; 0.5 * lane_width; 0; 26], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

car3 = Vehicle(3, "look_ahead", param_sys, [40; 0.5 * lane_width; 0; 26], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

% car4 is in the lane highest
car4 = Vehicle(4, "look_ahead", param_sys, [20; 0.5 * lane_width; 0; 26], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

% car5 = Vehicle(5, "IDM", param_sys, [0; 0.5 * lane_width; 0; 26], controller_car5, 1,  lanes, 0, 0, Scenarios_config);


platton_vehicles = [car1; car2; car3; car4];

car1.assign_neighbor_vehicle(platton_vehicles, [],"None", center_communication , graph);
car2.assign_neighbor_vehicle(platton_vehicles, [],"CACC", center_communication,  graph);
car3.assign_neighbor_vehicle(platton_vehicles, [],"CACC", center_communication, graph );
car4.assign_neighbor_vehicle(platton_vehicles, [],"CACC", center_communication, graph);
% car5.assign_neighbor_vehicle(platton_vehicles);



%% define a simulator and start simulation
IsShowAnimation = true;
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


%% Compare the ground truth and estimated states
car1.plot_ground_truth_vs_estimated()

car2.plot_ground_truth_vs_estimated()
car3.plot_ground_truth_vs_estimated()
car4.plot_ground_truth_vs_estimated()


%% Plot the trust log

car1.plot_trust_log()
car2.plot_trust_log()
car3.plot_trust_log()
car4.plot_trust_log()


%% Plot the trust log for each trip model
car1.trip_models{2}.plot_trust_log(1 , 2)

car2.trip_models{1}.plot_trust_log(2 ,1)
car2.trip_models{3}.plot_trust_log(2,3)


car3.trip_models{2}.plot_trust_log(3,2)
car3.trip_models{4}.plot_trust_log(3,4)

car4.trip_models{3}.plot_trust_log(4,3)


