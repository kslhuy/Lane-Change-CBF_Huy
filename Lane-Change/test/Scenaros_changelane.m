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

%% Simulation and Scenarios related
dt = 0.01; % time step
simulation_time = 15; % simulation time
Road_type = "Highway"; % "Highway", "Urban"
lead_senario = "constant"; % "constant", "Acceleration", "Deceleration", "Lane_change"

% Observer related
use_predict_observer = true;
predict_controller_type = "true_other"; % "self", "true_other", "predict_other"
Local_observer_type = "kalman"; % "measurement", "kalman", "observer"
set_Is_noise_measurement = false; % if the measurement is noisy

% Controller related
gamma_type = "min"; % type gamma for switching control = "min", "max", "mean"
controller_type = "mix"; % type of controller for the ego vehicle: "local", "coop", "mix"
data_type_for_u2 = "est"; % "est", "true"

% Trust related
opinion_type = "both"; % opinion type "distance", "trust", "both"
Dichiret_type = "Dual"; % "Single", "Dual"
monitor_sudden_change = false; % if the sudden change is monitored

% Model related
model_vehicle_type = "normal"; % "delay_v", "delay_a", "normal"

%% Graph related
graph = [0 1 1 0;  % Adjacency matrix
         1 0 1 1;
         1 1 0 1;
         0 1 1 0];

trust_threshold = 0.5; % for cutting communication in the graph
kappa = 1; % parameter in the design weights matrix
Weight_Trust_module = Weight_Trust_module(graph, trust_threshold, kappa);

%% Log and Debug related
IsShowAnimation = true;
debug_mode = false;
if debug_mode
    dbstop if error;
end

Scenarios_config = Scenarios_config(dt, simulation_time, Road_type, controller_type, data_type_for_u2, gamma_type, opinion_type, model_vehicle_type, debug_mode);
Scenarios_config.set_Lead_Senarios(lead_senario); % For different scenarios
% Observer related
Scenarios_config.set_predict_controller_type(predict_controller_type);
Scenarios_config.set_Use_predict_observer(use_predict_observer);
Scenarios_config.set_Local_observer_type(Local_observer_type);
% Scenarios_config.set_Is_noise_mesurement(set_Is_noise_mesurement); % if the measurement is noisy
Scenarios_config.set_Trip_Dichiret(Dichiret_type); % "Single", "Dual"
Scenarios_config.set_monitor_sudden_change(monitor_sudden_change); % if the sudden change is monitored

%% Define driving scenarios lanes
lane_width = Scenarios_config.getLaneWidth(); % width of each single lane
num_lanes = 3; % number of the lanes
max_length = 750; % maximum length of the lanes
straightLanes = StraightLane(num_lanes, lane_width, max_length);

%% Set Attack scenario
attack_module = Attack_module(Scenarios_config.dt);
t_star = 5;
t_end = 10;
attacker_vehicle_id = 2;
victim_id = -1;
case_nb_attack = 1;
data_type_attack = "local"; % "local", "global"
attack_type = "none"; % "DoS", "faulty", "scaling", "Collusion", "Bogus"

attack_module = Atk_Scenarios(attack_module, attack_type, data_type_attack, case_nb_attack, t_star, t_end, attacker_vehicle_id, victim_id);

%% Communication Module
center_communication = CenterCommunication(attack_module);

%% Define Vehicles

% %% --- CAR1 :  define a controller for a surrounding vehicle, which changes the lane
% car1_controller_flag = 1; 
% car1_controller_type = "CLF_QP"; % Controller type flag for car1
% car1_direction_flag = 1; % Direction flag for lane change (-1 for left, 1 for right)
% car1_initial_lane_id = 1; % Initial lane number for car1
% car1_target_speed = 30; % Target speed for car1 in m/s
% car1_lim_acc = 0.3 * 9.81; % Maximum acceleration for car1 in m/s^2
% car1_lim_beta = 15 * pi / 180; % Maximum steering angle for car1 in radians
% car1_lim_slip_rate = 12 * pi / 180; % Maximum slip rate for car1 in radians/s

% % Define the goal for the lane change controller
% controller_goal_1 = LaneChangeSurroundingVehicleGoal((car1_initial_lane_id + car1_direction_flag - 0.5) * lane_width, car1_target_speed, car1_lim_beta, car1_lim_acc, car1_lim_slip_rate);




%% --- CAR1 : define a clf_cbf_qp controller
car1_acc_flag = 0;
car1_initial_lane_id = 1; % the initial lane id of car1 vehicle
car1_direction_flag = 1; % the direction of the lane chane process of car1 vehicle
car1_desired_speed = 27.5;
% car1_veh_initial_state = [0; 0.5 * lane_width; 0; car1_desired_speed];
% car1_veh_initial_input = [0; 0];
car1_lim_slip_angle = 15 * pi / 180;
car1_lim_acc = 0.3 * 9.81;
car1_lim_slip_rate = 15 * pi / 180;
car1_controller_flag = 1;
car1_safety_factor = 0.5; % range: 0.1~1, 1 means try to have a maximum safety
controller_goal_1 = EgoControllerGoal(car1_initial_lane_id, car1_direction_flag, car1_desired_speed, straightLanes, car1_lim_slip_angle, car1_lim_acc, car1_lim_slip_rate, car1_safety_factor, Scenarios_config);



car1 = Vehicle(1, "CLF_QP", param_sys, [80; 0.5 * lane_width; 0; 24], car1_initial_lane_id,  straightLanes, car1_direction_flag, 0, Scenarios_config, Weight_Trust_module);

% Car2, Car3, Car4: Look-ahead controllers in the middle lane
initial_lane_id = 1;
direction_flag = 0; % No lane change

% car2 , car3 is in the middle lane, lane middle 
car2 = Vehicle(2, "look_ahead", param_sys, [60; 0.5 * lane_width; 0; 26], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

car3 = Vehicle(3, "look_ahead", param_sys, [40; 0.5 * lane_width; 0; 26], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

% car4 is in the lane highest
car4 = Vehicle(4, "look_ahead", param_sys, [20; 0.5 * lane_width; 0; 26], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

% car5 = Vehicle(5, "IDM", param_sys, [0; 0.5 * lane_width; 0; 26], controller_car5, 1,  lanes, 0, 0, Scenarios_config);


platton_vehicles = [car1; car2; car3; car4];

car1.assign_neighbor_vehicle(platton_vehicles, controller_goal_1,"CLF_QP", center_communication , graph);
car2.assign_neighbor_vehicle(platton_vehicles, [],"look_ahead", center_communication,  graph);
car3.assign_neighbor_vehicle(platton_vehicles, [],"look_ahead", center_communication, graph );
car4.assign_neighbor_vehicle(platton_vehicles, [],"look_ahead", center_communication, graph);


% car5.assign_neighbor_vehicle(platton_vehicles);



%% Define a simulator and start simulation
simulator0 = Simulator(straightLanes, [], platton_vehicles, Scenarios_config.dt, IsShowAnimation);
[state_log, input_log] = simulator0.startSimulation(Scenarios_config.simulation_time);

%% Plot the movement of the vehicles
num_vehicles = length(platton_vehicles);
simulator0.plot_movement_log(platton_vehicles, Scenarios_config, num_vehicles);
simulator0.plot_relative_movement_log(platton_vehicles, Scenarios_config, num_vehicles);

%% Plot the global state log
car1.observer.plot_global_state_log();
car2.observer.plot_global_state_log();
car3.observer.plot_global_state_log();
car4.observer.plot_global_state_log();

%% Plot the local estimation error
car1.observer.plot_error_local_estimated();
car2.observer.plot_error_local_estimated();
car3.observer.plot_error_local_estimated();
car4.observer.plot_error_local_estimated();


% car1.plot_trust_log()
% car2.plot_trust_log()
% car3.plot_trust_log()
% car4.plot_trust_log()


