clc
close all
clear 

dt = 0.01;
simulation_time = 10;
param_sys = ParamVeh();
scenario = 1; % 1 indicates highway, 2 indicates urban road

%% define driving lanes
if scenario == 1
    lane_width = 3.6;% Highway lane width in meters
elseif scenario == 2
    lane_width = 3;
end
% Create a straight lane with specified width and length
lanes = StraightLane(3, lane_width, 750);

%% define a controller for a surrounding vehicle, which changes the lane
car4_controller_flag = 1; 
car4_controller_type = "CLF_QP"; % Controller type flag for car4
car4_direction_flag = -1; % Direction flag for lane change (-1 for left, 1 for right)
car4_initial_lane = 3; % Initial lane number for car4
car4_target_speed = 30; % Target speed for car4 in m/s
car4_lim_acc = 0.3 * 9.81; % Maximum acceleration for car4 in m/s^2
car4_lim_beta = 15 * pi / 180; % Maximum steering angle for car4 in radians
car4_lim_slip_rate = 12 * pi / 180; % Maximum slip rate for car4 in radians/s

% Define the goal for the lane change controller
controller_goal = LaneChangeSurroundingVehicleGoal((car4_initial_lane + car4_direction_flag - 0.5) * lanes.lane_width, car4_target_speed, car4_lim_beta, car4_lim_acc, car4_lim_slip_rate);

% Define parameters for the surrounding vehicle's controller
param_opt_normal = ParamOptSurroundingVeh(dt);

% Create the controller for car4
controller4 = Controller(controller_goal, car4_controller_type, param_opt_normal, param_sys, lanes, []);

%% define driving scenario
% car1 is in the same lane with ego vehicle , lane lowest
car1 = Vehicle(1, 0, param_sys, [70; 0.5 * lane_width; 0; 24], [0; 0], [], 1, dt, lanes, 0, 0, scenario);

% car2 , car3 is in the middle lane, lane middle
car2 = Vehicle(2, 0, param_sys, [70; 1.5 * lane_width; 0; 28], [0; 0], [], 1, dt, lanes, 0, 0, scenario);
car3 = Vehicle(3, 0, param_sys, [-60; 1.5 * lane_width; 0; 25], [0; 0], [], 1, dt, lanes, 0, 0, scenario);

% car4 is in the lane highest
car4 = Vehicle(4, 2, param_sys, [40; 2.5 * lane_width; 0; car4_target_speed], [0; 0], controller4, 3, dt, lanes, car4_direction_flag, 0, scenario);
other_vehicles = [car1; car2; car3; car4];


%% define a following vehicle with 'look_ahead' controller , that following ego vehicle
car7_initial_state = [50; 0.5 * lane_width; 0; 20];
follower_controller_type = "IDM";
param_opt_car7 = ParamOptEgo(dt);
controller_car7 = Controller([], follower_controller_type, param_opt_car7, param_sys, lanes, other_vehicles);
car7 = Vehicle(7, 0, param_sys, car7_initial_state, [0; 0], controller_car7, 1, dt, lanes, 0, 0, scenario);

other_vehicles = [other_vehicles; car7];





%% define a clf_cbf_qp controller for ego vehicle 
ego_acc_flag = 0;
ego_initial_lane_id = 1; % the initial lane id of ego vehicle
ego_direction_flag = 1; % the direction of the lane chane process of ego vehicle
ego_desired_speed = 27.5;

% Inital Position 
ego_veh_initial_state = [0; 0.5 * lane_width; 0; ego_desired_speed];


ego_veh_initial_input = [0; 0];
ego_lim_slip_angle = 15 * pi / 180;
ego_lim_acc = 0.3 * 9.81;
ego_lim_slip_rate = 15 * pi / 180;
ego_controller_type = "CBF_CLF_QP";
ego_safety_factor = 0.5; % range: 0.1~1, 1 means try to have a maximum safety
controller_goal = EgoControllerGoal(ego_initial_lane_id, ego_direction_flag, ego_desired_speed, lanes, ego_lim_slip_angle, ego_lim_acc, ego_lim_slip_rate, ego_safety_factor, scenario);
param_opt = ParamOptEgo(dt);
controller0 = Controller(controller_goal, ego_controller_type, param_opt, param_sys, lanes, other_vehicles);

%% Ego-vehicle
ego_vehicle = Vehicle(5, 1, param_sys, ego_veh_initial_state, ego_veh_initial_input, controller0, ego_initial_lane_id, dt, lanes, ego_direction_flag, ego_acc_flag, scenario);


%% define a following vehicle with 'look_ahead' controller , that following ego vehicle
follower_initial_state = [-10; 0.5 * lane_width; 0; ego_desired_speed];
follower_controller_type = "look_ahead";
controller_follower = Controller([], follower_controller_type, param_opt, param_sys, lanes, [ego_vehicle]);
follower_vehicle = Vehicle(6, 2, param_sys, follower_initial_state, [0; 0], controller_follower, ego_initial_lane_id, dt, lanes, ego_direction_flag, 0, scenario);

other_vehicles = [other_vehicles; follower_vehicle];



%% define a simulator and start simulation
simulator0 = Simulator(lanes, ego_vehicle, other_vehicles, dt);
[state_log, input_log] = simulator0.startSimulation(simulation_time);

%% plot
[yaw_log, velocity_log, steering_log] = get_movement_log(state_log, input_log, param_sys);
plot_movement_log(yaw_log, velocity_log, steering_log, dt, simulation_time, scenario);
figure(3)
plot(state_log(1, :), state_log(2, :))



%% Extract input logs for car1
car1_input_log = other_vehicles(1).input_log;

% Plot the input logs for car1
figure(4)
subplot(2, 1, 1)
plot(0:dt:simulation_time, car1_input_log(1, :))
title('Car1 Input History - Acceleration')
ylabel('Acceleration (m/s^2)')
xlabel('Time (s)')

subplot(2, 1, 2)
plot(0:dt:simulation_time, car1_input_log(2, :))
title('Car1 Input History - Steering Angle')
ylabel('Steering Angle (rad)')
xlabel('Time (s)')



%% Plot the trajectories of all vehicles
figure(3)
hold on
plot(state_log(1, :), state_log(2, :), 'DisplayName', 'Ego Vehicle')
for i = 1:length(other_vehicles)
    plot(other_vehicles(i).state_log(1, :), other_vehicles(i).state_log(2, :), 'DisplayName', ['Vehicle ' num2str(i)])
end
hold off
legend
title('Vehicle Trajectories')
xlabel('X Position (m)')
ylabel('Y Position (m)')



%% Extract state logs for Vehicle 5 (ego vehicle) and Vehicle 6 (follower vehicle)
ego_state_log = state_log; % State log of ego vehicle (Vehicle 5)
follower_state_log = follower_vehicle.state_log; % State log of follower vehicle (Vehicle 6)

%% Calculate state errors
position_error = sqrt((ego_state_log(1, :) - follower_state_log(1, :)).^2 + ...
                 (ego_state_log(2, :) - follower_state_log(2, :)).^2); % Position error (Euclidean distance)
velocity_error = abs(ego_state_log(4, :) - follower_state_log(4, :)); % Velocity error (absolute difference)
yaw_error = abs(ego_state_log(3, :) - follower_state_log(3, :)); % Yaw angle error (absolute difference)

%% Plot state errors
time_vector = 0:dt:simulation_time; % Time vector for plotting

figure(5)
subplot(3, 1, 1)
plot(time_vector, position_error)
title('Position Error Between Ego Vehicle and Follower Vehicle')
ylabel('Position Error (m)')
xlabel('Time (s)')

subplot(3, 1, 2)
plot(time_vector, velocity_error)
title('Velocity Error Between Ego Vehicle and Follower Vehicle')
ylabel('Velocity Error (m/s)')
xlabel('Time (s)')

subplot(3, 1, 3)
plot(time_vector, yaw_error)
title('Yaw Angle Error Between Ego Vehicle and Follower Vehicle')
ylabel('Yaw Error (rad)')
xlabel('Time (s)')



%% Extract state logs for Vehicle 1 (ego vehicle) and Vehicle 7 (follower vehicle)
car1_state_log = car1.state_log; % State log of ego vehicle (Vehicle 5)
car7_state_log = car7.state_log; % State log of follower vehicle (Vehicle 6)

%% Calculate state errors
position_error = sqrt((car1_state_log(1, :) - car7_state_log(1, :)).^2 + ...
                 (car1_state_log(2, :) - car7_state_log(2, :)).^2); % Position error (Euclidean distance)
velocity_error = abs(car1_state_log(4, :) - car7_state_log(4, :)); % Velocity error (absolute difference)
yaw_error = abs(car1_state_log(3, :) - car7_state_log(3, :)); % Yaw angle error (absolute difference)

%% Plot state errors
time_vector = 0:dt:simulation_time; % Time vector for plotting

figure(6)
subplot(3, 1, 1)
plot(time_vector, position_error)
title('Position Error Between car1 and car7')
ylabel('Position Error (m)')
xlabel('Time (s)')

%% Function for plot

function [] = plot_movement_log(yaw_log, velocity_log, steering_log, dt, simulation_time, scenario)
figure(2)
subplot(3, 1, 1)
plot(0:dt:simulation_time, velocity_log)
title('velocity history')
ylabel('m/s')
xlabel('s')
if scenario == 1
    ylim([14, 36])
elseif scenario == 2
    ylim([7, 17])
end
subplot(3, 1, 2)
plot(0:dt:simulation_time, steering_log)
title('steering history')
ylabel('rad')
xlabel('s')
ylim([-0.05, 0.05])
subplot(3, 1, 3)
plot(0:dt:simulation_time, yaw_log)
title('yaw history')
ylabel('rad')
xlabel('s')
ylim([-0.3, 0.3])
end
function [yaw_log, velocity_log, steering_log] = get_movement_log(state_log, input_log, param_sys)
yaw_log = state_log(3, :);
velocity_log = state_log(4, :);
slip_angle_log = input_log(2, :);
steering_log = atan(tan(slip_angle_log).*(param_sys.l_f + param_sys.l_r)./param_sys.l_r);
end