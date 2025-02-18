clc
close all
clear 

addpath ../test/core/

addpath('Function');
addpath('core/observer');
addpath('core/Trust');


% Params = dt , simulation_time , scenario
% "Highway" , "Urban"
Scenarios_config = Scenarios_config( 0.01, 10,  "Highway");


%% Graph 
graph = [0 1 1 0;  % Adjacency matrix
         1 0 1 1;
         1 1 0 1;
         0 1 1 0];

% Generate virtual graphs for each vehicle
num_vehicles = size(graph, 1);
virtual_graphs = cell(num_vehicles, 1);
weights = cell(num_vehicles, 1);

for j = 1:num_vehicles
    virtual_graphs{j} = generate_virtual_graph(graph, j);
    % fprintf('Virtual graph matrix for vehicle %d:\n', j);
    % disp(virtual_graphs{j});

    weights{j} = calculate_weights_Defaut(virtual_graphs{j});
    % fprintf('Consensus weights matrix for vehicle %d:\n', j);
    % disp(weights{j});
end

weights_Dis_1 = weights{1,1}(1+1,:); 
weights_Dis_2 = weights{2,1}(2+1,:); 
weights_Dis_3 = weights{3,1}(3+1,:); 
weights_Dis_4 = weights{4,1}(4+1,:); 

%% define driving lanes
if Scenarios_config.where == "Highway"
    lane_width = 3.6;% Highway lane width in meters
elseif Scenarios_config.where == "Urban"
    lane_width = 3;
end
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


car1 = Vehicle(1, "None", param_sys, [80; 0.5 * lane_width; 0; 24], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, weights_Dis_1);

% car2 , car3 is in the middle lane, lane middle 
car2 = Vehicle(2, "IDM", param_sys, [60; 0.5 * lane_width; 0; 26], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, weights_Dis_2);

car3 = Vehicle(3, "IDM", param_sys, [40; 0.5 * lane_width; 0; 26], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, weights_Dis_3);

% car4 is in the lane highest
car4 = Vehicle(4, "IDM", param_sys, [20; 0.5 * lane_width; 0; 26], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, weights_Dis_4);

% car5 = Vehicle(5, "IDM", param_sys, [0; 0.5 * lane_width; 0; 26], controller_car5, 1,  lanes, 0, 0, Scenarios_config);


platton_vehicles = [car1; car2; car3; car4];

car1.assign_neighbor_vehicle(platton_vehicles);
car2.assign_neighbor_vehicle(platton_vehicles);
car3.assign_neighbor_vehicle(platton_vehicles);
car4.assign_neighbor_vehicle(platton_vehicles);
% car5.assign_neighbor_vehicle(platton_vehicles);










%% define a simulator and start simulation
simulator0 = Simulator(straightLanes, [] , platton_vehicles, Scenarios_config.dt);
[state_log, input_log] = simulator0.startSimulation(Scenarios_config.simulation_time);
%% Plot the movement of the vehicles
num_vehicles = length(platton_vehicles);
plot_movement_log(platton_vehicles, Scenarios_config, num_vehicles);

%%
car2.observer.plot_global_state_log()

%% Function for plot

function [] = plot_movement_log(vehicles, scenarios_config, num_vehicles)
    figure(2)
    
    % Define vehicle labels for the legend
    vehicle_labels = arrayfun(@(i) sprintf('Vehicle %d', i), 1:num_vehicles, 'UniformOutput', false);
    
    % Plot velocity history
    subplot(5, 1, 1)
    hold on
    for i = 1:num_vehicles
        plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).state_log(4, :))
    end
    title('Velocity history')
    ylabel('m/s')
    xlabel('s')
    if scenarios_config.where == "Highway"
        ylim([14, 36])
    elseif scenarios_config.where == "Urban"
        ylim([7, 17])
    end
    legend(vehicle_labels)
    hold off

    % Plot steering history
    subplot(5, 1, 2)
    hold on
    for i = 1:num_vehicles
        plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).input_log(2, :))
    end
    title('Steering history')
    ylabel('rad')
    xlabel('s')
    ylim([-0.05, 0.05])
    legend(vehicle_labels)
    hold off

    % Plot yaw history
    subplot(5, 1, 3)
    hold on
    for i = 1:num_vehicles
        plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).state_log(3, :))
    end
    title('Yaw history')
    ylabel('rad')
    xlabel('s')
    ylim([-0.3, 0.3])
    legend(vehicle_labels)
    hold off

    % Plot position X history
    subplot(5, 1, 4)
    hold on
    for i = 1:num_vehicles
        plot(0:scenarios_config.dt :scenarios_config.simulation_time, vehicles(i).state_log(1, :))
    end
    title('Position X history')
    ylabel('m')
    xlabel('s')
    legend(vehicle_labels)
    hold off

    % Plot position Y history
    subplot(5, 1, 5)
    hold on
    for i = 1:num_vehicles
        plot(0:scenarios_config.dt:scenarios_config.simulation_time, vehicles(i).state_log(2, :))
    end
    title('Position Y history')
    ylabel('m')
    xlabel('s')
    legend(vehicle_labels)
    hold off
end