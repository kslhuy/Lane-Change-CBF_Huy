Config
attack_module = Atk_Scenarios(attack_module , attack_type ,data_type_attack,case_nb_attack , t_star, t_end, attacker_vehicle_id,victim_id );


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Log and Debug related
IsShowAnimation = false;
debug_mode = false;
if (debug_mode )
    dbstop if error;
    dbstop in Observer at 317 if instant_index>=999;
    % dbstop in TriPTrustModel at 425 if instant_idx>=700;
    % dbstop in TriPTrustModel at 489 if instant_idx>=1101;

    % dbstop in TriPTrustModel at 571 if instant_idx>=1100;

end



%% END Attack senario

% car1 is in the same lane with ego vehicle , lane lowest
initial_lane_id = 1;
direction_flag = 0; % 1 stands for changing to the left adjacent lane, 0 stands for keeping the current lane, -1 stands for changing

%% Comunication Module
center_communication = CenterCommunication(attack_module);

% [80; 0.5 * lane_width; 0; 23 ; 0] = state initialization
car1 = Vehicle(1, "None", param_sys, [80; 0.5 * lane_width; 0; 23 ; 0], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

% car2 , car3 is in the middle lane, lane middle
car2 = Vehicle(2, "IDM", param_sys, [60; 0.5 * lane_width; 0; 26; 0], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

car3 = Vehicle(3, "IDM", param_sys, [40; 0.5 * lane_width; 0; 26; 0], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

% car4 is in the lane highest
car4 = Vehicle(4, "IDM", param_sys, [20; 0.5 * lane_width; 0; 26; 0], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);



platton_vehicles = [car1; car2; car3; car4];

car1.assign_neighbor_vehicle(platton_vehicles, [],"None", center_communication, graph);
car2.assign_neighbor_vehicle(platton_vehicles, [],"CACC", center_communication,  graph);
car3.assign_neighbor_vehicle(platton_vehicles, [],"CACC", center_communication, graph );
car4.assign_neighbor_vehicle(platton_vehicles, [],"CACC", center_communication, graph);



%% define a simulator and start simulation
simulator0 = Simulator(straightLanes, [] , platton_vehicles, Scenarios_config.dt , IsShowAnimation );
[state_log, input_log] = simulator0.startSimulation(Scenarios_config.simulation_time,t_star, t_end, attacker_vehicle_id);

Plot_all

%% Save configuration to CSV file
% save_config_csv();


