function [global_dist_err, global_theta_err, global_vel_err] = mean_simulation_lead(lead_senario)

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
    % lead_senario = "Deceleration"; % "constant" , "Acceleration" , "Deceleration" , "Lane_change"

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
    Dichiret_type = "Dual" ; % "Single" , "Dual"
    monitor_sudden_change = false; % if the sudden change is monitored
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
        dbstop if error;
        % dbstop in Observer at 75 if instant_index>=1000;
    end

    Scenarios_config = Scenarios_config(dt, simulation_time,  Road_type , controller_type, data_type_for_u2 , gamma_type , opinion_type,model_vehicle_type );
    Scenarios_config.set_Lead_Senarios(lead_senario); % For different senarios
    % Observer related
    Scenarios_config.set_predict_controller_type(predict_controller_type);
    Scenarios_config.set_Use_predict_observer(use_predict_observer);
    Scenarios_config.set_Local_observer_type(Local_observer_type);
    Scenarios_config.set_Is_noise_mesurement(set_Is_noise_mesurement); % if the measurement is noisy

    Scenarios_config.set_Trip_Dichiret(Dichiret_type); % "Single" , "Dual"
    Scenarios_config.set_monitor_sudden_change(monitor_sudden_change); % if the sudden change is monitored

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
    attack_type = "none"; % "DoS" , "faulty" , "scaling" , "Collusion" ,"Bogus"


    attack_module = Atk_Scenarios(attack_module , attack_type ,data_type_attack,case_nb_attack , t_star, t_end, attacker_vehicle_id,victim_id );




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

    % After the simulation, calculate errors for each vehicle
    vehicles_to_evaluate = [car2, car3, car4];  % Update based on which vehicles to evaluate

    all_global_dist_errors = [];
    all_global_theta_errors = [];
    all_global_vel_errors = [];

    for i = 1:length(vehicles_to_evaluate)
        v = vehicles_to_evaluate(i);
        [dist_err, theta_err, vel_err] = v.observer.calculate_global_errors();  % Call global error calculation
        all_global_dist_errors = [all_global_dist_errors, dist_err];
        all_global_theta_errors = [all_global_theta_errors, theta_err];
        all_global_vel_errors = [all_global_vel_errors, vel_err];
    end

    % Calculate overall mean errors for all vehicles in this scenario
    global_dist_err = mean(all_global_dist_errors);
    global_theta_err = mean(all_global_theta_errors);
    global_vel_err = mean(all_global_vel_errors);
end

