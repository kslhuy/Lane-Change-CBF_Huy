%%%%%%%%%
%%%%%%%%%
%%%%%%%%%
%%%%%%%%%
%% Here plot mean for all scenarios with the cyberattack ( Bogus type ).

Config

% Log and Debug related
IsShowAnimation = false;
debug_mode = false;
if (debug_mode )
    dbstop if error;
    % dbstop in Observer at 75 if instant_index>=1000;
end


%% Set Attack senario
attack_module = Attack_module(Scenarios_config.dt);
%% Comunication Module
center_communication = CenterCommunication(attack_module);



t_star = 10;
t_end = 15;
attacker_vehicle_id = 2;
victim_id = -1;
data_type_attack = "local"; % "local" , "global",
attack_type = "ACC"; % "DoS" , "faulty" , "scaling" , "Collusion" ,"Bogus" , "POS" , "VEL" , "ACC"


% ──────────────────────────────────────────────────────────────────────────────
% 3) Excel logging setup
excel_filename = 'Simulation_Results.xlsx';
sheet_name     = 'Summary';
headers = {'AttackType','AttackerVehicle','Case','Vehicle', ...
    'Mean_Distance','Mean_Orientation','Mean_Velocity','Mean_Acceleration'};

% Write headers if file does not exist
if ~isfile(excel_filename)
    writecell(headers, excel_filename, 'Sheet', sheet_name, 'Range', 'A1');
end

% Read existing rows to find next empty row
raw = readcell(excel_filename, 'Sheet', sheet_name);
row_index = size(raw,1) + 1;



% car1 is in the same lane with ego vehicle , lane lowest
initial_lane_id = 1;
direction_flag = 0; % 1 stands for changing to the left adjacent lane, 0 stands for keeping the current lane, -1 stands for changing

% Fixed lead scenario for all attack cases
Scenarios_config.set_Lead_Senarios("constant");




start_attack_senarios_index = 1;
last_attack_senarios_index = 7;

total_num_attack_cases = last_attack_senarios_index - start_attack_senarios_index;

vehicle_labels = {'V1', 'V2', 'V3', 'V4'};
scenario_labels = arrayfun(@(x) sprintf("Case %d", x), start_attack_senarios_index:total_num_attack_cases, 'UniformOutput', false);
metric_labels = {'Distance Error (m)', 'Orientation Error (rad)', 'Velocity Error (m/s)','Acc Error (m/s)'};

mean_errors = zeros(length(vehicle_labels), length(metric_labels),length(scenario_labels)); % 4 vehicles × 4 metrics × nb  scenarios


is_plot_each_case = false;

% Get vehicle IDs
all_vehicle_ids = [1, 2, 3, 4];

% Get IDs excluding attacker
non_attacker_ids = all_vehicle_ids(all_vehicle_ids ~= attacker_vehicle_id);

for case_nb_attack = start_attack_senarios_index:total_num_attack_cases
    fprintf('Running %s attack by V%d, Case %d/%d...\n', ...
        attack_type, attacker_vehicle_id, case_nb_attack, total_num_attack_cases);

    attack_module = Atk_Scenarios(attack_module , attack_type ,data_type_attack,case_nb_attack , t_star, t_end, attacker_vehicle_id,victim_id );


    car1 = Vehicle(1, "None", param_sys, [80; 0.5 * lane_width; 0; 23 ; 0], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

    % car2 , car3 is in the middle lane, lane middle
    car2 = Vehicle(2, "IDM", param_sys, [60; 0.5 * lane_width; 0; 26; 0], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

    car3 = Vehicle(3, "IDM", param_sys, [40; 0.5 * lane_width; 0; 26; 0], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);

    % car4 is in the lane highest
    car4 = Vehicle(4, "IDM", param_sys, [20; 0.5 * lane_width; 0; 26; 0], initial_lane_id,  straightLanes, direction_flag, 0, Scenarios_config, Weight_Trust_module);




    platton_vehicles = [car1; car2; car3; car4];

    car1.assign_neighbor_vehicle(platton_vehicles, [],"None", center_communication, graph);
    car2.assign_neighbor_vehicle(platton_vehicles, [],"CACC", center_communication, graph);
    car3.assign_neighbor_vehicle(platton_vehicles, [],"CACC", center_communication, graph );
    car4.assign_neighbor_vehicle(platton_vehicles, [],"CACC", center_communication, graph);


    %% define a simulator and start simulation
    simulator0 = Simulator(straightLanes, [] , platton_vehicles, Scenarios_config.dt , IsShowAnimation );
    [state_log, input_log] = simulator0.startSimulation(Scenarios_config.simulation_time,t_star, t_end, attacker_vehicle_id);

    % plot
    if (is_plot_each_case)
        car1.plot_ground_error_global_est(platton_vehicles);
        car2.plot_ground_error_global_est(platton_vehicles);
        car3.plot_ground_error_global_est(platton_vehicles);
        car4.plot_ground_error_global_est(platton_vehicles);
    end
    % After the simulation, calculate errors for each vehicle
    % vehicles_to_evaluate = [car2, car3, car4];  % Update based on which vehicles to evaluate



    % Select vehicles to evaluate based on IDs
    vehicles_to_evaluate = platton_vehicles(non_attacker_ids);

    all_global_dist_errors = []; % Initialize arrays to store global errors
    all_global_theta_errors = []; % global orientation errors of vehicles to evaluate
    all_global_vel_errors = [];
    all_global_acc_errors = [];

    for k = 1:length(vehicles_to_evaluate)
        v = vehicles_to_evaluate(k);
        [dist_err, theta_err, vel_err,global_acc_err] = v.observer.calculate_global_errors();  % Call global error calculation
        all_global_dist_errors = [all_global_dist_errors, dist_err];
        all_global_theta_errors = [all_global_theta_errors, theta_err];
        all_global_vel_errors = [all_global_vel_errors, vel_err];
        all_global_acc_errors = [all_global_acc_errors, global_acc_err];
    end

    % n_vehicles_eval = length(vehicles_to_evaluate);
    %
    % n_steps =  size(vehicles_to_evaluate(1).observer.est_global_state_log, 2);  % assuming dist_err is a time series
    % all_global_dist_errors = zeros(n_steps, n_vehicles_eval);
    % all_global_theta_errors = zeros(n_steps, n_vehicles_eval);
    % all_global_vel_errors   = zeros(n_steps, n_vehicles_eval);
    % all_global_acc_errors   = zeros(n_steps, n_vehicles_eval);
    %
    % for k = 1:n_vehicles_eval
    %     v = vehicles_to_evaluate(k);
    %     [dist_err, theta_err, vel_err, acc_err] = v.observer.calculate_global_errors();
    %     all_global_dist_errors(:,k) = dist_err;
    %     all_global_theta_errors(:,k) = theta_err;
    %     all_global_vel_errors(:,k)   = vel_err;
    %     all_global_acc_errors(:,k)   = acc_err;
    % end





    % Calculate overall mean of global errors for all vehicles in this scenario
    mean_dist = mean(all_global_dist_errors,2);
    mean_theta = mean(all_global_theta_errors,2);
    mean_vel = mean(all_global_vel_errors,2);
    mean_acc = mean(all_global_acc_errors,2);

    % write one row per evaluating vehicle
    for vi = 1:length(non_attacker_ids)
        row_data = {
            attack_type, ...
            sprintf('V%d', attacker_vehicle_id), ...
            sprintf('Case %d', case_nb_attack), ...
            sprintf('V%d', non_attacker_ids(vi)), ...
            mean_dist(vi), ...
            mean_theta(vi), ...
            mean_vel(vi), ...
            mean_acc(vi)
        };
        writecell(row_data, excel_filename, 'Sheet', sheet_name, ...
                  'Range', sprintf('A%d', row_index));
        row_index = row_index + 1;
    end



    % Store the errors for plotting
    mean_errors(:, :, case_nb_attack) = [mean_dist, mean_theta, mean_vel,mean_acc];
end

% Plotting results
% Plot each metric
for i = 1:length(metric_labels)  % for each metric
    data = squeeze(mean_errors(:, i, :))';  % Shape: (attack_cases x vehicles)
    figure;
    bar(data);
    xticklabels(scenario_labels);
    ylabel(metric_labels{i});
    legend(vehicle_labels, 'Location', 'northwest');
    title(['Mean ', metric_labels{i}, ' per Attack Scenario']);
    grid on;
end