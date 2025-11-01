%% Plot the movement of the vehicles
num_vehicles = length(platton_vehicles);
% simulator0.plot_movement_log(platton_vehicles, Scenarios_config, num_vehicles);
simulator0.plot_movement_log_simple(platton_vehicles, Scenarios_config, num_vehicles );

% simulator0.plot_relative_movement_log(platton_vehicles, Scenarios_config, num_vehicles);
simulator0.plot_relative_movement_log_simple(platton_vehicles, Scenarios_config, num_vehicles);

%% Plot the global state log
car1.observer.plot_global_state_log()
car2.observer.plot_global_state_log()
car3.observer.plot_global_state_log()
car4.observer.plot_global_state_log()

car1.observer.plot_error_local_estimated()
car2.observer.plot_error_local_estimated()
car3.observer.plot_error_local_estimated()
car4.observer.plot_error_local_estimated()


simulator0.plot_error_global_state_log(car1, car2);

%% Compare the ground truth and estimated states
% car1.plot_ground_truth_vs_estimated()
% car2.plot_ground_truth_vs_estimated()
% car3.plot_ground_truth_vs_estimated()
% car4.plot_ground_truth_vs_estimated()

simulator0.plot_ground_error_global_est_ALL(platton_vehicles);

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

% car1.plot_trust_log()
% car2.plot_trust_log()
% car3.plot_trust_log()
% car4.plot_trust_log()

simulator0.plot_all_trust_log(platton_vehicles);



%% Plot the trust log for each trip model
car1.trip_models{2}.plot_trust_log(1 , 2)

car2.trip_models{1}.plot_trust_log(2,1)
car2.trip_models{3}.plot_trust_log(2,3)
car2.trip_models{4}.plot_trust_log(2,4)


car3.trip_models{1}.plot_trust_log(3,1)

car3.trip_models{2}.plot_trust_log(3,2)
car3.trip_models{4}.plot_trust_log(3,4)

car4.trip_models{1}.plot_trust_log(4,1)
car4.trip_models{2}.plot_trust_log(4,2)
car4.trip_models{3}.plot_trust_log(4,3)

%% Details Acc_score

car1.trip_models{2}.plot_details_acc_score(1,2);
car2.trip_models{1}.plot_details_acc_score(2,1);

car1.trip_models{2}.plot_diff_acc_score(1,2);
car2.trip_models{1}.plot_diff_acc_score(2,1);

car2.trip_models{3}.plot_details_acc_score(2,3)
car2.trip_models{3}.plot_diff_acc_score(2,3)

car3.trip_models{2}.plot_details_acc_score(3,2)
car3.trip_models{2}.plot_diff_acc_score(3,2)




car3.trip_models{4}.plot_details_acc_score(3,4)
car4.trip_models{3}.plot_details_acc_score(4,3)

car3.trip_models{4}.plot_diff_acc_score(3,4)
car4.trip_models{3}.plot_diff_acc_score(4,3)




attack_module.plotAttackValues('data_plot' , 'local');
attack_module.plotAttackValues('data_plot' , 'global');
