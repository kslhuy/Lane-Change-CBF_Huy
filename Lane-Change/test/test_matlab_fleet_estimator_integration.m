function test_matlab_fleet_estimator_integration()
% End-to-end numerical smoke test for the Python-parity estimator path.

root_dir = fileparts(mfilename('fullpath'));
addpath(root_dir);
addpath(fullfile(root_dir, 'Function'));
addpath(fullfile(root_dir, 'core'));
addpath(fullfile(root_dir, 'core', 'observer'));
addpath(fullfile(root_dir, 'core', 'Trust'));
addpath(fullfile(root_dir, 'core', 'communication'));
addpath(fullfile(root_dir, 'core', 'Controller'));

dt = 0.01;
scenario = Scenarios_config(dt, 0.05, "Highway");
scenario.fleet_estimator_parity_mode = true;
scenario.Local_observer_type = "measurement";
scenario.Is_noise_mesurement = false;
scenario.Use_smooth_filter_in_local_observer = false;
scenario.Use_predict_observer = false;
scenario.predict_controller_type = "true_other";
scenario.model_vehicle_type = "normal";
scenario.rollback_enabled = false;
scenario.using_weight_trust_observer = true;
scenario.startup_fixed_duration_s = 0.5;
scenario.include_target_self_fleet_estimate = true;
scenario.enable_output_low_pass = true;
scenario.output_low_pass_alpha = 1.0;
scenario.dynamics_prediction_mode = "model";
scenario.trust_warmup_time = 0;
scenario.Dichiret_type = "Dual";
scenario.local_trust_fusion_mode = "weighted_geometric";

n = 2;
graph = ones(n) - eye(n);
attack_module = Attack_module(dt);
center = CenterCommunication(attack_module);
lanes = StraightLane(1, scenario.getLaneWidth(), 100);
param = ParamVeh();

vehicles = Vehicle.empty;
for vehicle_id = 1:n
    module = make_weight_module(graph, scenario, vehicle_id);
    initial_state = [20 - 10 * (vehicle_id - 1); 1.5; 0; 1; 0];
    vehicle = Vehicle(vehicle_id, "None", param, initial_state, 1, ...
        lanes, 0, 0, scenario, module);
    vehicles = [vehicles; vehicle]; %#ok<AGROW>
end
for vehicle_id = 1:n
    vehicles(vehicle_id).assign_neighbor_vehicle( ...
        vehicles, [], "None", center, graph);
end

simulator = Simulator(lanes, [], vehicles, dt, false);
[state_logs, input_logs] = simulator.startSimulation(scenario.simulation_time, 1, 2, 1);

assert(isequal(size(state_logs, 1), 5));
assert(isequal(size(state_logs, 3), n));
assert(isequal(size(input_logs, 1), 2));
for vehicle_id = 1:n
    observer = vehicles(vehicle_id).observer;
    assert(all(isfinite(observer.est_global_state_current(:))));
    assert_close(observer.est_global_state_current(:, vehicle_id), ...
        observer.est_local_state_current);
    target_id = 3 - vehicle_id;
    target_weights = observer.target_weights_current(:, target_id);
    assert_close(sum(target_weights), 1.0);
    assert(target_weights(target_id + 1) > 0, ...
        'Configured target fleet-self source should be active during startup.');
end

fprintf('test_matlab_fleet_estimator_integration passed\n');
end

function module = make_weight_module(graph, scenario, vehicle_id)
module = Weight_Trust_module(graph, scenario.trust_threshold, scenario.kappa);
module.vehicle_id = vehicle_id;
module.weight_type = scenario.weight_type;
module.w0_fixed = scenario.w0_fixed;
module.w_self_base = scenario.w_self_base;
module.w_cap = scenario.w_cap;
module.eta = scenario.eta;
module.enable_smoothing = scenario.enable_smoothing;
module.startup_fixed_duration_s = scenario.startup_fixed_duration_s;
module.use_gamma_self_weight_adaptation = scenario.use_gamma_self_weight_adaptation;
module.gamma_self_weight_floor = scenario.gamma_self_weight_floor;
module.include_target_self_fleet_estimate = scenario.include_target_self_fleet_estimate;
module.local_bad_zero_w0_neighbor_total_cap = scenario.local_bad_zero_w0_neighbor_total_cap;
module.flag_w0_target_attack_factor = scenario.flag_w0_target_attack_factor;
module.flag_w0_global_est_check_factor = scenario.flag_w0_global_est_check_factor;
module.flag_w0_local_est_check_factor = scenario.flag_w0_local_est_check_factor;
end

function assert_close(actual, expected)
tol = 1e-8;
assert(isequal(size(actual), size(expected)), 'Size mismatch');
assert(all(abs(actual(:) - expected(:)) < tol), ...
    'Expected %s, got %s', mat2str(expected), mat2str(actual));
end
