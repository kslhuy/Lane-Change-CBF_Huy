function test_matlab_weight_rollback_parity()
% Focused checks for shared MATLAB trust weights and rollback replay.

root_dir = fileparts(mfilename('fullpath'));
addpath(root_dir);
addpath(fullfile(root_dir, 'Function'));
addpath(fullfile(root_dir, 'core'));
addpath(fullfile(root_dir, 'core', 'observer'));

test_weight_no_trusted_neighbors();
test_weight_cap_overflow_to_self();
test_weight_trust_proportional_kappa();
test_weight_smoothing_row_stochastic();
test_target_weights_missing_direct_measurement();
test_target_weights_local_bad_caps_neighbors();
test_target_weights_exclude_target_as_source();
test_observer_records_target_weight_log();
test_target_local_check_caps_neighbors();
test_observer_target_weights_gate_bad_direct_channel();
test_rollback_replay_excludes_malicious_sources();
test_rollback_trigger_preserves_host_and_clears_buffer();

fprintf('test_matlab_weight_rollback_parity passed\n');
end

function test_weight_no_trusted_neighbors()
module = make_weight_module(3);
module.enable_smoothing = false;
weights = module.calculate_weights_Trust(1, [1 0.2 0.1], "trust_based");

assert_close(sum(weights), 1.0);
assert_close(weights(1), 0.4);
assert_close(weights(2), 0.6);
assert_close(weights(3), 0.0);
assert_close(weights(4), 0.0);
assert(all(weights >= 0));
end

function test_weight_cap_overflow_to_self()
module = make_weight_module(3);
module.enable_smoothing = false;
weights = module.calculate_weights_Trust(1, [1 0.9 0.1], "trust_based");

assert_close(sum(weights), 1.0);
assert_close(weights(1), 0.4);
assert_close(weights(2), 0.2);
assert_close(weights(3), 0.4);
assert_close(weights(4), 0.0);
end

function test_weight_trust_proportional_kappa()
graph = ones(4) - eye(4);
module = Weight_Trust_module(graph, 0.5, 2);
module.enable_smoothing = false;
weights = module.calculate_weights_Trust(1, [1 0.9 0.6 0.8], "trust_based");

assert_close(sum(weights), 1.0);
assert_close(weights(4), 0.0); % vehicle 3 excluded by kappa after sorting
assert_close(weights(3) / weights(5), 0.9 / 0.8);
assert(all(weights >= 0));
end

function test_weight_smoothing_row_stochastic()
module = make_weight_module(4);
module.enable_smoothing = true;
module.calculate_weights_Trust(1, [1 0.9 0.7 0.2], "trust_based");
weights = module.calculate_weights_Trust(1, [1 0.2 0.9 0.8], "trust_based");

assert_close(sum(weights), 1.0);
assert(all(isfinite(weights)));
assert(all(weights >= 0));
end

function test_target_weights_missing_direct_measurement()
graph = ones(4) - eye(4);
module = Weight_Trust_module(graph, 0.5, 3);
weights = module.calculate_weights_for_target(1, 2, [1 0.9 0.8 0.7], [1 1 1 1], [], []);

assert_close(sum(weights), 1.0);
assert_close(weights(1), 0.0);
assert_close(weights(3), 0.0); % target vehicle is not used as its own neighbor source
assert(weights(2) > 0.0); % host/self residual carries missing direct measurement
assert(weights(4) > 0.0);
assert(weights(5) > 0.0);
assert_close(weights(4) / weights(5), 0.8 / 0.7);
end

function test_target_weights_local_bad_caps_neighbors()
graph = ones(4) - eye(4);
module = Weight_Trust_module(graph, 0.5, 3);
target_model = struct( ...
    'flag_local_est_check', true, ...
    'trust_sample_log', 0.0, ...
    'gamma_local_our_self_log', 1.0);

weights = module.calculate_weights_for_target(1, 2, [1 0.9 0.8 0.7], [1 1 1 1], zeros(5, 1), target_model);

assert_close(sum(weights), 1.0);
assert_close(weights(1), 0.0);
assert_close(weights(3), 0.0);
assert_close(weights(4) + weights(5), 0.01);
assert_close(weights(2), 0.99);
end

function test_target_weights_exclude_target_as_source()
graph = ones(4) - eye(4);
module = Weight_Trust_module(graph, 0.5, 3);
target_model = struct( ...
    'trust_sample_log', 1.0, ...
    'gamma_local_our_self_log', 1.0);

weights = module.calculate_weights_for_target(1, 2, [1 0.99 0.6 0.2], [2 3], zeros(5, 1), target_model);

assert_close(sum(weights), 1.0);
assert_close(weights(3), 0.0); % source vehicle 2 is the target, so exclude it
assert_close(weights(4), 0.4);

module.vehicle_id = 1;
weights_python_style = module.calculate_weights_for_target(2, [1 0.99 0.6 0.2], [2 3], zeros(5, 1), target_model);
assert_close(weights_python_style, weights);
end

function test_observer_records_target_weight_log()
obs = make_mock_observer(3);
weights = [0.4; 0.2; 0.0; 0.4];

obs.record_target_weights(4, 2, weights);

assert_close(obs.target_weights_current(:, 2), weights);
assert_close(obs.target_weights_log(:, 4, 2), weights);
assert(isnan(obs.target_weights_log(1, 4, 1)));
end

function test_target_local_check_caps_neighbors()
obs = make_mock_observer(3);
obs.local_bad_zero_w0_neighbor_total_cap = 0.01;
obs.vehicle.trip_models{2}.flag_local_est_check = true;

weights = obs.get_weights_for_vehicle(2, 1, [0.3 0.2 0.3 0.2], 5);

assert_close(sum(weights), 1.0);
assert_close(weights(1), 0.0);
assert_close(weights(3) + weights(4), 0.01);
assert_close(weights(2), 0.99);
end

function test_observer_target_weights_gate_bad_direct_channel()
obs = make_mock_observer(3);
obs.vehicle.scenarios_config.using_weight_trust_observer = true;
obs.vehicle.weight_module = make_weight_module(3);
obs.vehicle.weight_module.startup_fixed_duration_s = 0;
obs.vehicle.trip_models{2}.flag_local_est_check = true;

weights = obs.get_python_target_weights_if_enabled( ...
    2, 1, [0.4 0.2 0.0 0.4], [1 0.9 0.8], ...
    [true true true], true, zeros(5, 1), 600);

assert_close(sum(weights), 1.0);
assert_close(weights(1), 0.0);
assert_close(weights(3), 0.0);
assert_close(weights(4), 0.01);
assert_close(weights(2), 0.99);
end

function test_rollback_replay_excludes_malicious_sources()
obs = make_mock_observer(3);
step_data = make_step_data(obs, 5);
step_data.target_components{2} = make_component( ...
    2, state_with_x(10), 0.1, ...
    {make_neighbor(3, state_with_x(100), 0.5)}, ...
    [0; 0], false);

corrected = obs.replay_step_without_malicious(step_data, 2, 3, zeros(5, 1));
assert_close(corrected(1), 1.0);

corrected = obs.replay_step_without_malicious(step_data, 2, [2 3], zeros(5, 1));
assert_close(corrected(1), 0.0);
end

function test_rollback_trigger_preserves_host_and_clears_buffer()
obs = make_mock_observer(3);
step_data = make_step_data(obs, 5);
step_data.target_components{2} = make_component( ...
    2, state_with_x(10), 0.1, ...
    {make_neighbor(3, state_with_x(100), 0.5)}, ...
    [0; 0], false);
obs.add_to_rollback_buffer(step_data);

current_states = zeros(5, 3);
current_states(:, 1) = [7; 8; 0.3; 1; 0];
current_states(:, 2) = state_with_x(99);

[applied, corrected] = obs.trigger_contamination_rollback(3, 6, current_states);

assert(applied);
assert_close(corrected(:, 1), current_states(:, 1));
assert_close(corrected(1, 2), 1.0);
assert_close(obs.get_buffer_size(), 0);
assert_close(obs.est_global_state_log(1, 5, 2), 1.0);
end

function module = make_weight_module(n)
graph = ones(n) - eye(n);
module = Weight_Trust_module(graph, 0.5, n);
end

function obs = make_mock_observer(n)
scenario = struct();
scenario.noise_filter_alpha = 0.7;
scenario.local_observer_output_filter_alpha = 0.3;
scenario.measurement_noise_correlation = 0.8;
scenario.rollback_enabled = true;
scenario.rollback_trusted_state_history_size = 15;
scenario.rollback_trusted_state_guard_steps = 0;
scenario.rollback_on_final_trust = true;
scenario.rollback_on_local_est_check = true;
scenario.rollback_on_global_est_check = true;
scenario.rollback_start_time = 0;
scenario.rollback_required_bad_steps = 1;
scenario.rollback_recovery_good_steps = 1;
scenario.rollback_cooldown_steps = 0;
scenario.local_bad_zero_w0_neighbor_total_cap = 0.01;
scenario.using_weight_trust_observer = false;
scenario.Local_observer_type = "kalman";
scenario.Is_noise_mesurement = false;
scenario.no_noise_process_variance = [0.01 0.001 0.005 0.02 0.0005];
scenario.no_noise_measurement_variance = [0.01 0.005 0.0001 0.005 0.0002];
scenario.process_noise_variance = scenario.no_noise_process_variance;
scenario.measurement_noise_variance = scenario.no_noise_measurement_variance;
scenario.model_vehicle_type = "paper";

vehicle = struct();
vehicle.scenarios_config = scenario;
vehicle.total_time_step = 20;
vehicle.vehicle_number = 1;
vehicle.state = zeros(5, 1);
vehicle.other_vehicles = repmat(struct('state', zeros(5, 1)), 1, n);
vehicle.trip_models = cell(1, n);
for idx = 1:n
    vehicle.trip_models{idx} = struct( ...
        'flag_local_est_check', false, ...
        'flag_glob_est_check', false, ...
        'flag_taget_attk', false);
end

param = struct();
param.dt = 0.01;
param.tau = 0.1;
param.max_acceleration = 3.0;
param.min_acceleration = -8.0;

obs = Observer(vehicle, param, zeros(5, n), zeros(5, 1));
end

function step_data = make_step_data(obs, instant_index)
step_data = struct();
step_data.instant_index = instant_index;
step_data.pre_update_states = zeros(5, obs.num_vehicles);
step_data.weights_used = cell(obs.num_vehicles, 1);
step_data.trust_scores = ones(1, obs.num_vehicles);
step_data.target_components = cell(obs.num_vehicles, 1);
end

function component = make_component(direct_source, direct_state, direct_weight, neighbors, control, predict_only)
component = struct();
component.direct = struct( ...
    'source', direct_source, ...
    'state', direct_state, ...
    'weight', direct_weight, ...
    'enabled', true);
component.neighbors = neighbors;
component.prediction = struct( ...
    'control', control, ...
    'predict_only', predict_only, ...
    'use_local', true);
end

function neighbor = make_neighbor(source, state, weight)
neighbor = struct('source', source, 'state', state, 'weight', weight);
end

function state = state_with_x(x)
state = zeros(5, 1);
state(1) = x;
end

function assert_close(actual, expected)
tol = 1e-9;
assert(all(size(actual) == size(expected)), 'Size mismatch');
assert(all(abs(actual(:) - expected(:)) < tol), ...
    'Expected %s, got %s', mat2str(expected), mat2str(actual));
end
