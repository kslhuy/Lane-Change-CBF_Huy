function test_matlab_weight_rollback_parity()
% Focused checks for shared MATLAB trust weights and rollback replay.

root_dir = fileparts(mfilename('fullpath'));
addpath(root_dir);
addpath(fullfile(root_dir, 'Function'));
addpath(fullfile(root_dir, 'core'));
addpath(fullfile(root_dir, 'core', 'observer'));
addpath(fullfile(root_dir, 'core', 'communication'));

test_weight_no_trusted_neighbors();
test_weight_cap_overflow_to_self();
test_weight_trust_proportional_kappa();
test_weight_smoothing_row_stochastic();
test_target_weights_missing_direct_measurement();
test_target_weights_local_bad_caps_neighbors();
test_target_weights_exclude_target_as_source();
test_target_weights_include_target_as_source();
test_equal_target_weights_preserve_source_order();
test_observer_records_target_weight_log();
test_target_local_check_caps_neighbors();
test_observer_target_weights_gate_bad_direct_channel();
test_rollback_replay_excludes_malicious_sources();
test_rollback_trigger_preserves_host_and_clears_buffer();
test_rollback_optional_history_rewrite();
test_circular_heading_residual();
test_target_state_prediction_kernel();
test_direct_trust_delay_exact_steps();
test_direct_recovery_hold_good_ramp();
test_final_trust_requires_local_channel_evidence();
test_relative_host_anchor_prediction();
test_clean_anchor_timestamp_alignment_and_age_guard();
test_relative_anchor_memory_and_trusted_alignment();
test_mixed_clean_without_force_uses_model();

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

function test_target_weights_include_target_as_source()
graph = ones(4) - eye(4);
module = Weight_Trust_module(graph, 0.5, 3);
module.include_target_self_fleet_estimate = true;
target_model = struct( ...
    'local_trust_sample', 1.0, ...
    'gamma_self', 1.0, ...
    'flag_target_attack', false, ...
    'flag_global_est_check', false, ...
    'flag_local_est_check', false);

weights = module.calculate_weights_for_target( ...
    1, 2, [1 0.9 0.8 0.7], [true true true true], zeros(5, 1), target_model);

assert_close(sum(weights), 1.0);
assert(weights(3) > 0.0); % Python YAML explicitly enables target fleet-self source.
end

function test_equal_target_weights_preserve_source_order()
graph = ones(4) - eye(4);
module = Weight_Trust_module(graph, 0.5, 2);
module.weight_type = "equal";
module.include_target_self_fleet_estimate = true;

weights = module.calculate_weights_for_target( ...
    1, 2, [1 0.6 0.99 0.98], [true true true true], [], []);

assert_close(sum(weights), 1.0);
assert_close(weights(3), 0.5); % target/source 2 occurs first
assert_close(weights(4), 0.5); % source 3 occurs second
assert_close(weights(5), 0.0); % source 4 loses the received-order kappa slice

arrival_order_weights = module.calculate_weights_for_target( ...
    1, 2, [1 0.6 0.99 0.98], [4 2 3], [], []);
assert_close(arrival_order_weights(5), 0.5); % source 4 arrived first
assert_close(arrival_order_weights(3), 0.5); % target/source 2 arrived second
assert_close(arrival_order_weights(4), 0.0); % source 3 arrived after kappa
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
% Python rollback returns the corrected current state and owns no historical
% estimate log.  MATLAB preserves that causal behavior by default.
assert_close(obs.est_global_state_log(1, 5, 2), 0.0);
end

function test_rollback_optional_history_rewrite()
obs = make_mock_observer(3);
obs.rollback_rewrite_history_log = true;
step_data = make_step_data(obs, 5);
step_data.target_components{2} = make_component( ...
    2, state_with_x(10), 0.1, ...
    {make_neighbor(3, state_with_x(100), 0.5)}, ...
    [0; 0], false);
obs.add_to_rollback_buffer(step_data);

current_states = zeros(5, 3);
current_states(:, 1) = [7; 8; 0.3; 1; 0];
[applied, ~] = obs.trigger_contamination_rollback(3, 6, current_states);

assert(applied);
assert_close(obs.est_global_state_log(1, 5, 2), 1.0);
end

function test_circular_heading_residual()
obs = make_mock_observer(3);
measurement = zeros(5, 1);
reference = zeros(5, 1);
measurement(3) = -pi + 0.1;
reference(3) = pi - 0.1;
residual = obs.state_residual(measurement, reference);
assert_close(residual(3), 0.2);
end

function test_target_state_prediction_kernel()
obs = make_mock_observer(3);
obs.vehicle.scenarios_config.model_vehicle_type = "normal";
obs.dynamics_prediction_mode = "model";
obs.param_sys.l_f = 1.0;
obs.param_sys.l_r = 1.0;
obs.param_sys.max_steering_angle = 0.5;
state = [1; 2; pi/2; 4; 0.5];
control = [1; 0.1]; % MATLAB ordering: longitudinal, steering
predicted = obs.predict_dynamics_parity( ...
    state, control, 0.01, 2, 10, false, false, []);

expected = [1; 2.04; pi/2 + 4*tan(0.1)/2*0.01; 4.01; 1];
assert_close(predicted, expected);
end

function test_direct_trust_delay_exact_steps()
obs = make_mock_observer(3);
obs.direct_trust_application_delay_steps = 2;
obs.vehicle.trip_models{2}.flag_taget_attk = true;
obs.vehicle.trip_models{2}.flag_local_est_check = true;
trust_scores = [1 0.1 1];

obs.update_direct_trust_delay_states(trust_scores);
assert(obs.is_direct_trust_delay_active(2));
proxy = obs.get_effective_target_trust_model(2, trust_scores);
assert(~proxy.flag_target_attack && ~proxy.flag_local_est_check);
obs.update_direct_trust_delay_states(trust_scores);
assert(obs.is_direct_trust_delay_active(2));
obs.update_direct_trust_delay_states(trust_scores);
assert(~obs.is_direct_trust_delay_active(2)); % close on N+1, exactly as Python
end

function test_direct_recovery_hold_good_ramp()
obs = make_mock_observer(3);
obs.direct_recovery_enabled = true;
obs.direct_recovery_hold_steps = 2;
obs.direct_recovery_required_good_steps = 2;
obs.direct_recovery_ramp_steps = 2;
obs.vehicle.trip_models{2}.flag_taget_attk = true;
obs.vehicle.trip_models{2}.flag_local_est_check = true;
trust_scores = [1 0.1 1];

obs.update_direct_channel_recovery_states(trust_scores);
assert_close(obs.direct_recovery_scale(2), 0.0);
obs.vehicle.trip_models{2}.flag_taget_attk = false;
obs.vehicle.trip_models{2}.flag_local_est_check = false;
obs.vehicle.trip_models{2}.local_trust_sample = 1.0;
obs.update_direct_channel_recovery_states([1 1 1]); % hold 2 -> 1
obs.update_direct_channel_recovery_states([1 1 1]); % hold 1 -> 0
obs.update_direct_channel_recovery_states([1 1 1]); % good 1
assert_close(obs.direct_recovery_scale(2), 0.0);
obs.update_direct_channel_recovery_states([1 1 1]); % good 2, ramp 1
assert_close(obs.direct_recovery_scale(2), 0.5);
obs.update_direct_channel_recovery_states([1 1 1]); % ramp 2
assert_close(obs.direct_recovery_scale(2), 1.0);
end

function test_final_trust_requires_local_channel_evidence()
obs = make_mock_observer(3);
obs.vehicle.trip_models{2}.local_trust_sample = 0.9;
obs.vehicle.trip_models{2}.flag_taget_attk = false;
obs.vehicle.trip_models{2}.flag_local_est_check = false;
obs.vehicle.trip_models{2}.flag_glob_est_check = false;
[applied, ~] = obs.check_and_trigger_rollback(10, [1 0.1 1], zeros(5, 3));
assert(~applied && ~ismember(2, obs.malicious_vehicles));

obs.vehicle.trip_models{2}.flag_glob_est_check = true;
[applied, ~] = obs.check_and_trigger_rollback(11, [1 0.9 1], zeros(5, 3));
assert(~applied); % no buffer, but the source is quarantined as global-only
assert(ismember(2, obs.malicious_vehicles));
assert(~obs.is_target_quarantined_by_rollback(2)); % direct channel remains usable
end

function test_relative_host_anchor_prediction()
obs = make_mock_observer(3);
obs.vehicle.scenarios_config.model_vehicle_type = "normal";
obs.dynamics_prediction_mode = "relative_host_anchor_mixed";
obs.relative_host_anchor_anchor_position_weight = 0.8;
obs.relative_host_anchor_estimate_position_weight = 0.2;
obs.relative_host_anchor_clean_theta_weight = 1.0;
obs.relative_host_anchor_host_theta_weight = 0.0;
obs.relative_host_anchor_target_velocity_weight = 0.1;
obs.relative_host_anchor_host_velocity_weight = 0.9;
obs.relative_host_anchor_target_acceleration_weight = 0.1;
obs.relative_host_anchor_host_acceleration_weight = 0.9;
obs.relative_host_anchor_use_bearing = true;
obs.param_sys.l_f = 1;
obs.param_sys.l_r = 1;

host = [0; 0; 0; 10; 1];
clean = [20; 5; pi/4; 8; 0];
obs.est_local_state_current = host;
attack = Attack_module(0.01);
center = CenterCommunication(attack);
center.local_state_storage(2) = clean;
center.local_state_atk_storage(2) = clean;
center.local_state_timestamp_storage(2) = 10;
obs.vehicle.center_communication = center;

state = [18; 4; 0; 7; 0];
snapshot = obs.build_relative_host_anchor_snapshot(2, 10, state, clean);
predicted = obs.predict_dynamics_parity( ...
    state, [], 0.1, 2, 10, false, true, snapshot);

velocity = 0.1 * state(4) + 0.9 * host(4);
anchor_xy = clean(1:2) + velocity * [cos(clean(3)); sin(clean(3))] * 0.1;
model_xy = state(1:2) + state(4) * [cos(state(3)); sin(state(3))] * 0.1;
expected_xy = 0.8 * anchor_xy + 0.2 * model_xy;
assert_close(predicted(1:2), expected_xy);
assert_close(predicted(3), clean(3));
assert_close(predicted(4), velocity);
assert_close(predicted(5), 0.9);
end

function test_clean_anchor_timestamp_alignment_and_age_guard()
obs = make_mock_observer(3);
obs.param_sys.dt = 0.1;
obs.timestamp_alignment_enabled = true;
obs.timestamp_alignment_max_extrapolation_s = 0.25;
obs.relative_host_anchor_max_message_age_s = 1.0;
obs.est_local_state_current = [0; 0; 0; 1; 0];

attack = Attack_module(0.1);
center = CenterCommunication(attack);
clean = [10; 0; 0; 3; 0];
center.local_state_storage(2) = clean;
center.local_state_atk_storage(2) = clean;
center.local_state_timestamp_storage(2) = 8;
obs.vehicle.center_communication = center;

[aligned, source_step] = obs.get_clean_aligned_state(2, 10);
assert_close(source_step, 8);
assert_close(aligned(1:2), [10.6; 0]);
assert(obs.relative_host_measurement_memory_valid(2));
assert_close(obs.relative_host_measurement_memory(:, 2), [10; 2; 0; 8]);

% Python drops clean packets older than max_message_age_s instead of using
% their unpropagated pose indefinitely. The independent memory remains.
center.local_state_timestamp_storage(2) = -1;
[stale, ~] = obs.get_clean_aligned_state(2, 10);
assert(isempty(stale));
assert_close(obs.relative_host_measurement_memory(:, 2), [10; 2; 0; 8]);
end

function test_relative_anchor_memory_and_trusted_alignment()
obs = make_mock_observer(3);
obs.param_sys.dt = 0.1;
obs.relative_host_anchor_max_message_age_s = 1.0;
obs.relative_host_anchor_use_bearing = true;
obs.est_local_state_current = [0; 0; 0; 1; 0];
obs.cache_relative_host_measurement( ...
    2, [10; 0; 0; 3; 0], 10, 10);

% Recent range memory advances by radial relative velocity and retains its
% original source step, matching recent_relative_measurements.
reference = [12; 0; 0; 3; 0];
snapshot = obs.build_relative_host_anchor_snapshot( ...
    2, 12, reference, [], false);
assert_close(snapshot(6), 10.4);
assert_close(snapshot(8), 2.0);
assert_close(snapshot(10:11), [10.4; 0]);
assert_close(snapshot(12), 10);

% Beyond max_message_age_s the remembered distance is still usable, but is
% no longer extrapolated. Current reference geometry owns ahead/behind sign.
old_snapshot = obs.build_relative_host_anchor_snapshot( ...
    2, 22, [-5; 0; 0; 0; 0], [], false);
assert_close(old_snapshot(6), 10.0);
assert_close(old_snapshot(10:11), [10; 0]);
assert_close(old_snapshot(7), -1);

% Trusted anchor history is selected ahead of live memory, time-aligned once
% from its source step, and refreshed with the current host pose.
obs.est_local_state_current = [5; 3; 0; 9; 0.5];
obs.rollback_trusted_relative_anchor_history(:, 1, 2) = ...
    [100; 100; 1; 2; 3; 10; 1; 2; pi / 2; 0; 10; 5];
obs.rollback_trusted_relative_anchor_count(2) = 1;
trusted = obs.build_relative_host_anchor_snapshot( ...
    2, 8, [20; 3; 0; 0; 0], [], true);
assert_close(trusted(1:5), obs.est_local_state_current);
assert_close(trusted(6), 10.6);
assert_close(trusted(10:11), [0; 10.6]);

expired = obs.build_relative_host_anchor_snapshot( ...
    2, 20, [20; 3; 0; 0; 0], [], true);
assert_close(expired(6), 10.0);
assert_close(expired(10:11), [0; 10]);
end

function test_mixed_clean_without_force_uses_model()
obs = make_mock_observer(3);
obs.vehicle.scenarios_config.model_vehicle_type = "normal";
obs.dynamics_prediction_mode = "mixed_clean_data";
obs.param_sys.l_f = 1;
obs.param_sys.l_r = 1;
state = [1; 2; 0.3; 4; 0];
control = [0.5; 0.1];
actual = obs.predict_dynamics_parity( ...
    state, control, 0.01, 2, 10, false, false, []);
expected = obs.predict_with_vehicle_model_parity(state, control, 0.01);
assert_close(actual, expected);
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
