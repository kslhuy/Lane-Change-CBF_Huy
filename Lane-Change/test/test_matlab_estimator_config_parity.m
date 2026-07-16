function test_matlab_estimator_config_parity()
% Guard the active numerical defaults mirrored from config_trust_estimator.yaml.

root_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(root_dir, 'core'));
addpath(fullfile(root_dir, 'core', 'Trust'));
cfg = Scenarios_config(0.01, 1.0, "Highway");

assert(cfg.fleet_estimator_parity_mode);
assert(cfg.weight_type == "trust_based");
assert_close([cfg.w0_fixed cfg.w_self_base cfg.w_cap], [0.4 0.2 0.4]);
assert(cfg.kappa == 3 && ~cfg.enable_smoothing);
assert_close(cfg.startup_fixed_duration_s, 0.5);
assert(cfg.include_target_self_fleet_estimate);
assert_close(cfg.local_bad_zero_w0_neighbor_total_cap, 0.05);
assert_close([cfg.flag_w0_target_attack_factor, ...
    cfg.flag_w0_global_est_check_factor, cfg.flag_w0_local_est_check_factor], ...
    [0.25 1.25 0.5]);

assert(cfg.dynamics_prediction_mode == "mixed_clean_data");
assert(~cfg.force_clean_pose_anchor && cfg.post_rollback_anchor_enabled);
assert_close([cfg.relative_host_anchor_anchor_position_weight, ...
    cfg.relative_host_anchor_estimate_position_weight], [0.8 0.2]);
assert_close([cfg.relative_host_anchor_clean_theta_weight, ...
    cfg.relative_host_anchor_host_theta_weight], [1.0 0.0]);
assert_close([cfg.relative_host_anchor_target_velocity_weight, ...
    cfg.relative_host_anchor_host_velocity_weight], [0.1 0.9]);
assert_close([cfg.relative_host_anchor_target_acceleration_weight, ...
    cfg.relative_host_anchor_host_acceleration_weight], [0.1 0.9]);
assert(cfg.enable_output_low_pass && cfg.output_low_pass_alpha == 1.0);
assert(~cfg.direct_recovery_enabled && cfg.direct_trust_application_delay_steps == 4);

assert(~cfg.rollback_enabled);
assert(cfg.rollback_window_size == 16);
assert_close(cfg.rollback_startup_suppress_duration_s, 5.0);
assert(cfg.rollback_trigger_delay_steps == 0);
assert(cfg.rollback_trusted_state_guard_steps == 8);
assert(cfg.rollback_trusted_state_history_size == 60);
assert(cfg.timestamp_alignment_enabled);
assert_close(cfg.timestamp_alignment_max_extrapolation_s, 0.25);
assert(cfg.Dichiret_type == "Dual");
assert(cfg.local_trust_fusion_mode == "weighted_geometric");
assert(cfg.use_generalized_trust_vector && cfg.trust_vector_theta_min == 0.4);
assert_close([cfg.trust_decay_lambda cfg.distributed_trust_fallback], [0.2 0.2]);
assert_close(cfg.max_message_age_s, 1.0);
assert(cfg.use_relative_bearing_in_gamma_self);
assert_close([cfg.gamma_self_bearing_tau2 cfg.gamma_self_penalty_floor, ...
    cfg.gamma_self_penalty_exponent], [0.25 0.4 0.9]);

trust = TriPTrustModel(cfg);
assert_close([trust.py_weight_velocity trust.py_weight_distance, ...
    trust.py_weight_acceleration trust.py_weight_heading], [1.0 2.0 0.3 0.3]);
assert_close(trust.distributed_trust_covariance_diag, [2.0 2.0 3.5 2.0 6.0]);
assert_close(trust.distributed_trust_contribution_caps, [4.0 4.0 2.0 2.0 0.2]);
assert_close(trust.distributed_local_tau2_diag, [1.5 0.6]);
assert(~trust.use_relative_velocity_in_relative_trust);
assert_close([trust.wt trust.wt_global trust.C trust.ema_alpha], [0.4 0.5 0.2 0.5]);
assert_close([trust.lambda_h trust.distributed_trust_fallback], [0.2 0.2]);
assert(trust.use_relative_bearing_in_gamma_self);

% Per-scenario overrides must reach each independent model.
cfg.velocity_tolerance = 0.73;
cfg.distributed_trust_fallback = 0.37;
cfg.gamma_self_bearing_tau2 = 0.19;
custom = TriPTrustModel(cfg);
assert_close([custom.velocity_tolerance custom.distributed_trust_fallback, ...
    custom.gamma_self_bearing_tau2], [0.73 0.37 0.19]);

% A missing packet retains 80% for lambda=.2 and only that path gets the
% Python fallback floor.
decay = TriPTrustModel();
assert_close(decay.apply_trust_decay(1, 0.5, true, 'local'), 0.5);
assert_close(decay.apply_trust_decay(1, 0.0, false, 'local'), 0.4);
assert_close(decay.apply_trust_decay(2, 0.0, true, 'global'), 0.0);
assert_close(decay.apply_trust_decay(2, 0.0, false, 'global'), 0.02);

% Bearing residuals wrap across +/-pi, and a clean V2V reference exempts
% target trust from the gamma_self host-contamination penalty.
d_bearing = trust.relative_mahalanobis_python( ...
    [0; pi - 0.01], [0; -pi + 0.01], 0, 0, 0, [1; 0.25], 2, []);
assert_close(d_bearing, (0.02 ^ 2) / 0.25);
assert(~trust.should_apply_gamma_self_penalty("v2v_clean_local_state"));
assert(trust.should_apply_gamma_self_penalty("yolo"));

fprintf('test_matlab_estimator_config_parity passed\n');
end

function assert_close(actual, expected)
tol = 1e-12;
assert(isequal(size(actual), size(expected)), 'Size mismatch');
assert(all(abs(double(actual(:)) - double(expected(:))) < tol), ...
    'Expected %s, got %s', mat2str(expected), mat2str(actual));
end
