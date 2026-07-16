function test_python_matlab_weight_cross_language()
% Compare MATLAB target weights against vectors emitted by the Python source.

root_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(root_dir, 'Function'));
generator = fullfile(root_dir, 'python', 'generate_weight_parity_fixture.py');
command = sprintf('python "%s"', generator);
[status, output] = system(command);
assert(status == 0, 'Python fixture generator failed: %s', output);
golden = jsondecode(strtrim(output));

graph = ones(4) - eye(4);
module = configure_module(Weight_Trust_module(graph, 0.5, 3));
trust_scores = [1.0, 0.9, 0.8, 0.7];
available = [true true true true];
direct = zeros(5, 1);

normal_model = trust_model(0.8, 0.6, false, false, false);
normal = module.calculate_weights_for_target( ...
    1, 2, trust_scores, available, direct, normal_model);
assert_close(normal, golden.normal);

local_bad_model = trust_model(0.1, 0.4, false, true, false);
local_bad = module.calculate_weights_for_target( ...
    1, 2, trust_scores, available, [], local_bad_model);
assert_close(local_bad, golden.local_bad);

global_bad_model = trust_model(1.0, 1.0, false, false, true);
global_bad = module.calculate_weights_for_target( ...
    1, 2, trust_scores, available, direct, global_bad_model);
assert_close(global_bad, golden.global_bad);

startup = module.calculate_startup_weights_for_target( ...
    1, 2, available, direct);
assert_close(startup, golden.startup);

paper_module = configure_module(Weight_Trust_module(graph, 0.5, 2));
paper_module.weight_type = "paper";
paper = paper_module.calculate_paper_weights_for_target( ...
    1, 2, trust_scores, available, direct, trust_model(0.9, 1, false, false, false));
assert_close(paper, golden.paper);

fprintf('test_python_matlab_weight_cross_language passed\n');
end

function module = configure_module(module)
module.vehicle_id = 1;
module.weight_type = "trust_based";
module.w0_fixed = 0.4;
module.w_self_base = 0.2;
module.w_cap = 0.4;
module.eta = 0.15;
module.enable_smoothing = false;
module.startup_fixed_duration_s = 0.5;
module.use_gamma_self_weight_adaptation = true;
module.gamma_self_weight_floor = 0.25;
module.include_target_self_fleet_estimate = true;
module.local_bad_zero_w0_neighbor_total_cap = 0.05;
module.flag_w0_target_attack_factor = 0.25;
module.flag_w0_global_est_check_factor = 1.25;
module.flag_w0_local_est_check_factor = 0.5;
end

function model = trust_model(local_trust, gamma_self, target_flag, local_flag, global_flag)
model = struct( ...
    'local_trust_sample', local_trust, ...
    'gamma_self', gamma_self, ...
    'flag_target_attack', target_flag, ...
    'flag_local_est_check', local_flag, ...
    'flag_global_est_check', global_flag);
end

function assert_close(actual, expected)
actual = double(actual(:));
expected = double(expected(:));
tol = 1e-12;
assert(isequal(size(actual), size(expected)), 'Size mismatch');
assert(all(abs(actual - expected) < tol), ...
    'Expected %s, got %s', mat2str(expected'), mat2str(actual'));
end
