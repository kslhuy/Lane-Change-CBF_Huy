function test_python_matlab_trust_cross_language()
% Compare local fusion, Dual Dirichlet evolution, EMA, and flags to Python.

root_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(root_dir, 'core', 'Trust'));
generator = fullfile(root_dir, 'python', 'generate_trust_parity_fixture.py');
[status, output] = system(sprintf('python "%s"', generator));
assert(status == 0, 'Python trust fixture generator failed: %s', output);
golden = jsondecode(strtrim(output));

model = TriPTrustModel();
model.local_trust_fusion_mode = "weighted_geometric";
local_sample = model.calculate_trust_sample_python(0.8, 0.6, 0.4, 1.0, 0.9, 0.7);
assert_close(local_sample, golden.local_sample);

sequence = double(golden.sample_sequence);
expected_scores = double(golden.final_scores(:));
previous = NaN;
actual_scores = zeros(size(expected_scores));
for idx = 1:size(sequence, 1)
    model.update_rating_vector(sequence(idx, 1), "local");
    local_score = model.calculate_trust_score(model.rating_vector);
    model.update_rating_vector(sequence(idx, 2), "global");
    global_score = model.calculate_trust_score(model.rating_vector_global);
    final_score = local_score * global_score;
    if isfinite(previous)
        final_score = model.ema_alpha * final_score + (1 - model.ema_alpha) * previous;
    end
    previous = final_score;
    actual_scores(idx) = final_score;

    model.set_python_attack_flags(sequence(idx, 1), sequence(idx, 2), 0.5);
    actual_flags = logical([model.flag_taget_attk, model.flag_glob_est_check, model.flag_local_est_check]);
    expected_flags = logical(golden.flags(idx, :));
    assert(isequal(actual_flags, expected_flags), 'Attack-flag mismatch at row %d.', idx);
end
assert_close(actual_scores, expected_scores);

fprintf('test_python_matlab_trust_cross_language passed\n');
end

function assert_close(actual, expected)
actual = double(actual(:));
expected = double(expected(:));
tol = 1e-12;
assert(isequal(size(actual), size(expected)), 'Size mismatch');
assert(all(abs(actual - expected) < tol), ...
    'Expected %s, got %s', mat2str(expected'), mat2str(actual'));
end
