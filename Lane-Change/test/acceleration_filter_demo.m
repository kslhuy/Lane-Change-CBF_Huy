%% Real-time Acceleration Filtering Demo
% This script shows how to enable and configure real-time acceleration filtering

clear; clc;

%% Initialize Trust Model
trust_model = TriPTrustModel();

fprintf('=== Real-time Acceleration Filtering Demo ===\n\n');

%% Configuration Options

% Option 1: Enable real-time acceleration filtering with default settings
fprintf('1. Basic Real-time Acceleration Filtering:\n');
trust_model.enable_realtime_acceleration_filter = true;
trust_model.acceleration_filter_type = 'median'; % Good for outlier rejection
fprintf('   - Real-time acceleration filtering: ENABLED\n');
fprintf('   - Filter type: %s\n', trust_model.acceleration_filter_type);

% Option 2: Configure specific filter type for acceleration
fprintf('\n2. Configure Acceleration Filter Type:\n');
trust_model.acceleration_filter_type = 'outlier_rejection'; % Best for acceleration spikes
trust_model.filter_window_size = 5; % Adjust sensitivity
fprintf('   - Filter type: %s\n', trust_model.acceleration_filter_type);
fprintf('   - Window size: %d\n', trust_model.filter_window_size);

% Option 3: Configure for different scenarios
fprintf('\n3. Scenario-based Configuration:\n');

% For normal driving conditions
fprintf('   Normal driving: median filter (outlier rejection)\n');
trust_model.acceleration_filter_type = 'median';
trust_model.filter_window_size = 5;

% For high-noise environments
fprintf('   High noise: moving average filter (smoothing)\n');
% trust_model.acceleration_filter_type = 'moving_average';
% trust_model.filter_window_size = 7;

% For responsive tracking
fprintf('   Responsive: exponential filter (low lag)\n');
% trust_model.acceleration_filter_type = 'exponential';
% trust_model.filter_alpha = 0.8;

%% Usage Examples

fprintf('\n4. Usage in Your Code:\n');
fprintf('   The filtering is automatically applied in calculateTrust() method\n');
fprintf('   No changes needed to your existing simulation code!\n\n');

fprintf('   Before (line 1370-1371):\n');
fprintf('   a_score = self.evaluate_acceleration(...);\n\n');

fprintf('   After (automatic):\n');
fprintf('   a_score_raw = self.evaluate_acceleration(...);\n');
fprintf('   if self.enable_realtime_acceleration_filter\n');
fprintf('       a_score = self.apply_single_score_filter(a_score_raw, ''acceleration'');\n');
fprintf('   else\n');
fprintf('       a_score = a_score_raw;\n');
fprintf('   end\n\n');

%% Quick Enable/Disable

fprintf('5. Quick Control:\n');
fprintf('   Enable:  trust_model.enable_realtime_acceleration_filter = true;\n');
fprintf('   Disable: trust_model.enable_realtime_acceleration_filter = false;\n\n');

%% Test with synthetic data
fprintf('6. Testing with Synthetic Data:\n');

% Generate noisy acceleration scores
num_steps = 50;
base_scores = 0.7 + 0.1 * sin((1:num_steps) * 0.2);
noisy_scores = base_scores + 0.15 * randn(1, num_steps);
% Add some spikes (outliers)
noisy_scores([10, 25, 40]) = [0.1, 0.9, 0.2]; 
noisy_scores = max(0, min(1, noisy_scores)); % Clamp to [0,1]

% Reset filter buffers
trust_model.reset_filters();

% Process and filter scores
filtered_scores = zeros(1, num_steps);
for i = 1:num_steps
    if trust_model.enable_realtime_acceleration_filter
        filtered_scores(i) = trust_model.apply_single_score_filter(noisy_scores(i), 'acceleration');
    else
        filtered_scores(i) = noisy_scores(i);
    end
end

% Plot comparison
figure('Name', 'Real-time Acceleration Filtering', 'Position', [100, 100, 900, 500]);
plot(1:num_steps, noisy_scores, 'b-o', 'LineWidth', 1, 'MarkerSize', 4, 'DisplayName', 'Raw Acceleration Scores');
hold on;
plot(1:num_steps, filtered_scores, 'r-s', 'LineWidth', 2, 'MarkerSize', 4, 'DisplayName', 'Filtered Acceleration Scores');
xlabel('Time Step');
ylabel('Acceleration Score');
title('Real-time Acceleration Filtering Demo');
legend('show', 'Location', 'best');
grid on;
ylim([0, 1]);

% Calculate metrics
variance_reduction = (var(noisy_scores) - var(filtered_scores)) / var(noisy_scores) * 100;
mean_diff = mean(abs(noisy_scores - filtered_scores));

text(0.6, 0.9, sprintf('Variance Reduction: %.1f%%', variance_reduction), ...
     'Units', 'normalized', 'FontSize', 12, 'BackgroundColor', 'white');
text(0.6, 0.85, sprintf('Mean Difference: %.3f', mean_diff), ...
     'Units', 'normalized', 'FontSize', 12, 'BackgroundColor', 'white');

fprintf('   Variance reduction: %.1f%%\n', variance_reduction);
fprintf('   Mean absolute difference: %.4f\n', mean_diff);

%% Summary
fprintf('\n=== Summary ===\n');
fprintf('✓ Real-time acceleration filtering is now integrated into calculateTrust()\n');
fprintf('✓ Independent control: enable_realtime_acceleration_filter\n');
fprintf('✓ No changes needed to your existing simulation loop\n');
fprintf('✓ Filter types: median, outlier_rejection, moving_average, exponential\n');
fprintf('✓ Automatic outlier detection and smoothing\n\n');

fprintf('Ready to use in your trust evaluation!\n');