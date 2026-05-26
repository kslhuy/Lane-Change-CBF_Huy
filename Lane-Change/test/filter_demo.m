%% Trust Score Filtering System Demo
% This script demonstrates how to use the comprehensive filtering system
% in the TriPTrustModel class

clear; clc; close all;

%% Initialize Trust Model
trust_model = TriPTrustModel();

%% Demo 1: Basic Filtering Configuration
fprintf('=== Trust Score Filtering System Demo ===\n\n');

% Show the demonstration
trust_model.demonstrate_filter_usage();

%% Demo 2: Test Different Filter Types
fprintf('\n=== Testing Different Filter Types ===\n');

% Simulate noisy score data for testing
num_steps = 100;
time_vector = 1:num_steps;

% Generate synthetic noisy scores (simulating real trust evaluation)
base_v_score = 0.8 + 0.15 * sin(time_vector * 0.1); % Sinusoidal base
base_d_score = 0.7 + 0.1 * cos(time_vector * 0.08);
base_a_score = 0.6 + 0.2 * sin(time_vector * 0.12);

% Add noise and outliers
noise_level = 0.1;
v_scores = base_v_score + noise_level * randn(1, num_steps);
d_scores = base_d_score + noise_level * randn(1, num_steps);
a_scores = base_a_score + noise_level * randn(1, num_steps);

% Add some outliers
outlier_indices = [20, 35, 60, 75];
v_scores(outlier_indices) = v_scores(outlier_indices) + 0.4 * randn(1, length(outlier_indices));
d_scores(outlier_indices) = d_scores(outlier_indices) - 0.3 * randn(1, length(outlier_indices));
a_scores(outlier_indices) = 0.1 * rand(1, length(outlier_indices)); % Very low scores

% Ensure scores are within [0,1]
v_scores = max(0, min(1, v_scores));
d_scores = max(0, min(1, d_scores));
a_scores = max(0, min(1, a_scores));

% Beacon scores (binary with some dropouts)
beacon_scores = ones(1, num_steps);
beacon_dropout_indices = [15:18, 45:47, 80:85];
beacon_scores(beacon_dropout_indices) = 0;

% Heading scores (generally high with occasional issues)
h_scores = 0.9 + 0.05 * randn(1, num_steps);
h_scores = max(0, min(1, h_scores));

%% Test Configuration 1: Moving Average Filters
fprintf('\nConfiguration 1: Moving Average Filters\n');
trust_model.enable_score_filtering = true;
trust_model.velocity_filter_type = 'moving_average';
trust_model.distance_filter_type = 'moving_average';
trust_model.acceleration_filter_type = 'moving_average';
trust_model.beacon_filter_type = 'moving_average';
trust_model.heading_filter_type = 'moving_average';
trust_model.filter_window_size = 5;

% Reset filters and process data
trust_model.reset_filters();

% Process each time step
for i = 1:num_steps
    [filt_v, filt_d, filt_a, filt_b, filt_h] = trust_model.filter_all_scores(...
        v_scores(i), d_scores(i), a_scores(i), beacon_scores(i), h_scores(i));
    
    % Store for logging (the filter method already logs internally)
end

% Create comparison plot
figure('Name', 'Moving Average Filter Comparison', 'Position', [100, 100, 1200, 800]);

subplot(2, 3, 1);
plot(time_vector, v_scores, 'b-', 'LineWidth', 1, 'DisplayName', 'Raw');
hold on;
plot(time_vector, trust_model.filtered_v_score_log, 'r-', 'LineWidth', 2, 'DisplayName', 'Filtered');
title('Velocity Score - Moving Average Filter');
xlabel('Time Step'); ylabel('Score');
legend('show'); grid on; ylim([0, 1]);

subplot(2, 3, 2);
plot(time_vector, d_scores, 'b-', 'LineWidth', 1, 'DisplayName', 'Raw');
hold on;
plot(time_vector, trust_model.filtered_d_score_log, 'r-', 'LineWidth', 2, 'DisplayName', 'Filtered');
title('Distance Score - Moving Average Filter');
xlabel('Time Step'); ylabel('Score');
legend('show'); grid on; ylim([0, 1]);

subplot(2, 3, 3);
plot(time_vector, a_scores, 'b-', 'LineWidth', 1, 'DisplayName', 'Raw');
hold on;
plot(time_vector, trust_model.filtered_a_score_log, 'r-', 'LineWidth', 2, 'DisplayName', 'Filtered');
title('Acceleration Score - Moving Average Filter');
xlabel('Time Step'); ylabel('Score');
legend('show'); grid on; ylim([0, 1]);

%% Test Configuration 2: Mixed Filter Types
fprintf('\nConfiguration 2: Mixed Filter Types (Optimized)\n');

% Reset and configure mixed filters
trust_model.reset_filters();
trust_model.velocity_filter_type = 'exponential';      % Smooth velocity tracking
trust_model.distance_filter_type = 'median';           % Outlier rejection for distance
trust_model.acceleration_filter_type = 'outlier_rejection'; % Handle acceleration spikes
trust_model.beacon_filter_type = 'threshold';          % Binary beacon handling
trust_model.heading_filter_type = 'adaptive_weighted'; % Recent heading emphasis

trust_model.filter_alpha = 0.7; % For exponential filter

% Process data again
for i = 1:num_steps
    [filt_v, filt_d, filt_a, filt_b, filt_h] = trust_model.filter_all_scores(...
        v_scores(i), d_scores(i), a_scores(i), beacon_scores(i), h_scores(i));
end

% Plot mixed filter results
subplot(2, 3, 4);
plot(time_vector, v_scores, 'b-', 'LineWidth', 1, 'DisplayName', 'Raw');
hold on;
plot(time_vector, trust_model.filtered_v_score_log, 'g-', 'LineWidth', 2, 'DisplayName', 'Exponential');
title('Velocity Score - Exponential Filter');
xlabel('Time Step'); ylabel('Score');
legend('show'); grid on; ylim([0, 1]);

subplot(2, 3, 5);
plot(time_vector, d_scores, 'b-', 'LineWidth', 1, 'DisplayName', 'Raw');
hold on;
plot(time_vector, trust_model.filtered_d_score_log, 'g-', 'LineWidth', 2, 'DisplayName', 'Median');
title('Distance Score - Median Filter');
xlabel('Time Step'); ylabel('Score');
legend('show'); grid on; ylim([0, 1]);

subplot(2, 3, 6);
plot(time_vector, a_scores, 'b-', 'LineWidth', 1, 'DisplayName', 'Raw');
hold on;
plot(time_vector, trust_model.filtered_a_score_log, 'g-', 'LineWidth', 2, 'DisplayName', 'Outlier Rejection');
title('Acceleration Score - Outlier Rejection');
xlabel('Time Step'); ylabel('Score');
legend('show'); grid on; ylim([0, 1]);

sgtitle('Trust Score Filtering Demonstration', 'FontSize', 14, 'FontWeight', 'bold');

%% Demo 3: Filtering vs No Filtering Trust Sample Comparison
fprintf('\nConfiguration 3: Trust Sample Calculation Comparison\n');

% Calculate trust samples with and without filtering
trust_model.reset_filters();
trust_model.enable_score_filtering = true;

trust_samples_filtered = zeros(1, num_steps);
trust_samples_raw = zeros(1, num_steps);

is_nearby = true; % Assume nearby for this demo

for i = 1:num_steps
    % Calculate with filtering
    trust_samples_filtered(i) = trust_model.calculate_trust_sample_with_filtering(...
        v_scores(i), d_scores(i), a_scores(i), beacon_scores(i), h_scores(i), is_nearby);
    
    % Calculate without filtering (disable temporarily)
    trust_model.enable_score_filtering = false;
    trust_samples_raw(i) = trust_model.calculate_trust_sample_with_filtering(...
        v_scores(i), d_scores(i), a_scores(i), beacon_scores(i), h_scores(i), is_nearby);
    trust_model.enable_score_filtering = true; % Re-enable
end

% Plot trust sample comparison
figure('Name', 'Trust Sample Comparison', 'Position', [150, 150, 1000, 600]);

subplot(2, 1, 1);
plot(time_vector, trust_samples_raw, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Raw Trust Sample');
hold on;
plot(time_vector, trust_samples_filtered, 'r-', 'LineWidth', 2, 'DisplayName', 'Filtered Trust Sample');
title('Trust Sample Comparison: Raw vs Filtered');
xlabel('Time Step'); ylabel('Trust Sample');
legend('show', 'Location', 'best'); grid on; ylim([0, 1]);

% Calculate improvement metrics
variance_reduction = (var(trust_samples_raw) - var(trust_samples_filtered)) / var(trust_samples_raw) * 100;
mean_difference = mean(abs(trust_samples_raw - trust_samples_filtered));

subplot(2, 1, 2);
plot(time_vector, abs(trust_samples_raw - trust_samples_filtered), 'g-', 'LineWidth', 1.5);
title(sprintf('Absolute Difference (Variance Reduction: %.1f%%, Mean Diff: %.3f)', ...
              variance_reduction, mean_difference));
xlabel('Time Step'); ylabel('|Raw - Filtered|');
grid on;

%% Summary
fprintf('\n=== Demo Summary ===\n');
fprintf('1. Filtering system successfully reduces noise in trust scores\n');
fprintf('2. Different filter types are suitable for different score characteristics:\n');
fprintf('   - Moving Average: General smoothing\n');
fprintf('   - Median: Outlier rejection\n');
fprintf('   - Exponential: Responsive smoothing\n');
fprintf('   - Threshold: Binary signal cleanup\n');
fprintf('   - Adaptive Weighted: Recent value emphasis\n');
fprintf('   - Outlier Rejection: Statistical anomaly handling\n');
fprintf('3. Variance reduction achieved: %.1f%%\n', variance_reduction);
fprintf('4. Mean absolute difference: %.4f\n', mean_difference);
fprintf('\nFiltering system ready for integration into trust evaluation!\n');