# Trust Score Filtering System

## Overview

The Trust Score Filtering System is a comprehensive filtering framework integrated into the `TriPTrustModel` class. It provides configurable filtering capabilities for all trust score types (velocity, distance, acceleration, beacon, and heading scores) to improve trust evaluation reliability by reducing noise and handling outliers.

## Key Features

- **Comprehensive Score Filtering**: Filters all 5 types of trust scores
- **Multiple Filter Types**: 6 different filtering algorithms to choose from
- **Configurable Parameters**: Adjustable filter settings per score type  
- **Master Enable/Disable**: Global switch to enable/disable all filtering
- **Individual Score Control**: Enable/disable filtering per score type
- **Real-time Processing**: Efficient filtering during trust evaluation
- **Visualization Tools**: Built-in plotting functions to analyze filter effectiveness
- **Easy Integration**: Drop-in replacement for existing trust calculation methods

## Available Filter Types

### 1. Moving Average (`'moving_average'`)
- **Description**: Smooths scores using a sliding window average
- **Best For**: General noise reduction, stable smoothing
- **Parameters**: `filter_window_size` (default: 5)
- **Characteristics**: Reduces noise, introduces lag

### 2. Median Filter (`'median'`)
- **Description**: Uses median of sliding window to reject outliers
- **Best For**: Outlier rejection, preserving edges
- **Parameters**: `filter_window_size` (default: 5)  
- **Characteristics**: Excellent outlier rejection, preserves signal jumps

### 3. Exponential Filter (`'exponential'`)
- **Description**: Exponential moving average for responsive smoothing
- **Best For**: Responsive tracking with noise reduction
- **Parameters**: `filter_alpha` (default: 0.7, range: 0-1)
- **Characteristics**: Low lag, adjustable responsiveness

### 4. Threshold Filter (`'threshold'`)
- **Description**: Clips values outside acceptable range
- **Best For**: Binary signals, enforcing bounds
- **Parameters**: `filter_threshold_min`, `filter_threshold_max`
- **Characteristics**: Hard limiting, preserves signal within bounds

### 5. Adaptive Weighted (`'adaptive_weighted'`)
- **Description**: Recent values get higher weights
- **Best For**: Time-varying signals, recent emphasis
- **Parameters**: Internal exponential decay (0.3)
- **Characteristics**: Emphasizes recent history

### 6. Outlier Rejection (`'outlier_rejection'`)
- **Description**: Statistical outlier detection and rejection
- **Best For**: Handling statistical anomalies
- **Parameters**: 2-sigma threshold (built-in)
- **Characteristics**: Intelligent outlier handling

## Configuration Methods

### Method 1: Direct Property Assignment
```matlab
% Enable master filtering
trust_model.enable_score_filtering = true;

% Configure individual score filters
trust_model.enable_velocity_filter = true;
trust_model.velocity_filter_type = 'moving_average';

trust_model.enable_distance_filter = true;  
trust_model.distance_filter_type = 'median';

trust_model.enable_acceleration_filter = true;
trust_model.acceleration_filter_type = 'outlier_rejection';

% Set filter parameters
trust_model.filter_window_size = 5;
trust_model.filter_alpha = 0.7;
trust_model.filter_threshold_min = 0.1;
trust_model.filter_threshold_max = 0.9;
```

### Method 2: Structured Configuration
```matlab
% Create configuration structure
filter_configs = struct();
filter_configs.velocity = struct('enable', true, 'type', 'exponential');
filter_configs.distance = struct('enable', true, 'type', 'median');
filter_configs.acceleration = struct('enable', false, 'type', 'none');
filter_configs.beacon = struct('enable', true, 'type', 'threshold');
filter_configs.heading = struct('enable', true, 'type', 'adaptive_weighted');

% Apply configuration
trust_model.configure_filtering(true, filter_configs);
```

### Method 3: Preset Configurations

#### Configuration A: Noise Reduction Focus
```matlab
trust_model.enable_score_filtering = true;
trust_model.velocity_filter_type = 'moving_average';
trust_model.distance_filter_type = 'moving_average';
trust_model.acceleration_filter_type = 'moving_average';
trust_model.beacon_filter_type = 'moving_average';
trust_model.heading_filter_type = 'moving_average';
trust_model.filter_window_size = 5;
```

#### Configuration B: Outlier Rejection Focus
```matlab
trust_model.enable_score_filtering = true;
trust_model.velocity_filter_type = 'median';
trust_model.distance_filter_type = 'outlier_rejection';
trust_model.acceleration_filter_type = 'outlier_rejection';
trust_model.beacon_filter_type = 'threshold';
trust_model.heading_filter_type = 'median';
```

#### Configuration C: Responsive Tracking
```matlab
trust_model.enable_score_filtering = true;
trust_model.velocity_filter_type = 'exponential';
trust_model.distance_filter_type = 'adaptive_weighted';
trust_model.acceleration_filter_type = 'exponential';
trust_model.beacon_filter_type = 'threshold';
trust_model.heading_filter_type = 'exponential';
trust_model.filter_alpha = 0.8; % Higher responsiveness
```

## Usage in Trust Calculation

### Replace Existing Trust Sample Calculation
Replace your current trust sample calculation:

```matlab
% OLD METHOD
trust_sample = trust_model.calculate_trust_sample(v_score, d_score, a_score, beacon_score, h_score, is_nearby);
```

With the filtered version:

```matlab
% NEW METHOD (with filtering)
trust_sample = trust_model.calculate_trust_sample_with_filtering(v_score, d_score, a_score, beacon_score, h_score, is_nearby);
```

### Alternative: Weighted Filtered Calculation
For weighted-based trust calculation:

```matlab
trust_sample = trust_model.calculate_trust_sample_filtered_weighted(v_score, d_score, a_score, beacon_score, h_score, is_nearby);
```

### Manual Filtering Control
If you want to filter scores manually:

```matlab
[filtered_v, filtered_d, filtered_a, filtered_b, filtered_h] = ...
    trust_model.filter_all_scores(v_score, d_score, a_score, beacon_score, h_score);

% Use filtered scores in your own calculation
trust_sample = your_custom_calculation(filtered_v, filtered_d, filtered_a, filtered_b, filtered_h);
```

## Filter Management

### Reset Filters
Clear all filter buffers and reset to initial state:
```matlab
trust_model.reset_filters();
```

### Disable All Filtering
Temporarily disable all filtering:
```matlab
trust_model.enable_score_filtering = false;
```

### Check Filter Status
```matlab
if trust_model.enable_score_filtering
    fprintf('Filtering is enabled\n');
    fprintf('Velocity filter: %s (%s)\n', string(trust_model.enable_velocity_filter), trust_model.velocity_filter_type);
    fprintf('Distance filter: %s (%s)\n', string(trust_model.enable_distance_filter), trust_model.distance_filter_type);
    % ... etc
end
```

## Visualization and Analysis

### Plot Filter Comparison
Compare raw vs filtered scores:
```matlab
trust_model.plot_filter_comparison(host_vehicle_id, target_vehicle_id);
```

### Plot Filter Effectiveness
Analyze filter performance metrics:
```matlab
trust_model.plot_filter_effectiveness(host_vehicle_id, target_vehicle_id);
```

## Performance Considerations

### Computational Cost
- **Moving Average**: O(1) per sample
- **Median**: O(n log n) where n = window size  
- **Exponential**: O(1) per sample
- **Threshold**: O(1) per sample
- **Adaptive Weighted**: O(n) where n = window size
- **Outlier Rejection**: O(n) where n = window size

### Memory Usage
- Each filter maintains a buffer of size `filter_window_size`
- 5 score types × buffer size × 8 bytes per double
- Default: 5 × 5 × 8 = 200 bytes total

### Recommended Settings

#### For Real-Time Applications
```matlab
trust_model.filter_window_size = 3;  % Smaller buffer
trust_model.velocity_filter_type = 'exponential';  % Fast
trust_model.distance_filter_type = 'exponential';
trust_model.acceleration_filter_type = 'threshold';  % Very fast
```

#### For High Accuracy Applications  
```matlab
trust_model.filter_window_size = 7;  % Larger buffer
trust_model.velocity_filter_type = 'adaptive_weighted';
trust_model.distance_filter_type = 'outlier_rejection';
trust_model.acceleration_filter_type = 'outlier_rejection';
```

## Integration Examples

### Example 1: Basic Integration
```matlab
% Initialize trust model
trust_model = TriPTrustModel();

% Configure filtering
trust_model.enable_score_filtering = true;
trust_model.velocity_filter_type = 'moving_average';
trust_model.distance_filter_type = 'median';
trust_model.filter_window_size = 5;

% Use in your simulation loop
for t = 1:simulation_steps
    % ... calculate raw scores ...
    
    % Calculate trust with filtering
    trust_sample = trust_model.calculate_trust_sample_with_filtering(...
        v_score, d_score, a_score, beacon_score, h_score, is_nearby);
    
    % ... use trust_sample in your application ...
end

% Analyze results
trust_model.plot_filter_comparison(host_id, target_id);
```

### Example 2: Adaptive Filtering Based on Conditions
```matlab
% Configure different filtering for different scenarios
if attack_detected
    % Use aggressive outlier rejection during attacks
    trust_model.velocity_filter_type = 'outlier_rejection';
    trust_model.distance_filter_type = 'outlier_rejection';
    trust_model.acceleration_filter_type = 'median';
elseif high_noise_environment  
    % Use strong smoothing in noisy conditions
    trust_model.velocity_filter_type = 'moving_average';
    trust_model.filter_window_size = 7;
else
    % Use responsive filtering in normal conditions
    trust_model.velocity_filter_type = 'exponential';
    trust_model.filter_alpha = 0.8;
end
```

### Example 3: Scenario-Specific Configuration
```matlab
% Different filter configurations for different vehicle types
if vehicle_type == "leader"
    % Leader vehicles need responsive tracking
    configure_responsive_filtering(trust_model);
elseif vehicle_type == "follower"
    % Follower vehicles need smooth control
    configure_smooth_filtering(trust_model);
elseif vehicle_type == "emergency"
    % Emergency vehicles need outlier rejection
    configure_robust_filtering(trust_model);
end

function configure_responsive_filtering(trust_model)
    trust_model.enable_score_filtering = true;
    trust_model.velocity_filter_type = 'exponential';
    trust_model.distance_filter_type = 'exponential';
    trust_model.filter_alpha = 0.9; % High responsiveness
end

function configure_smooth_filtering(trust_model)
    trust_model.enable_score_filtering = true;
    trust_model.velocity_filter_type = 'moving_average';
    trust_model.distance_filter_type = 'moving_average';
    trust_model.filter_window_size = 6;
end

function configure_robust_filtering(trust_model)
    trust_model.enable_score_filtering = true;
    trust_model.velocity_filter_type = 'outlier_rejection';
    trust_model.distance_filter_type = 'median';
    trust_model.acceleration_filter_type = 'outlier_rejection';
end
```

## Troubleshooting

### Common Issues

#### 1. No Filtered Data in Plots
**Problem**: `plot_filter_comparison()` shows warning about no filtered data
**Solution**: Ensure filtering is enabled and trust evaluation has been run:
```matlab
trust_model.enable_score_filtering = true;
% Run your simulation with trust evaluation
trust_model.plot_filter_comparison(host_id, target_id);
```

#### 2. Excessive Smoothing
**Problem**: Filtered signals are too smooth, losing important dynamics
**Solution**: Reduce window size or use more responsive filter:
```matlab
trust_model.filter_window_size = 3;  % Smaller window
% OR
trust_model.velocity_filter_type = 'exponential';
trust_model.filter_alpha = 0.9;  % More responsive
```

#### 3. Insufficient Noise Reduction
**Problem**: Filtered signals still contain too much noise
**Solution**: Increase smoothing or use different filter:
```matlab
trust_model.filter_window_size = 7;  % Larger window
% OR
trust_model.velocity_filter_type = 'median';  % Better outlier rejection
```

#### 4. Performance Issues
**Problem**: Filtering is too slow for real-time application
**Solution**: Use faster filter types:
```matlab
trust_model.velocity_filter_type = 'exponential';  % O(1) complexity
trust_model.distance_filter_type = 'threshold';    % O(1) complexity
trust_model.filter_window_size = 3;               % Smaller buffers
```

## Testing and Validation

Run the comprehensive demo to test the filtering system:

```matlab
% Run the demonstration script
run('filter_demo.m');

% Or use the built-in demonstration
trust_model = TriPTrustModel();
trust_model.demonstrate_filter_usage();
```

This will show:
1. Basic configuration examples
2. Comparative analysis of different filter types
3. Performance metrics (variance reduction, smoothness improvement)
4. Visual comparisons of raw vs filtered signals

## Best Practices

1. **Start Simple**: Begin with moving average filters before trying more advanced options
2. **Test Thoroughly**: Use the demo scripts to understand filter behavior
3. **Monitor Performance**: Check computational impact in your specific application
4. **Visualize Results**: Always plot filtered vs raw signals to verify behavior
5. **Scenario-Specific**: Adapt filter configuration based on operating conditions
6. **Reset When Needed**: Call `reset_filters()` when starting new simulations
7. **Document Settings**: Keep track of filter configurations that work well

The filtering system provides a powerful and flexible way to improve trust score reliability while maintaining the ability to disable filtering entirely if needed. Choose filter types and parameters based on your specific application requirements for noise characteristics, computational constraints, and response time needs.