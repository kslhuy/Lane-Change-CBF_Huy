% Simulation parameters
dt = 0.01; % time step
simulation_time = 23; % simulation time
num_steps = round(simulation_time / dt); % number of simulation steps

tau = 0.4; % constant time lag for the vehicle model

% Initial state vector [x1; x2; x3; x4; x5]
state = [10; 0; 0; 10; 0];

% Model parameters (continuous-time)
A_conti = [0 0 0 1 0;
           0 0 0 0 0;
           0 0 0 0 0;
           0 0 0 0 1;
           0 0 0 0 -1/tau];
B_conti = [0 0;
           0 0;
           0 0;
           0 0;
           1/tau 0];

% Discretization of the state-space model
A_discret = eye(length(A_conti)) + dt * A_conti;
B_discret = dt * B_conti;

% Kalman filter parameters
num_states = length(state);
% Q = 0.01 * eye(num_states); % Process noise covariance
% R = 0.01 * eye(num_states); % Measurement noise covariance

Q = 1e-6 * eye(num_states); % Small process noise covariance (no model error)
R = 1e-10 * eye(num_states); % Very small measurement noise covariance (perfect measurements)


P = eye(num_states); % Initial error covariance
est_state = zeros(num_states, 1); % Initial state estimate

% Measurement matrix (assuming all states are measured)
C = eye(num_states);

% Input signal (e.g., step input for the first control input)
input = [1; 0]; % Constant input [u1; u2]

% Storage for states and estimates
state_history = zeros(num_states, num_steps);
est_state_history = zeros(num_states, num_steps);

% Store states for comparison
state_history(:, 1) = state;
est_state_history(:, 1) = est_state;

% Simulate the system and run the Kalman filter
for i = 1:num_steps



    % Simulate the system (true state update)
    dX = A_discret * state + B_discret * input;
    state = dX; % Update true state

    % Generate noisy measurement
    % measurement = state + sqrt(R) * randn(num_states, 1);
    measurement = state; 
    % Run Kalman filter
    [est_state, P] = kalman_filter(est_state, P, measurement, A_discret, B_discret, input, Q, R, C);



    % Store states for comparison
    state_history(:, i+1) = state;
    est_state_history(:, i+1) = est_state;
end

% Plot results
t = (0:num_steps) * dt; % Time vector
figure;
for j = 1:num_states
    subplot(num_states, 1, j);
    plot(t, est_state_history(j, :) - state_history(j, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'True State');
    hold on;
    ylabel(['State x', num2str(j)]);
    grid on;
    legend;
    if j == num_states
        xlabel('Time (s)');
    end
end
sgtitle('True vs. Estimated States (Kalman Filter)');


% t = (0:num_steps-1) * dt; % Time vector
% figure;
% for j = 1:num_states
%     subplot(num_states, 1, j);
%     plot(t, state_history(j, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'True State');
%     hold on;
%     plot(t, est_state_history(j, :), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Estimated State');
%     ylabel(['State x', num2str(j)]);
%     grid on;
%     legend;
%     if j == num_states
%         xlabel('Time (s)');
%     end
% end
% sgtitle('True vs. Estimated States (Kalman Filter)');

% Kalman filter function
function [est_state, P] = kalman_filter(est_state, P, measurement, A, B, input, Q, R, C)
    % Prediction step
    x_pred = A * est_state + B * input;
    P_pred = A * P * A' + Q;

    % Measurement update step
    y = measurement; % Measurement
    S = C * P_pred * C' + R; % Innovation covariance
    K = P_pred * C' / S; % Kalman gain

    % Update state estimate
    est_state = x_pred + K * (y - C * x_pred);
    % Update error covariance
    P = (eye(size(P)) - K * C) * P_pred;
end