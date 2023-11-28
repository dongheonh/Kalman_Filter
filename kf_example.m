% Time settings
T = 10;            % Total simulation time in seconds
dt = 0.1;          % Time step (Delta t)
time = 0:dt:T;     % Time vector

% State-Space model
A = [1 dt; 0 1];   % State transition matrix
H = [1 0];         % Measurement matrix

% Assume no control input for simplicity
B = [0; 0];        % Control matrix
u = 0;             % Control input

% Noise covariances (Assume some small values)
Q = 0.01 * eye(2); % Process noise covariance
R = 0.1;           % Measurement noise covariance

% Initialization
x_est = [0; 0];    % Initial state estimate (angle and angular velocity)
P_est = eye(2);    % Initial error covariance

% Simulating some true states (for demonstration purposes)
theta_true = sin(0.1*time);          % True angle (just a sine wave for example)
omega_true = 0.1*cos(0.1*time);      % True angular velocity
x_true = [theta_true; omega_true];   % True state matrix

% Creating noisy measurements
z = theta_true + sqrt(R)*randn(size(theta_true)); % Noisy measurements

% Kalman Filter Implementation
for k = 2:length(time)
    % Prediction
    x_pred = A * x_est(:, k-1) + B * u;
    P_pred = A * P_est(:, :, k-1) * A' + Q;

    % Measurement Update
    K = P_pred * H' / (H * P_pred * H' + R); % Kalman gain
    x_est(:, k) = x_pred + K * (z(k) - H * x_pred);
    P_est(:, :, k) = (eye(2) - K * H) * P_pred;

    % (In a real application, you would use the updated state estimate here)
end

% Plotting the results
figure;
plot(time, theta_true, 'g', 'DisplayName', 'True Angle');
hold on;
plot(time, z, 'b', 'DisplayName', 'Noisy Measurements');
plot(time, x_est(1, :), 'r', 'DisplayName', 'Kalman Estimate of Angle');
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Kalman Filter for Robot Arm Angle Estimation');
legend;
