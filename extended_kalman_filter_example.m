clear; clc;

% System parameters
Q = 0.1; % Process noise covariance
R = 0.1; % Measurement noise covariance
P = 1;   % Initial estimate covariance
x = 0.1; % Initial state estimate

% Nonlinear state evolution and measurement functions
f = @(x) x^2;
h = @(x) sqrt(x);

% Number of iterations
N = 20;

% Extended Kalman Filter iterations
for k = 1:N
    % Simulate system (for demonstration)
    x_true = f(x) + sqrt(Q) * randn;
    
    % Simulate measurement
    z = h(x_true) + sqrt(R) * randn;

    % EKF Prediction
    F = 2 * x; % Jacobian of f at the current state
    x = f(x);  % Predicted state
    P = F * P * F' + Q; % Predicted estimate covariance

    % EKF Update
    H = 1 / (2 * sqrt(x)); % Jacobian of h at the current state
    K = P * H' / (H * P * H' + R); % Kalman gain
    x = x + K * (z - h(x)); % Updated state estimate
    P = (1 - K * H) * P;   % Updated estimate covariance

    % Display results
    disp(['Iteration ' num2str(k) ':']);
    disp(['True state: ' num2str(x_true)]);
    disp(['State estimate: ' num2str(x)]);
    disp(['Covariance: ' num2str(P)]);
end
