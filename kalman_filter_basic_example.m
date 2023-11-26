clear, clc 
% Define the system
A = [1 1; 0 1]; % State Transition Matrix
H = [1 0]; % Measurement Matrix
Q = [0.001 0; 0 0.001]; % Process Noise Covariance
R = 0.1; % Measurement Noise Covariance
P = eye(2); % Initial Estimate Covariance
x = [0; 0]; % Initial State Estimate

% Measurements
z = [1.0; 1.5; 2.0; 2.2; 3.3; 123; 123131 ...
    ; 1.0; 1; 1; 1; 1; 1; 1; 1; 1; 1]; % Example measurements

% Kalman Filter Iterations
for k = 1:length(z)
    % Prediction
    x = A * x;
    P = A * P * A' + Q;
    
    % Update
    K = P * H' / (H * P * H' + R);
    x = x + K * (z(k) - H * x);
    P = (eye(2) - K * H) * P;
    
    % Display the results
    disp(['Iteration ' num2str(k) ':']);
    disp(['State estimate: ' num2str(x')]);
    disp('Covariance: ');
    disp(P);
end
