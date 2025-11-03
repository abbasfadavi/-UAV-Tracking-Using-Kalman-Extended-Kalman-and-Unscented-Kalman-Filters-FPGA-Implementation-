function [z, x_true] = trajectory(state_dim, T, Q_pos, Q_vel, dt, R)
% Generates true UAV trajectory (3D position + velocity)
% and noisy measurements for testing the Kalman filter.
rng(5);
x_true = zeros(state_dim, T, 'single');
x_true(:,1) = single([0; 0; 10; 1; 0; 0]);  % initial position & velocity

% process noise standard deviations
sigma_pos = sqrt(Q_pos);
sigma_vel = sqrt(Q_vel);

for k = 2:T
    % Maneuver pattern (piecewise accelerations)
    if k < 25
        a = [0.1; 0.05; 0.02];
    elseif k < 50
        a = [0.05; 0.1; -0.1];
    elseif k < 75
        a = [-0.1; 0.05; 0.05];
    else
        a = [0.02; -0.1; 0.02];
    end

    % True state propagation
    x_prev = x_true(:,k-1);
    pos = x_prev(1:3) + dt * x_prev(4:6) + 0.5 * (dt^2) * a;
    vel = x_prev(4:6) + dt * a;

    % Add process noise (small random drift)
    pos = pos + sigma_pos * randn(3,1,'single');
    vel = vel + sigma_vel * randn(3,1,'single');

    x_true(:,k) = single([pos; vel]);
end

% Generate measurements: z = position + measurement noise
z = x_true(1:3,:) + sqrtm(R) * randn(3, T, 'single');
end
