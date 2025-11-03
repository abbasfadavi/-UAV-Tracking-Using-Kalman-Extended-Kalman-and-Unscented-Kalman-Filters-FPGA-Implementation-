function [z, x_true] = trajectory_ekf(state_dim, T, Q_pos, Q_vel, dt, R)
% TRAJECTORY_EKF
% Generates a 3D UAV trajectory and nonlinear radar-like measurements
% State model: [x; y; z; vx; vy; vz]
% Measurement model (nonlinear): [range; azimuth; elevation]
%
% range     = sqrt(x^2 + y^2 + z^2)
% azimuth   = atan2(y, x)
% elevation = atan2(z, sqrt(x^2 + y^2))
%
% Inputs:
%   state_dim : number of states (should be 6)
%   T         : number of time steps
%   Q_pos     : process noise variance (position)
%   Q_vel     : process noise variance (velocity)
%   dt        : time step
%   R         : measurement noise covariance (3x3)
%
% Outputs:
%   z         : 3xT nonlinear measurements (range, azimuth, elevation)
%   x_true    : 6xT true state trajectory

x_true = zeros(state_dim, T, 'single');
x_true(:,1) = single([0; 0; 10; 1; 0; 0]);  % initial position and velocity

%% Generate true UAV trajectory
for k = 2:T
    % simple maneuver pattern
    if k < 25
        acc = [0.1; 0.05; 0.02];
    elseif k < 50
        acc = [0.05; 0.1; -0.1];
    elseif k < 75
        acc = [-0.1; 0.05; 0.05];
    else
        acc = [0.02; -0.1; 0.02];
    end

    % Update motion
    x_true(1:3,k) = x_true(1:3,k-1) + dt * x_true(4:6,k-1) + 0.5 * dt^2 * acc;
    x_true(4:6,k) = x_true(4:6,k-1) + dt * acc;

    % Add small process noise
    x_true(:,k) = x_true(:,k) + [sqrt(Q_pos)*randn(3,1); sqrt(Q_vel)*randn(3,1)];
end

%% Generate nonlinear radar-like measurements
z = zeros(3, T, 'single');
for k = 1:T
    px = x_true(1,k);
    py = x_true(2,k);
    pz = x_true(3,k);

    % true measurement (nonlinear)
    range     = sqrt(px^2 + py^2 + pz^2);
    azimuth   = atan2(py, px);
    elevation = atan2(pz, sqrt(px^2 + py^2));

    z(:,k) = [range; azimuth; elevation];
end

% Add measurement noise
for k = 1:T
    z(:,k) = z(:,k) + sqrtm(R) * randn(3,1);
end

% Convert to single precision
z = single(z);
x_true = single(x_true);
end
