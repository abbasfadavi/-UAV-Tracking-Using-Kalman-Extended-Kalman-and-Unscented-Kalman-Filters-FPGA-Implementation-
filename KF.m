%% Advanced 3D Kalman Filter - UAV Tracking in XYZ Space
clc; clear; close all;
fs = 20e6;
ts = 1/fs;
state_dim = 6;
T = 100;
dt = single(0.1);
% process & measurement noise (same as your particle filter)
Q_pos = single(0.01);
Q_vel = single(0.05);
Q = single(blkdiag(Q_pos * eye(3), Q_vel * eye(3)));
R = single(diag([0.5, 0.5, 1.0]));    % measurement covariance (3x3)
A = single([eye(3), dt * eye(3);zeros(3), eye(3)]);
H = single([eye(3), zeros(3)]);
[z, x_true] = trajectory(state_dim, T, Q_pos, Q_vel, dt, R);
save_file(z','z_matrix.bin');
%% initialization
x_est = zeros(state_dim, T, 'single');   % state estimates
P = single(eye(state_dim) * 10.0);       % initial covariance 
%% Main Kalman loop
for t = 1:T
    if t == 1
        x_est(:,1) = single([z(:,1); zeros(3,1)]);
    else
        % Prediction step
        x_pred = A * x_est(:,t-1);
        P_pred = A * P * A' + Q;
        % Update step (Kalman gain)
        S = H * P_pred * H' + R;
        K = (P_pred * H') / S;
        % State update
        y = single(z(:,t)) - H * x_pred;
        x_upd = x_pred + K * y;
        KH = K * H;
        P = (eye(state_dim) - KH) * P_pred;
        % Save estimate
        x_est(:,t) = single(x_upd);
    end
end
%%
x_est'
err = sum(sum(abs(x_true - x_est)))
save_file(x_est','x_matrix.bin');
%% helper functions
function save_file(x,name)
fid = fopen(name, 'wb');
fwrite(fid,x, 'float32');
fclose(fid);
end
