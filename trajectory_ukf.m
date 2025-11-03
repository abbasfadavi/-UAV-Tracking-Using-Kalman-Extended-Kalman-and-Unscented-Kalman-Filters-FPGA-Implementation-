function [z, x_true, acc_traj] = trajectory_ukf(state_dim,T,Q_pos,Q_vel,dt,R)
% Generate UAV trajectory with acceleration info
% acc_traj: 3 x T, acceleration at each step (برای استفاده در UKF)
x_true = zeros(state_dim, T, 'single');
x_true(:,1) = [0; 0; 10; 1; 0; 0];  % شروع در (0,0,10) با سرعت اولیه

acc_traj = zeros(3,T,'single');

for k = 2:T
    % تعیین شتاب بر اساس زمان
    if k < 25
        acc = [0.1; 0.05; 0.02];
    elseif k < 50
        acc = [0.05; 0.1; -0.1];
    elseif k < 75
        acc = [-0.1; 0.05; 0.05];
    else
        acc = [0.02; -0.1; 0.02];
    end
    acc_traj(:,k) = acc;

    % State update با شتاب
    x_true(1:3,k) = x_true(1:3,k-1) + dt*x_true(4:6,k-1) + 0.5*dt^2*acc;
    x_true(4:6,k) = x_true(4:6,k-1) + dt*acc;

    % Add process noise
    x_true(:,k) = x_true(:,k) + [sqrt(Q_pos)*randn(3,1); sqrt(Q_vel)*randn(3,1)];
end

% Measurement with measurement noise
z = x_true(1:3,:) + sqrtm(R)*randn(3,T);

z = single(z);
x_true = single(x_true);
end
