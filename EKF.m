clc; clear; close all;
format longG
state_dim = 6;
T = 100;
dt = 0.1;
Q_pos = single(0.01); 
Q_vel = single(0.001);
Q = single(diag([Q_pos*ones(1,3), Q_vel*ones(1,3)]));
R = single(diag([0.5, deg2rad(2), deg2rad(2)]));
rng(25)
% Generate true trajectory and nonlinear measurements
[z, x_true] = trajectory_ekf(state_dim, T, Q_pos, Q_vel, dt, R);
save_file(z','z_matrix.bin');
T = size(z, 2);
x_est = zeros(6, T, 'single');
P_est = zeros(6, 6, 'single');
F = single([eye(3), dt*eye(3); zeros(3), eye(3)]);
%% Main EKF loop
for k = 1:T
    if k == 1
        x_est(:,1) = single([0; 0; 10; 1; 0; 0]);
        P_est = eye(6, 'single') * 10;
    else
        % ---------- Prediction ----------
        x_pred = F * x_est(:,k-1);
        P_pred = F * P_est * F' + Q;
        % ---------- Measurement update ----------
        z_pred = h_meas(x_pred);
        H = jacobian_h(x_pred);
        y = z(:,k-1) - z_pred;  % innovation
        % Normalize angles (to avoid wraparound)
        y(2) = wrapToPi(y(2));
        y(3) = wrapToPi(y(3));
        S = H * P_pred * H' + R;
        K = P_pred * H' / S;
        x_est(:,k) = x_pred + K * y;
        KH = (eye(6,'single') - K * H);
        P_est = KH * P_pred;
    end
end
x_est'
err = sum(sum(abs(x_true - x_est)))
save_file(x_est','x_matrix.bin');
%% Nonlinear measurement function
function z = h_meas(x)
px = x(1);
py = x(2);
pz = x(3);
range = sqrt(px^2 + py^2 + pz^2);
azimuth = atan2(py, px);
elevation = atan2(pz, sqrt(px^2 + py^2));
z = single([range; azimuth; elevation]);
end
%% Jacobian of h(x)
function H = jacobian_h(x)
px = x(1);
py = x(2); 
pz = x(3);
r = sqrt(px^2 + py^2 + pz^2);
rho_xy = sqrt(px^2 + py^2);

% Avoid division by zero
if r < 1e-6
    H = zeros(3,6,'single');
    return;
end

% Partial derivatives
dr_dx = px / r;
dr_dy = py / r;
dr_dz = pz / r;

daz_dx = -py / (px^2 + py^2);
daz_dy =  px / (px^2 + py^2);
daz_dz = 0;

del_dx = -px*pz / (r^2 * rho_xy);
del_dy = -py*pz / (r^2 * rho_xy);
del_dz =  rho_xy / (r^2);

H = single([ ...
    dr_dx, dr_dy, dr_dz, 0, 0, 0;
    daz_dx, daz_dy, daz_dz, 0, 0, 0;
    del_dx, del_dy, del_dz, 0, 0, 0]);
end
%% helper functions
function save_file(x,name)
fid = fopen(name, 'wb');
fwrite(fid,x, 'float32');
fclose(fid);
end
