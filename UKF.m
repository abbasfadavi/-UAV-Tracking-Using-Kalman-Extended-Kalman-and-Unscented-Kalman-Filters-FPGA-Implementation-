clc; clear; close all;
format longG
rng(25)
state_dim = 6;
T = 100;
dt = 0.1;
Q_pos = 0.01; Q_vel = 0.001;
R = single(diag([0.5, deg2rad(2), deg2rad(2)]));
%%
[z, x_true, acc_traj] = trajectory_ukf(state_dim,T,Q_pos,Q_vel,dt,R);
save_file(z','z_matrix.bin');
%%
Q_scale = 10;
state_dim = 6;
meas_dim = 3;
T = 100;
alpha = single(0.05);
beta = single(2);
kappa = single(0);
lambda = alpha^2*(state_dim+kappa)-state_dim;
c = state_dim + lambda;
% weights
L = single(2*state_dim+1);
Wm = zeros(L,1,'single');
Wc = zeros(L,1,'single');
Wm(1) = lambda/c;
Wc(1) = Wm(1) + (1-alpha^2+beta);
for i=2:L
    Wm(i) = 1/(2*c);
    Wc(i) = Wm(i);
end
% process noise (scaled)
Q = single(Q_scale * blkdiag(eye(3), eye(3)));
H = single([eye(3), zeros(3)]);
for t=1:T
    if t == 1
        P = single(eye(state_dim)*10);
        x_est(:,1) = [z(:,1); zeros(3,1)];
    else
        % --- Sigma points ---
        S = chol(c*(P + 1e-6*eye(state_dim)),'lower');
        Xsigma = zeros(state_dim,L,'single');
        Xsigma(:,1) = x_est(:,t-1);
        for i=1:state_dim
            Xsigma(:,i+1          ) = x_est(:,t-1)+S(:,i);
            Xsigma(:,i+1+state_dim) = x_est(:,t-1)-S(:,i);
        end
        % --- Prediction (linear motion) ---
        Xsigma_pred = zeros(state_dim,L,'single');
        for i=1:L
            Xsigma_pred(1:3,i) = Xsigma(1:3,i) + dt*Xsigma(4:6,i); % pos
            Xsigma_pred(4:6,i) = Xsigma(4:6,i);                     % vel
        end

        % predicted mean
        x_pred = sum(Xsigma_pred .* Wm',2);

        % predicted covariance
        P_pred = zeros(state_dim,state_dim,'single');
        for i=1:L
            dx = Xsigma_pred(:,i)-x_pred;
            P_pred = P_pred + Wc(i)*(dx*dx');
        end

        P_pred = P_pred + Q; % process noise added here

        % Measurement prediction
        
        Zsigma = H*Xsigma_pred;
        z_pred = sum(Zsigma .* Wm',2);

        % Innovation covariance and cross covariance
        S_mat = zeros(3,3,'single');
        Pxz = zeros(state_dim,3,'single');
        for i=1:L
            dz = Zsigma(:,i)-z_pred;
            dx = Xsigma_pred(:,i)-x_pred;
            S_mat = S_mat + Wc(i)*(dz*dz');
            Pxz = Pxz + Wc(i)*(dx*dz');
        end
        S_mat = S_mat + R;
        K = Pxz / S_mat;
        y = z(:,t) - z_pred;
        x_upd = single(x_pred + K*y);
        ksk = K*S_mat*K';
        P = P_pred - ksk;
        x_est(:,t) = x_upd;
    end
end
P_est = P;
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
