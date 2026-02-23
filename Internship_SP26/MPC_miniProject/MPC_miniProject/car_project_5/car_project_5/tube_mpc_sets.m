clear; close all; clc

%% 1) Load or define system
Ts       = 1/10; 
car      = Car(Ts);
[xs, us] = car.steady_state(100/3.6);
sys      = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);
N = 50;
H_lon = N*Ts; % Horizon length in seconds
mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);

A        = mpc_lon.A;
B_input  = -mpc_lon.B;  % negative sign
B_noise  =  mpc_lon.B;  % used for disturbance
u_T_s    =  us(2);

%% 2) LQR
Q = 0.1 * eye(2);
R = 1.0;
[K, Qf, ~] = dlqr(A, B_input, Q, R);
K = -K;
P = Qf;
A_cl = A + B_input*K;


%% 3) Define disturbance set
W = Polyhedron('lb', -0.5+u_T_s, 'ub', 0.5+u_T_s);
%% 4) Compute the minimal robust invariant set (mRPI) via iteration:
maxIter = 100;
n       = size(A_cl,1);
F_old   = Polyhedron('V', zeros(1,n));  % F0 = {0}
% B_noise * W 
EW = W.affineMap(B_noise);
for i = 1:maxIter
    % A_cl * F_old
    AF = F_old.affineMap(A_cl);
    % Minkowski sum: F_new
    F_new = AF + EW;
    % Reduce redundant constraints
    F_new.minHRep();
    
    % Check if F_new == F_old
    if norm(A_cl^i) < 1e-2
        fprintf('mRPI converged at iteration %d.\n', i);
        break;
    end
    F_old = F_new;
end
% Create a new figure each iteration
figure; 
hold on; grid on; axis equal;
xlabel('deltaX'); % Label for x-axis
ylabel('deltaV'); % Label for y-axis
% Plot AF in red
plot(AF, 'Color','g', 'Alpha',0.3);
% Plot EW in green
plot(EW, 'Color','r', 'Alpha',1.0);
% Plot F_new in blue
plot(F_new, 'Color','b', 'Alpha',0.1);
legend('AF','EW','F_{new}','Location','best');
title(sprintf('Iteration %d', i));



%% 5) Tightened Constraints
deltax_ub =  8;
deltax_lb = -4;
deltav_ub =  20;
deltav_lb = -20;
A_con = [ 1  0;     %  deltax <= 8
         -1  0;     %  deltax >= -4
          0  1;     %  deltav <= 20
          0 -1;];   %  deltav >= -20
b_con = [ deltax_ub; 
         -deltax_lb;
         deltav_ub;
        -deltav_lb;];
X = Polyhedron(A_con, b_con);
U = Polyhedron('lb', Car.lbu(2), 'ub', Car.ubu(2));

% We have computed our error invariant set E = F_new
E = F_new;

% 1) Tightened state constraint:  X_tilde = X - E
X_tilde = X - E;

% 2) Tightened input constraint:  U_tilde = U - (K*E)
KE   = E.affineMap(K);   % apply K to each point in E
U_tilde = U - KE;

figure; hold on; grid on; axis equal;
plot(X, 'Color','r', 'Alpha',0.1);
plot(X_tilde, 'Color','b', 'Alpha',0.1);
xlabel('deltaX'); % Label for x-axis
ylabel('deltaV'); % Label for y-axis
legend('X','X_tilde');
title('Original vs. tightened state constraints');
figure; hold on; grid on; axis equal;
plot(U, 'Color','r', 'Alpha',1.0);
plot(U_tilde, 'Color','b', 'Alpha',0.1);
xlabel('U'); % Label for x-axis
legend('U','U_tilde');
title('Original vs. tightened state constraints');
% figure; KE
% plot(KE, 'Color','b', 'Alpha',0.1);

%% 6) Terminal Set
% First constract by U_tilde and X_tilde
A_U = U_tilde.A; 
b_U = U_tilde.b;
XK = Polyhedron('A', A_U*K, 'b', b_U);
X0 = intersect(X_tilde, XK); 
% Initial state constraint set
Xf = X0;

% Compute maximal invariant set
max_iter = 1000; i = 0;
while true
    prevXf = Xf; % Store the previous set for comparison
    % Extract current constraints
    [T, t] = deal(Xf.A, Xf.b); 
    % Compute the pre-set
    preXf = Polyhedron('A', T * A_cl, 'b', t);
    % Compute the intersection of the current set and the pre-set
    Xf = intersect(Xf, preXf);
    Xf.minHRep();
    % Check for convergence
    i = i + 1;
    if prevXf == Xf || i > max_iter
        break;
    end
end

% Plot the initial terminal set
figure;
hold on;
xlabel('deltaX'); % Label for x-axis
ylabel('deltaV'); % Label for y-axis
title('Maximal Invariant Set'); % Title of the plot

% Shrink Set
plot(X_tilde, 'color', 'pink', 'alpha', 0.3);
plot(Xf, 'color', 'purple', 'alpha', 0.3);
legend('X_{tilde}', 'Xf')
%% 7) Save Result
save('tube_mpc_data.mat', 'A', 'B_input', 'X_tilde', 'U_tilde', 'Xf', 'P', 'K', 'E');
