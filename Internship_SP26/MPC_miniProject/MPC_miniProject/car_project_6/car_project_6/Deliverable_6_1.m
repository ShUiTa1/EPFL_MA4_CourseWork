clear; close; clc

Ts = 1/10; % Sample time
car = Car(Ts);
% Design MPC controller
N = 20;
H = N*Ts; % Horizon length in seconds
mpc = NmpcControl(car, H);



params = {};
params.Tf = 10;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 80/3.6]';
params.myCar.u = @mpc.get_u;
ref1 = [0 80/3.6]';
ref2 = [3 100/3.6]';
params.myCar.ref = car.ref_step(ref1, ref2, 2);
result = simulate(params);
visualization(car, result);
