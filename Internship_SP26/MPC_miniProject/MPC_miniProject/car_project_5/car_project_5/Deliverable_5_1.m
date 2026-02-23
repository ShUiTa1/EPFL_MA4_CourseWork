clear; close; clc

Ts = 1/10; % Sample time
car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);
% Design MPC controller
N = 20;
H_lon = N*Ts; % Horizon length in seconds
mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);
mpc_lat = MpcControl_lat(sys_lat, Ts, H_lon);
mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);

ref = [0 120/3.6]';
otherRef = 100 / 3.6;


params = {};
params.Tf = 10;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 100/3.6]';
params.myCar.u = @mpc.get_u;
params.myCar.ref = ref;
params.otherCar.model = car;
params.otherCar.x0 = [15 0 0 otherRef]';
params.otherCar.u = car.u_const(otherRef);


result = simulate(params);
visualization(car, result);
