addpath(fullfile('..', 'src'));

%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20;
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

H = 5.0; % Horizon length

% Define the MPC controllers
mpc_x = MpcControl_x(sys_x, Ts, H);
mpc_y = MpcControl_y(sys_y, Ts, H);
mpc_z = MpcControl_z(sys_z, Ts, H);
mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% Merge the four controllers into one for the whole system
mpc = rocket.merge_lin_controllers(xs, us, mpc_x, mpc_y, mpc_z, mpc_roll);

% Initial state and reference
x0 = [zeros(1, 9), 1 0 3]';
ref = [1.2, 0, 3, 0]';

Tf = 8; % Simulation length

% Manipulate mass for simulation
rocket.mass = 2.13;

% Simulate without offset-free tracking
[T, X, U, Ref] = rocket.simulate(x0, Tf, @mpc.get_u, ref);
ph = rocket.plotvis(T, X, U, Ref);

% Simulate with offset-free tracking
[T, X, U, Ref, Z_hat] = rocket.simulate_est_z(x0, Tf, @mpc.get_u, ref, mpc_z, sys_z);
ph = rocket.plotvis(T, X, U, Ref);