addpath(fullfile('..', 'src'));

%close all
%clear all
%clc

%% TODO: This file should produce all the plots for the deliverable

Ts = 1/20; % Sampling time
rocket = Rocket(Ts);
[xs, us] = rocket.trim();
sys = rocket.linearize(xs, us); % Linearization
[sys_x, sys_y, sys_z, sys_roll] = rocket.decompose(sys, xs, us);

Tf = 8.0; % Simulation time
H = 10.0; % Horizon length

% Initial states
x0 = [0; 0; 0; 3];
y0 = [0; 0; 0; 3];
z0 = [0; 3];
roll0 = [0; pi/6];

%% Design MPC controller for x

mpc_x = MpcControl_x(sys_x, Ts, H);

% Get control input
u_x = mpc_x.get_u(x0);

% Evaluate once and plot optimal open-loop trajectory
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x0);
U_opt(:,end+1) = NaN;
% Account for linearization point
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us);

% Evaluate and plot closed-loop trajectory accounting for linearization
% point
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us);

%% Design MPC controller for y

mpc_y = MpcControl_y(sys_y, Ts, H);

% Get control input
u_y = mpc_y.get_u(y0);

% Evaluate once and plot optimal open-loop trajectory
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_y.get_u(y0);
U_opt(:,end+1) = NaN;
% Account for linearization point
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us);

% Evaluate and plot closed-loop trajectory accounting for linearization
% point
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us);

%% Design MPC controller for z

mpc_z = MpcControl_z(sys_z, Ts, H);

% Get control input
u_z = mpc_z.get_u(z0);

% Evaluate once and plot optimal open-loop trajectory
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(z0);
U_opt(:,end+1) = NaN;
% Account for linearization point
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us);

% Evaluate and plot closed-loop trajectory accounting for linearization
% point
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us);

%% Design MPC controller for roll (gamma)

mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% Get control input
u_roll = mpc_roll.get_u(roll0);

% Evaluate once and plot optimal open-loop trajectory
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(roll0);
U_opt(:,end+1) = NaN;
% Account for linearization point
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us);

% Evaluate and plot closed-loop trajectory accounting for linearization
% point
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, 0);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us);