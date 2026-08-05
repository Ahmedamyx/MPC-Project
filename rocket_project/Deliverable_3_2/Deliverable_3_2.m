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

Tf = 10.0; % Simulation length
H = 10.0; % Horizon length

% Initial states and references
x0 = [0; 0; 0; 0];
x_ref = -4;

y0 = [0; 0; 0; 0];
y_ref = -4;

z0 = [0; 0];
z_ref = -4;

roll0 = [0; 0];
roll_ref = 0.61;

%% Design MPC controller for x

mpc_x = MpcControl_x(sys_x, Ts, H);

% Evaluate once and plot optimal open-loop trajectory
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_x.get_u(x0, x_ref);
U_opt(:,end+1) = NaN;
% Account for linearization point
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_x, xs, us);

% Evaluate and plot closed-loop trajectory accounting for linearization
% point
[T, X_sub, U_sub] = rocket.simulate_f(sys_x, x0, Tf, @mpc_x.get_u, x_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_x, xs, us, x_ref);

%% Design MPC controller for y

mpc_y = MpcControl_y(sys_y, Ts, H);

% Evaluate once and plot optimal open-loop trajectory
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_y.get_u(y0, y_ref);
U_opt(:,end+1) = NaN;
% Account for linearization point
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_y, xs, us);

% Evaluate and plot closed-loop trajectory accounting for linearization
% point
[T, X_sub, U_sub] = rocket.simulate_f(sys_y, y0, Tf, @mpc_y.get_u, y_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_y, xs, us, y_ref);

%% Design MPC controller for z

mpc_z = MpcControl_z(sys_z, Ts, H);

% Evaluate once and plot optimal open-loop trajectory
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_z.get_u(z0, z_ref);
U_opt(:,end+1) = NaN;
% Account for linearization point
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_z, xs, us);

% Evaluate and plot closed-loop trajectory accounting for linearization
% point
[T, X_sub, U_sub] = rocket.simulate_f(sys_z, z0, Tf, @mpc_z.get_u, z_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_z, xs, us, z_ref);

%% Design MPC controller for roll (gamma)

mpc_roll = MpcControl_roll(sys_roll, Ts, H);

% Evaluate once and plot optimal open-loop trajectory
% pad last input to get consistent size with time and state
[u, T_opt, X_opt, U_opt] = mpc_roll.get_u(roll0, roll_ref);
U_opt(:,end+1) = NaN;
% Account for linearization point
ph = rocket.plotvis_sub(T_opt, X_opt, U_opt, sys_roll, xs, us);

% Evaluate and plot closed-loop trajectory accounting for linearization
% point
[T, X_sub, U_sub] = rocket.simulate_f(sys_roll, roll0, Tf, @mpc_roll.get_u, roll_ref);
ph = rocket.plotvis_sub(T, X_sub, U_sub, sys_roll, xs, us, roll_ref);