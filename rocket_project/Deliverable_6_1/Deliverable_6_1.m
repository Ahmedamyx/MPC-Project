Ts = 1/20;
rocket = Rocket(Ts);

H = 7; % Horizon length in seconds
nmpc = NmpcControl(rocket, H);

% MPC reference with default maximum roll = 15 deg
ref15 = @(t_, x_) ref_TVC(t_);

% MPC reference with specified maximum roll = 50 deg
roll_max = deg2rad(50);
ref50 = @(t_, x_) ref_TVC(t_, roll_max);

%% OPEN LOOP SIMULATION

x0 = zeros(12, 1);

[u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref15(1)');
U_opt(:,end+1) = nan;
rocket.anim_rate = 10;
ph = rocket.plotvis(T_opt, X_opt, U_opt, ref15(1)');
ph.fig.Name = 'NMPC OPEN LOOP DEFAULT MAXIMUM ROLL';

[u, T_opt, X_opt, U_opt] = nmpc.get_u(x0, ref50(1)');
U_opt(:,end+1) = nan;
rocket.anim_rate = 10;
ph1 = rocket.plotvis(T_opt, X_opt, U_opt, ref50(1)');
ph1.fig.Name = 'NMPC OPEN LOOP MAXIMUM ROLL =  50 deg';


%% CLOSED LOOP SIMULATION

Tf = 30;

%% gamma = 15 deg

[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref15);
rocket.anim_rate = 10;
ph3 =  rocket.plotvis(T,X,U,Ref);
ph3.fig.Name = 'NMPC CLOSED LOOP MAXIMUM ROLL =  15 deg';

%% gamma = 50 deg
        
[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref50);
rocket.anim_rate = 10;
ph3 =  rocket.plotvis(T,X,U,Ref);
ph3.fig.Name = 'NMPC CLOSED LOOP MAXIMUM ROLL =  50 deg';