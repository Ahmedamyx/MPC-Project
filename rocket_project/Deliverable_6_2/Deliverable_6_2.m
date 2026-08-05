Ts = 1/40; % Higher sampling rate for this part!
rocket = Rocket(Ts);

H = 7; % Horizon length in seconds
nmpc = NmpcControl6_2(rocket, H );

x0 = zeros(12, 1);
ref = [0.5, 0, 1, deg2rad(65)]';
Tf = 2.5;

rocket.mass = 1.75;

%% No delay
% 
% rocket.delay = 0; 
% [T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
% 
% rocket.anim_rate = 1;
% ph =  rocket.plotvis(T,X,U,Ref);
% ph.fig.Name = '0 delay';
%
%% Max working delay
% 
% rocket.delay = 2; 
% [T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
% 
% rocket.anim_rate = 1;
% ph =  rocket.plotvis(T,X,U,Ref);
% ph.fig.Name = '2 delay';
% 
% %% Min non working delay
% 
% rocket.delay = 3; 
% [T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
% 
% rocket.anim_rate = 1;
% ph =  rocket.plotvis(T,X,U,Ref);
% ph.fig.Name = '3 delay';
% 
%% Fully delay compensated

expected_delay = 3;

nmpc = NmpcControl6_2(rocket, H, expected_delay);

rocket.delay = 3;

[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
rocket.anim_rate = 1;
ph1 =  rocket.plotvis(T,X,U,Ref);
ph1.fig.Name = 'Fully delay compensated';

%%  Partially delay compensated 

expected_delay = 2;

nmpc = NmpcControl6_2(rocket, H, expected_delay);

rocket.delay = 3;

[T, X, U, Ref] = rocket.simulate(x0, Tf, @nmpc.get_u, ref);
rocket.anim_rate = 1;
ph1 =  rocket.plotvis(T,X,U,Ref);
ph1.fig.Name = 'Partially delay compensated';

