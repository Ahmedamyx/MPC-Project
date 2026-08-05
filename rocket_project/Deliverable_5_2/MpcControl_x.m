classdef MpcControl_x < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE

            A = mpc.A; B = mpc.B;
            d2_max = 0.26;
            beta_max = 0.1745;

            % u in U = { u | Mu <= m }
            M = [1; -1]; m = [d2_max; d2_max];

            % x in X = { x | Fx <= f }
            F = [0 1 0 0; 0 -1 0 0]; 
            f = [beta_max; beta_max];

            % Cost matrices
            Q = [10 0 0 0;
                 0 10 0 0;
                 0 0 10 0;
                 0 0 0 40];
            R = 0.001*eye(nu);

            % Compute LQR controller for unconstrained system
            [K,Qf,~] = dlqr(A,B,Q,R);
            % MATLAB defines K as -K, so invert its signal
            K = -K; 
            
            % Compute maximal invariant set
            Xf = polytope([F;M*K],[f;m]);
            Acl = [A+B*K];
            while 1
                prevXf = Xf;
                [T,t] = double(Xf);
                preXf = polytope(T*Acl,t);
                Xf = intersect(Xf, preXf);
                if isequal(prevXf, Xf)
                    break
                end
            end
            [Ff,ff] = double(Xf);

            % Define objectives and constraints
            con = (X(:,2) == A*X(:,1) + B*U(:,1)) + (M*U(:,1) <= m);
            obj = (U(:,1)-u_ref)'*R*(U(:,1)-u_ref);
            
            for i = 2:N-1
            con = con + (X(:,i+1) == A*X(:,i) + B*U(:,i));
            con = con + (F*X(:,i) <= f) + (M*U(:,i) <= m);
            obj = obj + (X(:,i)-x_ref)'*Q*(X(:,i)-x_ref);
            obj = obj + (U(:,i)-u_ref)'*R*(U(:,i)-u_ref);
            end
            
            con = con + (Ff*X(:,N) <= ff);
            obj = obj + (X(:,N)-x_ref)'*Qf*(X(:,N)-x_ref);
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, {U(:,1), X, U});
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);

            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D

            A = mpc.A; B = mpc.B; C = mpc.C;
            d2_max = 0.26;
            beta_max = 0.1745;

            % u in U = { u | Mu <= m }
            M = [1; -1]; m = [d2_max; d2_max];

            % x in X = { x | Fx <= f }
            F = [eye(nx); -eye(nx)]; 
            f = [inf; beta_max; inf; inf; inf; beta_max; inf; inf];

            obj = us^2;
            con = [A*xs + B*us == xs, C*xs == ref, F*xs <= f, M*us <= m];
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
