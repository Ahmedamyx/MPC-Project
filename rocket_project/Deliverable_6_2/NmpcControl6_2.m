classdef NmpcControl6_2 < handle
    
    properties
        solver
        nx, nu, N
        nlp_x0
        nlp_lbx, nlp_ubx
        nlp_lbg, nlp_ubg
        nlp_p
        
        T_opt
        sol
        idx
        
        % Delay compensation
        rocket
        expected_delay
        mem_u
        
        % Warmstart
        nlp_lam_x0
        nlp_lam_g0
    end
    
     methods
        function obj = NmpcControl6_2(rocket, tf, expected_delay)
        
            if nargin < 3, expected_delay = 0; end

            import casadi.*
            opti = casadi.Opti();

            N_segs = ceil(tf/rocket.Ts); % MPC horizon
            nx = 12; % Number of states
            nu = 4;  % Number of inputs
            
            % Decision variables (symbolic)
            N = N_segs + 1; % Index of last point
           
            X_sym = SX.sym('X_sym', nx, N); % state trajectory
            U_sym = SX.sym('U_sym', nu, N-1); % control trajectory)
            
            % Parameters (symbolic)
            x0_sym  = SX.sym('x0_sym', nx, 1);  % initial state
            ref_sym = SX.sym('ref_sym', 4, 1);  % target position
            
            % Default state and input constraints
            ubx = inf(nx, 1);
            lbx = -inf(nx, 1);
            ubu = inf(nu, 1);
            lbu = -inf(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            h = rocket.Ts;
            f = @(x,u) rocket.f(x,u);
            f_discrete = @(x,u) RK4(x,u,h,f);
            
            cost = 0;
            X_ref = [0 0 0 0 0 ref_sym(4) 0 0 0 ref_sym(1) ref_sym(2) ref_sym(3)]'; 

            %Gamma = 15°
            %Q = diag([5,5,5,5,5,75,1,1,1,50,50,100]);
            %Q = 15*eye(nx);
            %Gamma = 50°
            Q = diag([5,5,25,25,25,100,1,1,75,50,50,200]);
            
            R = diag([1,1,0.001,0.001]);
            Qf = Q;

            % Box constraints on state and input
            ubx(5) = deg2rad(75);
            lbx(5) = deg2rad(-75);

            ubu(1) = deg2rad(15);
            lbu(1) = deg2rad(-15);

            ubu(2) = deg2rad(15);
            lbu(2) = deg2rad(-15);

            ubu(3) = 80;        
            lbu(3) = 50;

            ubu(4) = 20;
            lbu(4)= -20;

            eq_constr=[;];
            ineq_constr=[;];

            % initial state constraint
            eq_constr = [eq_constr; X_sym(:,1) - x0_sym]; 

            for k = 1:N-1
                X_next = f_discrete(X_sym(:,k), U_sym(:,k));
                eq_constr = [eq_constr; X_sym(:,k+1) - X_next]; 
            end

            for k = 1:N
                ineq_constr = [ineq_constr; lbx(5) - X_sym(5,k)];
                ineq_constr = [ineq_constr; X_sym(5,k) - ubx(5)];
                if k < N  
                    ineq_constr = [ineq_constr; lbu - U_sym(:,k)];
                    ineq_constr = [ineq_constr; U_sym(:,k) - ubu];
                end
            end


            % Accumulate cost over the prediction horizon
            for k = 1:N-1
                state_deviation = X_sym(:,k) - X_ref;
                control_deviation = U_sym(:,k); %-us si offset en openlooop
                cost = cost + state_deviation' * Q * state_deviation + ...
                       control_deviation' * R * control_deviation;
            end
            
            % Add terminal cost for the final state deviation from trimmed state
            cost = cost + (X_sym(:,N) - X_ref)' * Qf * (X_sym(:,N) - X_ref);

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % ---- Assemble NLP ------
            nlp_x = [X_sym(:); U_sym(:)];
            nlp_p = [x0_sym; ref_sym];
            nlp_f = cost;
            nlp_g = [eq_constr; ineq_constr];
            
            nlp = struct('x', nlp_x, 'p', nlp_p, 'f', nlp_f, 'g', nlp_g);
            
            % ---- Setup solver ------
            opts = struct('ipopt', struct('print_level', 0), 'print_time', false);
            obj.solver = nlpsol('solver', 'ipopt', nlp, opts);
            
            % ---- Assemble NLP bounds ----
            obj.nlp_x0  = zeros(size(nlp_x));
            
            obj.nlp_ubx = [repmat(ubx, N, 1); repmat(ubu, (N-1), 1)];
            obj.nlp_lbx = [repmat(lbx, N, 1); repmat(lbu, (N-1), 1)];
            
            obj.nlp_ubg = [zeros(size(eq_constr)); zeros(size(ineq_constr))];
            obj.nlp_lbg = [zeros(size(eq_constr)); -inf(size(ineq_constr))];
            
            obj.nlp_p = [zeros(size(x0_sym)); zeros(size(ref_sym))];
            
            obj.nlp_lam_x0 = [];
            obj.nlp_lam_g0 = [];
            
            obj.nx = nx;
            obj.nu = nu;
            obj.N = N;
            obj.T_opt = linspace(0, N * rocket.Ts, N);
            
            obj.idx.X = [1, obj.N * obj.nx];
            obj.idx.U = obj.idx.X(2) + [1, (obj.N-1) * obj.nu];
            obj.idx.u0 = obj.idx.U(1) + [0, obj.nu-1];
            
            % Members for delay compensation
            obj.rocket = rocket;
            obj.expected_delay = expected_delay;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            [~, us] = trim(obj.rocket);
            u_init = us; 
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.mem_u = repmat(u_init, 1, expected_delay);
        end
        
        function [u, T_opt, X_opt, U_opt] = get_u(obj, x0, ref)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            delay = obj.expected_delay;
            mem_u = obj.mem_u;
            
            % Delay compensation: Predict x0 delay timesteps later.
            % Simulate x_ for 'delay' timesteps
            x_ = x0;
            for i=1:delay
                x_ = x_ + obj.rocket.Ts*obj.rocket.f(x_,mem_u(:,i));
            end

            x0 = x_;
           
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute solution from x0
            obj.solve(x0, ref);
            
            % Evaluate u0
            nlp_x = obj.sol.x;
            id = obj.idx.u0;
            u = full( nlp_x(id(1):id(2)) );      
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % Delay compensation: Save current u
            if obj.expected_delay > 0
                obj.mem_u = [obj.mem_u(:, 2:end),u];  
            end
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            if nargout > 1, T_opt = obj.get_T_opt(); end
            if nargout > 2, X_opt = obj.get_X_opt(); end
            if nargout > 3, U_opt = obj.get_U_opt(); end
            return
        end
        
        function solve(obj, x0, ref)
            % ---- Set the initial state and reference ----
            obj.nlp_p = [x0; ref];     % Initial condition
            obj.nlp_x0(1:obj.nx) = x0; % Initial guess consistent
            
            % ---- Solve the optimization problem ----
            args = {'x0', obj.nlp_x0, ...
                'lbg', obj.nlp_lbg, ...
                'ubg', obj.nlp_ubg, ...
                'lbx', obj.nlp_lbx, ...
                'ubx', obj.nlp_ubx, ...
                'p', obj.nlp_p, ...
                %                 'lam_x0', obj.nlp_lam_x0, ...
                %                 'lam_g0', obj.nlp_lam_g0
                };
            
            obj.sol = obj.solver(args{:});
            if obj.solver.stats.success ~= true
                solve_status_str = obj.solver.stats.return_status;
                fprintf([' [' class(obj) ': ' solve_status_str '] ']);
                obj.sol.x(obj.idx.u0) = nan;
            end
            
            % Use the current solution to speed up the next optimization
            obj.nlp_x0 = obj.sol.x;
            obj.nlp_lam_x0 = obj.sol.lam_x;
            obj.nlp_lam_g0 = obj.sol.lam_g;
        end
        function T_opt = get_T_opt(obj)
            T_opt = obj.T_opt;
        end
        function X_opt = get_X_opt(obj)
            nlp_x = obj.sol.x;
            id = obj.idx.X;
            X_opt = full(reshape(nlp_x(id(1):id(2)), obj.nx, obj.N));
        end
        function U_opt = get_U_opt(obj)
            nlp_x = obj.sol.x;
            id = obj.idx.U;
            U_opt = full(reshape(nlp_x(id(1):id(2)), obj.nu, obj.N - 1));
        end
    end
end

