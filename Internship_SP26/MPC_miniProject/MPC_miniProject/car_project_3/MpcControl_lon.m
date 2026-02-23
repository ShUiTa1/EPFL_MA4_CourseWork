classdef MpcControl_lon < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            % setup
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   V_ref, u_ref - reference state/input
            %   d_est        - disturbance estimate
            %   x0other      - initial state of other car
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps %ceil upper inter
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            A = mpc.A;
            B = mpc.B;
            % Targets
            X_ref = sdpvar(nx,1);
            u_ref = sdpvar(nu,1);
            X = sdpvar(nx, N); % State trajectory
            U = sdpvar(nu, N-1); % Input trajectory
            % Disturbance estimate (Ignore this before Todo 4.1)
            d_est = sdpvar(1);

            % Initial states
            x0 = X(:, 1); %2x1
            x0other = sdpvar(nx, 1); % (Ignore this before Todo 5.1)
            
            % Input to apply to the system
            u0 = U(:, 1); %1x1
            xs = mpc.xs;
            us = mpc.us;
            f_xs_us = mpc.f_xs_us;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system.
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.

            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            con = [];

            Q = 1.0 * eye(2); % State tracking cost
            R = 1; % Input cost
            % P = dlyap(A(2,2),Q(2,2)); % Terminal cost (if applicable) 
            % Compute LQR controller for unconstrained system
            [K,Qf,~] = dlqr(A(2,2),B(2,1),Q(2,2),R);
            P = Qf;
            % MATLAB defines K as -K, so invert its signal
            K = -K; 
            M = [1;-1];
            m = [Car.ubu(2); -Car.lbu(2)];

            % Compute maximal invariant set
            Xf = polytope(M*K,m);
            Acl = [A(2,2)+B(2,1)*K];
            while 1
                prevXf = Xf;
                [T,t] = double(Xf);
                preXf = polytope(T*Acl,t);
                Xf = intersect(Xf, prevXf);
                if isequal(prevXf, Xf)
                    break
                end
            end
            [Ff,ff] = double(Xf);
         
            % Replace this line and set u0 to be the input that you
            % want applied to the system. Note that u0 is applied directly
            % to the nonlinear system. You need to take care of any 
            % offsets resulting from the linearization.
            % If you want to use the delta formulation make sure to
            % substract mpc.xs/mpc.us accordingly.
            % con = con + ( u0 == 0 );
            con = [con, X_ref(1) == 0];
            for k = 1:N-1
                % Dynamics constraint
%                 con = [con, X(:, k+1) == f_xs_us + A * (X(:, k)-xs) + B * (U(:, k)-us)];
%                 Dynamics constraint (with disturbance estimate)
                con = [con, X(:, k+1) == f_xs_us + A * (X(:, k) - xs) + B * (U(:, k) - us) + B*d_est];
    
                % Input constraints
                con = [con, -1.0 <= U(:, k) <= 1.0]; 

                % Objective function
                obj = obj + (X(2, k) - X_ref(2))' * Q(2,2) * (X(2, k) - X_ref(2)) + ...
                            (U(:, k) - u_ref)' * R * (U(:, k) - u_ref);
            end
            obj = obj + (X(2, N) - X_ref(2))' * P * (X(2, N) - X_ref(2));
            %con = [con, Ff * (X(2, N)-X_ref(2)) <=  ff]; % Terminal constraint

            % Pass here YALMIP sdpvars which you want to debug. You can
            % then access them when calling your mpc controller like
            debugVars = {X, U};
            % Pass here YALMIP sdpvars which you want to debug. You can
            % then access them when calling your mpc controller like
            % [u, X, U] = mpc_lon.get_u(x0, ref);
            % with debugVars = {X_var, U_var};
            %-X_ref(2)  - X_ref
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %gurobi
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, X_ref, u_ref, d_est, x0other}, {u0, debugVars{:}});
        end

        % Computes the steady state target which is passed to the
        % controller
        function [Vs_ref, us_ref] = compute_steady_state_target(mpc, ref, odom, d_est)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            %   d_est  - disturbance estimate (Ignore before Todo 4.1)
            % OUTPUTS
            %   Vs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Compute alpha based on the difference, ensuring it is between 0 and 1
            gain = 0.25;
            diff = abs(odom(2) - ref);
            alpha = 1 / (1 + gain * diff); % Sigmoid-like mapping to [0, 1]
            if alpha > 0.80
                alpha = 1.0;
            end
            ref = alpha * ref + (1-alpha) * odom(2);
            % disp('Lon alpha:')
            % disp(alpha)


            % Steady-state subsystem
            A = mpc.A;
            B = mpc.B;
            nx = size(A,1);
            nu = size(B,2);  
            % Subsystem linearization steady-state
            xs = mpc.xs; % V_ref
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            f_xs_us = mpc.f_xs_us;
            u = sdpvar(nu,1);
            x = sdpvar(nx,1);

            umin = Car.lbu(2);
            umax = Car.ubu(2); 

            constraints = [umin <= u <= umax ,...
                           x(2) == f_xs_us(2) + A(2, 2) * (x(2) - xs(2)) + B(2, :) * (u - us) + B(2)*d_est, ...
                           ref == x(2),...
                           x(1) == xs(1)       ];
            
            objective   = u^2;
            diagnostics = solvesdp(constraints,objective,sdpsettings('verbose',0));
            
            if diagnostics.problem == 0
               % Good! 
            elseif diagnostics.problem == 1
                throw(MException('','Infeasible'));
            else
                throw(MException('','Something else happened'));
            end
            
            Vs_ref = double(x);
            us_ref = double(u);

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
