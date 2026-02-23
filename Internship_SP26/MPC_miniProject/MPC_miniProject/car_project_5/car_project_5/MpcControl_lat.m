classdef MpcControl_lat < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets
            X_ref = sdpvar(nx,1);
            u_ref = sdpvar(nu,1);
            % Initial states
            x0other = sdpvar(nx, 1); % (Ignore this, not used)

            % Input to apply to the system

            A = mpc.A;
            B = mpc.B;

            X = sdpvar(nx, N); % State trajectory
            U = sdpvar(nu, N-1); % Input trajectory
            % Disturbance estimate (Ignore this before Todo 4.1)
            %d_est = sdpvar(1);

            % Initial states
            x0 = X(:, 1); %2x1
            x0other = sdpvar(nx, 1); % (Ignore this before Todo 5.1)
            
            % Input to apply to the system
            u0 = U(:, 1); %1x1
            xs = mpc.xs;
            us = mpc.us;
            f_xs_us = mpc.f_xs_us;


            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            con = [];

            Q = eye(2); % State tracking cost
            R = 1; % Input cost
            

            % Compute LQR controller for unconstrained system
            [K,Qf,~] = dlqr(A,B,Q,R);
            P = Qf; % Terminal cost (if applicable) 
            % MATLAB defines K as -K, so invert its signal
            K = -K; 
            M = [1;-1]; m = [Car.ubu(1); -Car.lbu(1)];
            F = [1 0; 0 1; -1 0; 0 -1]; f = [Car.ubx(2); Car.ubx(3); -Car.lbx(2); -Car.lbx(3)];
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
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            

            % Replace this line and set u0 to be the input that you
            % want applied to the system. Note that u0 is applied directly
            % to the nonlinear system. You need to take care of any 
            % offsets resulting from the linearization.
            % If you want to use the delta formulation make sure to
            % substract mpc.xs/mpc.us accordingly.
        
            con = [con, x0 == X(:, 1)];
            for k = 1:N-1
                % Dynamics constraint
                con = [con, X(:, k+1) == f_xs_us + A * (X(:, k)-xs) + B * (U(:, k)-us)];
    
                % Input constraints
                con = [con, Car.lbu(1) <= U(:,k) <= Car.ubu(1)];
                con = [con, Car.lbx(2) <= X(1,k) <= Car.ubx(2)];
                con = [con, Car.lbx(3) <= X(2,k) <= Car.ubx(3)];
                % Objective function
                obj = obj + (X(:, k) - X_ref)' * Q * (X(:, k) - X_ref) + ...
                            (U(:, k) - u_ref)' * R * (U(:, k) - u_ref);
            end
            con = [con, Car.lbx(2) <= X(1,N) <= Car.ubx(2)];
            con = [con, Car.lbx(3) <= X(2,N) <= Car.ubx(3)];
            obj = obj + (X(:, N) - X_ref)' * P * (X(:, N) - X_ref);
            con = [con, Ff * (X(:, N)-X_ref)<=  ff]; % Terminal constraint

            % Pass here YALMIP sdpvars which you want to debug. You can
            % then access them when calling your mpc controller like
            debugVars = {X, U};
            

            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, X_ref, u_ref, x0other}, {u0, debugVars{:}});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [xs_ref, us_ref] = compute_steady_state_target(mpc, ref, odom)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Compute alpha based on the difference, ensuring it is between 0 and 1
            gain = 2;
            diff = abs(odom(1) - ref);
            alpha = 1 / (1 + gain * diff); % Sigmoid-like mapping to [0, 1]
            if alpha > 0.80
                alpha = 1.0;
            end
            ref = alpha * ref + (1-alpha) * odom(1);
            % disp('Lat alpha:')
            % disp(alpha)

            % Steady-state system
            A = mpc.A; %2x2
            B = mpc.B;
            nx = size(A,1);
            nu = size(B,2); 
            % Linearization steady-state
            xs = mpc.xs;
            us = mpc.us;
            f_xs_us = mpc.f_xs_us;
            C = mpc.C;
            u = sdpvar(nu,1);
            x = sdpvar(nx,1);

            xs_ref = [0; 0];
            us_ref = 0;
            umax = Car.ubu(1);
            umin = Car.lbu(1);
            xmin = Car.lbx(2:3);
            xmax = Car.ubx(2:3);
            constraints = [umin <= u <= umax ,...
                           x    == f_xs_us + A*(x-xs) + B*(u-us) ,...
                           ref  == x(1)                           ,...
                           x(2) == xs(2)                           ,...
                           xmin <= x <= xmax      ];
            
            objective   = u^2;
            diagnostics = solvesdp(constraints,objective,sdpsettings('verbose',0));
            
            if diagnostics.problem == 0
               % Good! 
            elseif diagnostics.problem == 1
                throw(MException('','Infeasible'));
            else
                throw(MException('','Something else happened'));
            end
            
            xs_ref = double(x);
            us_ref = double(u);
        end
    end
end
