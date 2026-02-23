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
            % Loading
            data = load('tube_mpc_data.mat');
            A = data.A;
            B = data.B_input;
            x_con = data.X_tilde; H = x_con.A;  h = x_con.b; 
            u_con = data.U_tilde; F = u_con.A;  f = u_con.b;
            P = data.P; Xf = data.Xf; Ff = Xf.A; ff = Xf.b;
            K = data.K;
            E = data.E; He = E.A;  he = E.b; 
            [nx, nu] = size(B);
            % Targets
            X_ref = sdpvar(nx,1);
            u_ref = sdpvar(nu,1);
            X = sdpvar(nx, N); % State trajectory
            V = sdpvar(nu, N); % Nominal Input trajectory
            z = sdpvar(nx, N); % Delata trajectory

            % Initial states
            x0 = X(:, 1); %2x1
            x0other = sdpvar(nx, 1);
            x_safe = [10; 0];


            

            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            Qx = 0.1; % Position cost
            Qv = 0.1; % Velocity cost
            R = 100;
            
            % Initial 'z0'==deltax(:, 1); 'x0'= x0other - x0 - x_safe
            con = [He * (x0other - x0 - x_safe - z(:, 1)) <= he];
            % Input to apply to the system
            u0 = K * (x0other - x0 - x_safe - z(:, 1)) + V(:, 1); %1x1
            for k = 1:N-1
                % Dynamics constraint
                con = [con, z(:, k+1) ==  A * z(:, k) + B * V(:, k)];


                % State constraints
                con = [con, H * z(:, k) <= h]; 
                % Input constraints
                con = [con, F * V(:, k) <= f]; 


                % Objective function
                obj = obj + z(1, k)' * Qx * z(1, k) + z(2, k)' * Qv * z(2, k) +...
                      (V(:, k+1) - V(:, k))' * R * (V(:, k+1) - V(:, k));
            end
            obj = obj + z(:, N)' * P * z(:, N);
            con = [con, Ff * z(:, N) <=  ff]; % Terminal constraint

            % Pass here YALMIP sdpvars which you want to debug. You can
            % then access them when calling your mpc controller like
            debugVars = {z, V};

            
            %gurobi
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, X_ref, u_ref, x0other}, {u0, debugVars{:}});
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
            gain = 1.0;
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
                            x(2)  ==f_xs_us(2) + A(2,2)*(x(2)-xs(2)) + B(2,:)*(u-us) ,...
                            ref == x(2)                           ,...
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

        end
    end
end
