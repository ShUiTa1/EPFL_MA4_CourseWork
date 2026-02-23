classdef NmpcControl < handle

    properties
        % The NMPC problem
        opti

        % Problem parameters
        x0, ref, x0other, X, U

        % Most recent problem solution
        sol

        % The input that you want to apply to the system
        u0

        % Dynamic model
        f, f_discrete_ode

        % Constraints
        ubu, ubx, lbu, lbx

        % Cost
        Q, R

        car
        
    end

    methods
        function obj = NmpcControl(car, H)

            import casadi.*

            N_segs = ceil(H/car.Ts); % Horizon steps
            N = N_segs + 1;          % Last index in 1-based Matlab indexing

            nx = 4;
            nu = 2;
            obj.car = car;

            % Define the NMPC optimization problem
            opti = casadi.Opti();
            
            % Parameters (symbolic)
            obj.x0 = opti.parameter(nx, 1);       % initial state
            obj.ref = opti.parameter(2, 1);       % target y, velocity
            obj.x0other = opti.parameter(nx, 1);  % initial state of other car
            obj.X = opti.variable(nx, N);    % state trajectory variables
            obj.U = opti.variable(nu, N-1);  % control trajectory (delta, throttle)
            % SET THIS VALUE TO BE YOUR CONTROL INPUT
            obj.u0 = opti.variable(nu, 1);

            % Define dynamic model (symbolic using CasADi)
            x = SX.sym('x', nx, 1);
            u = SX.sym('u', nu, 1);
          
            beta = atan((car.lr * tan(u(1))) / (car.lr + car.lf));
            F_motor = u(2) * car.Pmax / max(x(4), 1);
            F_drag = 0.5 * car.rho * car.Cd * car.Af * x(4)^2;
            F_roll = car.Cr * car.mass * car.g;

            f_dyn = [
                x(4) * cos(x(3) + beta);   % dx/dt
                x(4) * sin(x(3) + beta);   % dy/dt
                x(4) * sin(beta) / car.lr; % dθ/dt
                (F_motor - F_drag - F_roll) / car.mass % dV/dt
            ];
        
            % Continuous-time dynamics as a CasADi function
            obj.f = Function('f', {x, u}, {f_dyn});


            % Discretize dynamics using RK4
            dt = car.Ts;
            k1 = obj.f(x, u);
            k2 = obj.f(x + dt/2 * k1, u);
            k3 = obj.f(x + dt/2 * k2, u);
            k4 = obj.f(x + dt * k3, u);
        
            x_next = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
        
            % Define discrete-time dynamics as a CasADi function
            obj.f_discrete_ode = Function('f_discrete_ode', {x, u}, {x_next});

            % Constraints bound
            obj.ubu = car.ubu;
            obj.lbu = car.lbu;
            obj.ubx = car.ubx;
            obj.lbx = car.lbx;

            % Define your problem using the opti object created above
            cost = 0;
            obj.Q = 1;
            obj.R = 1;

            % Constraints
            opti.subject_to(obj.X(:,1) == obj.x0);
            opti.subject_to(obj.U(:,1) == obj.u0);
            for k=1:N-1 % loop over control intervals
              % Dynamic
              opti.subject_to(obj.X(:,k+1) == obj.f_discrete_ode(obj.X(:,k),obj.U(:,k)));

              % Constraint on U
              opti.subject_to(obj.U(:,k) <= obj.ubu);
              opti.subject_to(obj.U(:,k) >= obj.lbu);

              % Constraint on X
              opti.subject_to(obj.X(:,k) <= obj.ubx);
              opti.subject_to(obj.X(:,k) >= obj.lbx);

              % Cost
              y_cost = (obj.X(2,k) - obj.ref(1)) * obj.Q * (obj.X(2,k) - obj.ref(1));
              v_cost = (obj.X(4,k) - obj.ref(2)) * obj.Q * (obj.X(4,k) - obj.ref(2));
              u_cost = obj.U(:, k)' * obj.R * obj.U(:, k);
              stage_cost = y_cost + v_cost + u_cost;
              cost = cost + stage_cost;
              
            end

            % Constraint on X
            opti.subject_to(obj.X(:,N) <= obj.ubx);
            opti.subject_to(obj.X(:,N) >= obj.lbx);
            y_cost = (obj.X(2,N) - obj.ref(1)) * obj.Q * (obj.X(2,N) - obj.ref(1));
            v_cost = (obj.X(4,N) - obj.ref(2)) * obj.Q * (obj.X(4,N) - obj.ref(2));
            stage_cost = y_cost + v_cost;
            cost = cost + stage_cost;

            opti.minimize(cost);


            % Store the defined problem to solve in get_u
            obj.opti = opti;

            % Setup solver
            options = struct;
            options.ipopt.print_level = 0;
            options.print_time = 0;
            options.expand = true;
            obj.opti.solver('ipopt', options);
        end

        function u = get_u(obj, x0, ref, x0other)

            if nargin < 4
                x0other = zeros(4, 1);
            end

            % Compute solution from x0
            obj.solve(x0(1:4), ref, x0other(1:4));

            u = obj.sol.value(obj.u0);
        end

        function solve(obj, x0, ref, x0other)


            % Pass parameter values
            obj.opti.set_value(obj.x0, x0);
            obj.opti.set_value(obj.ref, ref);
            obj.opti.set_value(obj.x0other, x0other);

            obj.sol = obj.opti.solve();   % actual solve
            
            % Set warm start for next solve
            obj.opti.set_initial(obj.sol.value_variables());
            obj.opti.set_initial(obj.opti.lam_g, obj.sol.value(obj.opti.lam_g));
        end
    end
end
