classdef LonEstimator
    properties
        % continous sub-system
        sys
        % Extended linearization points
        xs_hat, us_hat, fd_xs_us, vs
        % Extended system matrices
        A_hat, B_hat, C_hat, A, B
        % Observer gain matrix
        L
    end
    
    methods
        % Setup the longitudinal disturbance estimator
        function est = LonEstimator(sys, Ts)

            xs = sys.UserData.xs;
            us = sys.UserData.us;

            
            % Discretize the system and extract the A,B,C,D matrices
            [fd_xs_us, Ad, Bd, ~, ~] = Car.c2d_with_offset(sys, Ts);
            

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            % Extended state dynamics
            [nx, nu] = size(Bd);
            est.xs_hat = xs;
            est.us_hat = us;
            est.fd_xs_us = fd_xs_us;
            est.A_hat = [Ad(2, 2), Bd(2); 0, eye(nu)];
            est.B_hat = [Bd(2); 0];
            est.C_hat = [1, 0];
            est.A = Ad;
            est.B = Bd;
            est.vs = fd_xs_us(2) - Ad(2, :) * xs - Bd(2, :) * us;


            poles = [0.5, 0.6];
            est.L = -place(est.A_hat', est.C_hat', poles')';
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        % This function takes in the the estimate, input, and measurement
        % at timestep i and predicts the next (extended) state estimate at
        % the next timestep i + 1.
        function z_hat_next = estimate(est, z_hat, u, y)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   z_hat      - current estimate (V, dist)
            %   u          - longitudinal input (u_T)
            %   y          - longitudinal measurement (V)
            % OUTPUTS
            %   z_hat_next - next time step estimate
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % Estimation equation
            z_hat_next = est.A_hat * z_hat + est.B_hat * u + est.L * (est.C_hat * z_hat - y) + [est.vs; 0];
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
