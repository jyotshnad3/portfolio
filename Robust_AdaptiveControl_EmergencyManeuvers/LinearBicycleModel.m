classdef LinearBicycleModel < handle
    % LINEARBICYCLEMODEL Linearized Vehicle Model for Lateral Dynamics
    % Standard folder version (no package)
    
    properties
        m, I_z, a, b, u, C_af_nom, C_ar_nom
    end

    methods
        function obj = LinearBicycleModel(params)
            obj.m = params.m;
            obj.I_z = params.I_z;
            obj.a = params.a;
            obj.b = params.b;
            obj.u = params.u;
            obj.C_af_nom = params.C_af;
            obj.C_ar_nom = params.C_ar;
        end

        function [A, B] = get_matrices(obj, mu)
            % mu: Can be a scalar (applied to both) or vector [mu_f, mu_r]
            if nargin < 2, mu = [1.0, 1.0]; end
            
            if length(mu) == 1
                mu_f = mu;
                mu_r = mu;
            else
                mu_f = mu(1);
                mu_r = mu(2);
            end
            
            % Apply specific scaling to Front and Rear stiffness
            C_af = obj.C_af_nom * mu_f;
            C_ar = obj.C_ar_nom * mu_r;
            
            % Unpack for readability
            m_val = obj.m; u_val = obj.u; Iz_val = obj.I_z;
            a_val = obj.a; b_val = obj.b;
            %lateral displacement, lateral velocity, yaw angle, yaw rate
            A = zeros(4,4);
            A(1,1)= -0.5;
            A(3,3)=-0.5;
            A(1, 2) = 1;
            A(2, 2) = -(C_af + C_ar) / (m_val * u_val);
            A(2, 3) = (C_af + C_ar) / m_val;
            A(2, 4) = (b_val * C_ar - a_val * C_af) / (m_val * u_val);
            A(3, 4) = 1;
            A(4, 2) = (b_val * C_ar - a_val * C_af) / (Iz_val * u_val);
            A(4, 3) = (a_val * C_af - b_val * C_ar) / Iz_val;
            A(4, 4) = -(a_val^2 * C_af + b_val^2 * C_ar) / (Iz_val * u_val);

            B = zeros(4,1);
            B(2) = C_af / m_val;
            B(4) = (a_val * C_af) / Iz_val;
        end

        function x_dot = get_dynamics(obj, x, delta, mu)
            % GET_DYNAMICS Compute x_dot = Ax + Bu
            [A, B] = obj.get_matrices(mu);
            x_dot = A * x + B * delta;
        end

        function x_next = step(obj, x, delta, dt, mu)
            % STEP Runge-Kutta 4th Order (RK4) Integration
            
            k1 = obj.get_dynamics(x, delta, mu);
            k2 = obj.get_dynamics(x + 0.5*dt*k1, delta, mu);
            k3 = obj.get_dynamics(x + 0.5*dt*k2, delta, mu);
            k4 = obj.get_dynamics(x + dt*k3, delta, mu);
            
            x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
        end
    end
end