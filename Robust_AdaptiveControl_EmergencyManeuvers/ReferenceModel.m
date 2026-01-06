% classdef ReferenceModel < handle
%     % REFERENCEMODEL The ideal system behavior for MRAC tracking.
%     % Uses fixed nominal parameters.
% 
%     properties
%         A_m % Reference System Matrix
%         B_m % Reference Input Matrix
%     end
% 
%     methods
%         function obj = ReferenceModel(params)
%             % Initialize with NOMINAL parameters (mu=1.0 always)
%             temp_plant = LinearBicycleModel(params);
%             [obj.A_m, obj.B_m] = temp_plant.get_matrices(1.0); 
%         end
% 
%         function x_dot = get_dynamics(obj, x, r_cmd)
%             % Helper for RK4
%             x_dot = obj.A_m * x + obj.B_m * r_cmd;
%         end
% 
%         function x_next = step(obj, x, r_cmd, dt)
%             % STEP Runge-Kutta 4th Order (RK4) Integration
%             % Using the same solver as the plant ensures fair comparison.
% 
%             k1 = obj.get_dynamics(x, r_cmd);
%             k2 = obj.get_dynamics(x + 0.5*dt*k1, r_cmd);
%             k3 = obj.get_dynamics(x + 0.5*dt*k2, r_cmd);
%             k4 = obj.get_dynamics(x + dt*k3, r_cmd);
% 
%             x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
%         end
%     end
% end

classdef ReferenceModel < handle
    % REFERENCEMODEL The ideal system behavior for MRAC tracking.
    % Defines a stable, second-order response with relative degree 2.

    properties
        A_m % Reference System Matrix (4x4)
        B_m % Reference Input Matrix (4x1)
    end

    methods
        function obj = ReferenceModel(~)
            % Inputs: (~) - We ignore physical parameters now because 
            % the reference model represents "Desired Behavior," not physics.

            % --- TUNING PARAMETERS ---
            tau = 0.1;          % Time constant (seconds)
            zeta = 1.0;         % Damping ratio (1.0 = Critical, No Overshoot)

            % Natural frequency derived from time constant
            wn = 1 / (tau * zeta); % wn = 10 rad/s

            % --- STATE DEFINITION ASSUMPTION ---
            % Assuming state vector x = [e_y; dy; e_psi; dpsi]
            % 1. Lateral Error
            % 2. Lateral Error Rate
            % 3. Heading Error
            % 4. Heading Error Rate

            % --- MATRIX CONSTRUCTION (Canonical Form) ---
            % We decouple the lateral and heading dynamics for the reference.
            % We want e_y to track r_cmd, and e_psi to regulate to 0.

            % Row 1: Kinematics (d(e_y)/dt = dy)
            % Row 2: Dynamics (d(dy)/dt = -wn^2*e_y - 2*zeta*wn*dy + wn^2*r)
            % Row 3: Kinematics (d(e_psi)/dt = dpsi)
            % Row 4: Dynamics (d(dpsi)/dt = -wn^2*e_psi - 2*zeta*wn*dpsi)

            obj.A_m = [
                0,      1,              0,          0;
                -wn^2,  -2*zeta*wn,     0,          0;
                0,      0,              0,          1;
                0,      0,              -wn^2,      -2*zeta*wn
            ];

            % B_m Matrix (Relative Degree 2)
            % The input enters the 'acceleration' (rate of rate) channel.
            % To ensure Unit Gain (Steady State Error = 0), gain is wn^2.
            obj.B_m = [
                0; 
                wn^2; 
                0; 
                0
            ];
        end

        function x_dot = get_dynamics(obj, x, r_cmd)
            % Helper for RK4
            x_dot = obj.A_m * x + obj.B_m * r_cmd;
        end

        function x_next = step(obj, x, r_cmd, dt)
            % STEP Runge-Kutta 4th Order (RK4) Integration
            k1 = obj.get_dynamics(x, r_cmd);
            k2 = obj.get_dynamics(x + 0.5*dt*k1, r_cmd);
            k3 = obj.get_dynamics(x + 0.5*dt*k2, r_cmd);
            k4 = obj.get_dynamics(x + dt*k3, r_cmd);

            x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
        end
    end
end




