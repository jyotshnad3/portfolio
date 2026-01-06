classdef RoadProfile
    % ROADPROFILE Handles environmental conditions
    % Allows querying road parameters (friction/stiffness scaling) based on time.
    
    properties
        friction_drop_time = 5.0; % Time when road becomes slippery
        mu_dry = 1.0;
        mu_wet = 0.5;
        
        % Lag between front and rear axle hitting the new surface
        % Assuming 20m/s and ~3m wheelbase -> approx 0.15s lag
        axle_lag = 0.15; 
    end
    
    methods
        function mu_vector = get_coefficients(obj, t)
            % GET_COEFFICIENTS Returns [mu_f, mu_r]
            % This allows simulating front tires losing grip before rear tires.
            
            % 1. Front Axle Friction
            if t < obj.friction_drop_time
                mu_f = obj.mu_dry;
            else
                mu_f = obj.mu_wet;
            end
            
            % 2. Rear Axle Friction (hits the patch slightly later)
            if t < (obj.friction_drop_time + obj.axle_lag)
                mu_r = obj.mu_dry;
            else
                mu_r = obj.mu_wet;
            end
            
            mu_vector = [mu_f, mu_r];
        end
    end
end