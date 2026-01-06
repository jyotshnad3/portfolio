classdef BaselineController < handle
    % BASELINECONTROLLER Simple open-loop or PID controller
    % Currently configured to output a constant value for testing.
    
    properties
        const_output = 0.0; 
        Kp = 0.01; % Example Proportional Gain (unused if using const_output)
    end
    
    methods
        function u = compute(obj, y_ref, y_meas)
            % COMPUTE Calculate control input
            % y_ref: Target deviation
            % y_meas: Actual deviation
            
            % For now, return constant per user request
            % u = obj.const_output;
            
            % Uncomment below for simple P-Control:
            error = y_ref - y_meas;
            u = error * obj.Kp; 
        end
    end
end