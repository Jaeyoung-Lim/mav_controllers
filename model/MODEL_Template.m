classdef MODEL_Template
    properties
        q = 0.0;
        w = 0.0;
        pos = [0.0, 0.0];
        vel = [0.0, 0.0];
        
        thrust = 0.0;
    end
    methods
        function state = step(obj)
            [gamma_a, psi_a, phi, pos, vel, v_a] = getstate(state);
             state = setstate(gamma_a, psi_a, phi, pos, vel, v_a);

        end
        function visualize()
            
        end
        
    end
end