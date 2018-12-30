classdef MODEL_2D_Multirotor
    properties
        q;
        pos;
        vel;
    end
    methods
        function obj = MODEL_2D_Multirotor()
            obj.q = 0.0;
        
            obj.pos = [0.0, 0.0];
            obj.vel = [0.0, 0.0];
            
        end
        
        function step(obj, w, thrust, dt)
            % Parse states
            [position, attitude, velocity] = getstate(obj);
            
            % Calculate dynamics
            acc = thrust  * [cos(attitude + pi()/2), sin(attitude + pi()/2)];
            velocity = velocity + acc * dt;
            position = position + velocity * dt + 0.5*acc*dt^2;
            attitude = attitude + w * dt;
            
            % Encode states
            setstate(obj, position, attitude, velocity)
        end
        
        function [pos, q, vel] = getstate(obj)
            pos = obj.pos;
            q = obj.q;
            vel = obj.vel;
            
        end
        
        function obj = setstate(obj, pos, q, vel)
            obj.pos = pos;
            obj.q = q;
            obj.vel = vel;
            
        end
    end
end