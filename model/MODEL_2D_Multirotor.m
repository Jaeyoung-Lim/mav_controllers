classdef MODEL_2D_Multirotor
    properties
        q;
        pos;
        vel;
    end
    methods
        function obj = MODEL_2D_Multirotor()
            obj.q = 0.0;
        
            obj.pos = [0.0, 1.0];
            obj.vel = [0.0, 0.0];
            
        end
        
        function obj = step(obj, w, thrust, dt)
            % Parse states
            [position, attitude, velocity] = getstate(obj);
            
            % Calculate dynamics
            acc = thrust  * [cos(attitude + pi()/2), sin(attitude + pi()/2)] + [0.0, -9.8];
            vel = velocity + acc * dt;
            pos = position + velocity * dt + 0.5*acc*dt^2;
            q = attitude + w * dt;
            
            % Encode states
            obj.pos = pos;
            obj.q = q;
            obj.vel = vel;
        end
        
        function [pos, q, vel] = getstate(obj)
            pos = obj.pos
            q = obj.q;
            vel = obj.vel;
            
        end
        function visualize(obj)
            [position, ~, ~] = getstate(obj);
             plot(position(1), position(2), 'rx'); hold on;
             drawnow
            
        end
    end
end