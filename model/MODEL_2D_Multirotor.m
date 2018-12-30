classdef MODEL_2D_Multirotor
    properties
        q;
        pos;
        vel;
        m;
    end
    methods
        function obj = MODEL_2D_Multirotor()
            obj.m = 1.0;
            obj.q = -0.8;
        
            obj.pos = [-0.5, 1.0];
            obj.vel = [1.0, 0.0];
            
        end
        
        function obj = step(obj, w, thrust, dt)
            % Parse states
            [position, attitude, velocity] = getstate(obj);
            
            % Calculate dynamics
            acc = thrust/obj.m  * [cos(attitude + pi()/2), sin(attitude + pi()/2)] + [0.0, -9.8];
            vel = velocity + acc * dt;
            pos = position + velocity * dt + 0.5*acc*dt^2;
            q = attitude + w * dt;
            
            % Encode states
            obj.pos = pos;
            obj.q = q;
            obj.vel = vel;
        end
        
        function [pos, q, vel] = getstate(obj)
            pos = obj.pos;
            q = obj.q;
            vel = obj.vel;
            
        end
        function visualize(obj)
            [position, attitude, ~] = getstate(obj);
            arm_length = 0.5;
            rotor1 = position + arm_length * [cos(attitude), sin(attitude)];
            rotor2 = position - arm_length * [cos(attitude), sin(attitude)];
            
            fuselarge = [rotor1; rotor2];
            plot(position(1), position(2), 'kx'); hold on;
            plot(fuselarge(:, 1), fuselarge(:, 2), 'k-'); 
            xlim([-1.0, 1.0]); ylim([0, 2.0]); hold off;

        end
    end
end