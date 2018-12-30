classdef MODEL_2D_Multirotor_Tethered
    properties
        q;
        pos;
        vel;
        l; % Tether length
        theta; % Tether angle
        origin;
        slack;
        % Constants
        m;
        tether_length;
    end
    methods
        function obj = MODEL_2D_Multirotor_Tethered()
            obj.m = 1.0;
            obj.q = 0.0;
        
            obj.pos = [0.0, 1.0];
            obj.vel = [1.0, 0.0];
            
            obj.l = 1.0; % Tether length
            obj.origin = [0.0, 0.0];
            obj.theta = 0.0; % Tether angle

            obj.slack = true;
            obj.tether_length = 1.2;

            
        end
        
        function obj = step(obj, w, thrust, dt)
            % Parse states
            [position, attitude, velocity, l, ~, slack] = getstate(obj);
            
            % Calculate dynamics
            tether_vec = position - obj.origin;
            theta = atan2(tether_vec(2), tether_vec(1));
            tension_vec = tether_vec * (-l);
            
            if obj.slack
                acc = thrust/obj.m  * [cos(attitude + pi()/2), sin(attitude + pi()/2)] + [0.0, -9.8];
                vel = velocity + acc * dt;
                pos = position + velocity * dt + 0.5*acc*dt^2;
                q = attitude + w * dt;
            else
                acc = thrust/obj.m  * [cos(attitude + pi()/2), sin(attitude + pi()/2)] + [0.0, -9.8];
                acc = acc - dot(acc, tether_vec);
                vel = velocity + acc * dt;
                vel = vel - dot(vel, tether_vec);
                pos = position + velocity * dt + 0.5*acc*dt^2;
                pos = pos * obj.tether_length / norm(pos);
                q = attitude + w * dt;

            end
            
            theta = atan2(-tension_vec(2), -tension_vec(1));
            l = norm(pos - obj.origin);
            
            if l >= obj.tether_length
                slack = false;
                pos = pos * obj.tether_length / norm(pos);

            else
                slack = true;
            end
            
            % Encode states
            obj.pos = pos;
            obj.q = q;
            obj.vel = vel;
            obj.l = l;
            obj.theta = theta;
            obj.slack = slack;
        end
        
        function [pos, q, vel, l, theta, slack] = getstate(obj)
            pos = obj.pos;
            q = obj.q;
            vel = obj.vel;
            l = obj.l;
            theta = obj.theta;
            slack = obj.slack;
            
        end
        function visualize(obj)
            [position, attitude, ~] = getstate(obj);
            arm_length = 0.5;
            rotor1 = position + arm_length * [cos(attitude), sin(attitude)];
            rotor2 = position - arm_length * [cos(attitude), sin(attitude)];
            
            fuselarge = [rotor1; rotor2];
            
            %% Visualize vehicle
            title('Tethered Multrirotor');
            plot(position(1), position(2), 'kx'); hold on;
            plot(fuselarge(:, 1), fuselarge(:, 2), 'k-'); hold on;
            tether = [obj.pos; obj.origin];
            plot(tether(:, 1), tether(:, 2), 'k-'); hold on;
            
            xlim([-1.0, 1.0]); ylim([0, 2.0]); hold off;

        end
    end
end