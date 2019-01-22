classdef MODEL_2D_Multirotor_BallJuggling
    properties
        q;
        pos;
        vel;
        l; % Tether length
        load_pos;
        load_vel;
        load_acc;
        thrust;
        % Constants
        m;
        m_ball;
        quad_width;
        radius;
        e; % Restitution
    end
    methods
        function obj = MODEL_2D_Multirotor_BallJuggling()
            obj.m = 1.0;
            obj.m_ball = 0.1;
            obj.q = 0.0;
        
            obj.pos = [0.0, 1.0];
            obj.vel = [1.0, 0.0];            
            obj.load_pos = [0.0, 1.5];
            obj.load_vel = [0.5, 0.5];
            obj.load_acc = [0.0, 0.0];
            obj.thrust = 0.0;
            
            obj.quad_width = 0.5;
            obj.radius = 0.01;
            obj.e = 1.0;
        end
        
        function obj = step(obj, w, thrust, dt)
            % Parse states
            [position, attitude, velocity, load_position, load_velocity, load_acceleration, ~] = getstate(obj);
            g = [0.0, -9.8];
            
            F = [0.0, 0.0];
            
            if iscollision(position, attitude, load_position, obj.quad_width, obj.radius)
                if dot(load_velocity, velocity) < 0
                    F = 1.1*obj.m * obj.m_ball*(1 + obj.e) * (velocity - load_velocity)/(dt* (obj.m + obj.m_ball));
                end
            end

            % Load dynamics
            load_acceleration = g + F/obj.m_ball;
            load_velocity1 = load_velocity + load_acceleration * dt;
            load_position = load_position + load_velocity * dt + 0.5 * load_acceleration * dt^2;

            % Quadrotor dynamics
            acc = thrust/obj.m  * [cos(attitude + pi()/2), sin(attitude + pi()/2)] + g;
            velocity1 = velocity + acc * dt;
            position = position + velocity * dt + 0.5*acc*dt^2;
            attitude = attitude + w * dt;
            
            % Encode states
            obj.thrust = thrust;
            obj.pos = position;
            obj.q = attitude;
            obj.vel = velocity1;
            obj.load_pos = load_position;
            obj.load_vel = load_velocity1;
            obj.load_acc = load_acceleration;
            
        end        
        function [pos, q, vel, load_pos, load_vel, load_acc, thrust] = getstate(obj)
            thrust = obj.thrust;
            pos = obj.pos;
            q = obj.q;
            vel = obj.vel;
            load_pos = obj.load_pos;
            load_vel = obj.load_vel;
            load_acc = obj.load_acc;
            
        end
        function visualize(obj)
            [position, attitude, velocity, load_position, load_velocity, load_acceleration, slack] = getstate(obj);
            arm_length = 0.5;
            rotor1 = position + arm_length * [cos(attitude), sin(attitude)];
            rotor2 = position - arm_length * [cos(attitude), sin(attitude)];
            
            fuselarge = [rotor1; rotor2];
            
            %% Visualize vehicle
            title('Tethered Multrirotor');
            plot(position(1), position(2), 'kx'); hold on;
            plot(fuselarge(:, 1), fuselarge(:, 2), 'k-'); hold on;
            tether = [obj.pos; obj.load_pos];
%             plot(tether(:, 1), tether(:, 2), 'k-'); hold on;
            plot(obj.load_pos(1), obj.load_pos(2), 'ko-');
            
            xlim([-1.0, 1.0]); ylim([0.0, 2.0]); hold off;

        end
    end
end
