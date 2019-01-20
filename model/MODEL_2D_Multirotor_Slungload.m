classdef MODEL_2D_Multirotor_Slungload
    properties
        q;
        pos;
        vel;
        l; % Tether length
        load_pos;
        load_vel;
        load_acc;
        thrust
        slack;
        % Constants
        m;
        m_load;
        tether_length;
    end
    methods
        function obj = MODEL_2D_Multirotor_Slungload()
            obj.m = 1.0;
            obj.m_load = 0.1;
            obj.q = 0.0;
        
            obj.pos = [0.0, 1.0];
            obj.vel = [1.0, 0.0];
            
            obj.l = 0.6; % Tether length
            
            obj.load_pos = [0.0, 1.0];
            obj.load_vel = [-0.5, 0.5];
            obj.load_acc = [0.0, 0.0];
            obj.thrust = 0.0;
            
            obj.slack = true;
            obj.tether_length = 1.2;
            
        end
        
        function obj = step(obj, w, thrust, dt)
            % Parse states
            [position, attitude, velocity, load_position, load_velocity, load_acceleration, ~, slack] = getstate(obj);
            g = [0.0, -9.8];
            
            % Calculate dynamics
            tether_vec = load_position - position;
            unit_tether_vec = tether_vec / norm(tether_vec);
            if norm(tether_vec) >= obj.l
                % Thether is taut
                % Load dynamics
%                 load_acceleration = g - T / obj.m_load;
                load_acceleration = (1/(obj.m + obj.m_load)) * (dot(unit_tether_vec, thrust*[cos(attitude+ pi()/2), sin(attitude + pi()/2)]) - obj.m * obj.l * dot(load_velocity, load_velocity)) * unit_tether_vec + g;
                load_velocity1 = load_velocity + load_acceleration * dt;
                load_position = load_position + load_velocity * dt + 0.5 * load_acceleration * dt^2;
                
                T = obj.m_load * norm(-g + load_acceleration) * tether_vec / norm(tether_vec);

                slack = false;
                
                % Quadrotor dynamics
                acc = thrust/obj.m  * [cos(attitude + pi()/2), sin(attitude + pi()/2)] + g + T/obj.m;
                velocity1 = velocity + acc * dt;
                position = position + velocity * dt + 0.5*acc*dt^2;
                attitude = attitude + w * dt;
                
                % Enforce kinematic constraints
                load_direction = (load_position - position) / norm(load_position - position);
                load_position = position + load_direction * obj.l;
                load_velocity1 = load_velocity - dot(load_velocity1 - velocity1, load_direction) * load_direction


            else
                % Thether is slack
                T = [0.0, 0.0];
                slack = true;
                
                % Load dynamics
                load_acceleration = g;
                load_velocity1 = load_velocity + load_acceleration * dt;
                load_position = load_position + load_velocity * dt + 0.5 * load_acceleration * dt^2;

                % Quadrotor dynamics
                acc = thrust/obj.m  * [cos(attitude + pi()/2), sin(attitude + pi()/2)] + g;
                velocity1 = velocity + acc * dt;
                position = position + velocity * dt + 0.5*acc*dt^2;
                attitude = attitude + w * dt;
            end
            
            % Encode states
            obj.thrust = thrust;
            obj.pos = position;
            obj.q = attitude;
            obj.vel = velocity1;
            obj.load_pos = load_position;
            obj.load_vel = load_velocity1;
            obj.load_acc = load_acceleration;
            obj.slack = slack;
        end
        
        function [pos, q, vel, load_pos, load_vel, load_acc, thrust, slack] = getstate(obj)
            thrust = obj.thrust;
            pos = obj.pos;
            q = obj.q;
            vel = obj.vel;
            load_pos = obj.load_pos;
            load_vel = obj.load_vel;
            load_acc = obj.load_acc;
            slack = obj.slack;
            
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
            plot(tether(:, 1), tether(:, 2), 'k-'); hold on;
            plot(obj.load_pos(1), obj.load_pos(2), 'ko-');
            
            xlim([-1.0, 1.0]); ylim([0.0, 2.0]); hold off;

        end
    end
end