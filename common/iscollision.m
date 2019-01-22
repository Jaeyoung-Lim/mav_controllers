function [collision] = iscollision(position, attitude, load_position, quad_width, radius)
    
    collision = false;
    
    ball_direction = load_position - position;
    thrust_direction = [cos(attitude + pi()/2), sin(attitude + pi()/2)];
    
    if dot(ball_direction, thrust_direction) >= 0
        if norm(ball_direction - dot(ball_direction, thrust_direction) * thrust_direction) < quad_width
            if dot(ball_direction, thrust_direction) <= radius
                dot(ball_direction, thrust_direction)
                collision = true;            
        
            end
        end
    end
end