function [w, thrust] = geometricCtrl2D(obj, ref)
    [position, attitude, velocity] = obj.getstate();
    
    error_pos = position - ref;
    error_vel = velocity - [0.0, 0.0];
    
    %% Calculate desired acceleration
    Kp = -5.0;
    Kv = -4.0;

    desired_acc = Kp * error_pos + Kv * error_vel + [0.0, 9.8];
    
    %% Calculate desired angular velocity
    tau = 0.1;
    
    desired_att = atan2(desired_acc(2), desired_acc(1)) - 0.5 * pi();
    
    error_att = desired_att - attitude;
    w = (1/tau) * error_att;
    thrust = obj.m * norm(desired_acc);

end