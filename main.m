%% Autonomous dynamic soaring simulator
% Written by: Jaeyoung Lim jalim@student.ethz.ch

%% Initialize
clc; clear all;
pause on
figure(1);
dt = 0.01;

ref = [0.0, 1.0];

%%
pos = [];
loop = 0;

% Initialize model
model = MODEL_2D_Multirotor();

while true
   [w, thrust] = geometricCtrl2D(model, ref);
   model = model.step(w, thrust, dt);
   loop = loop + 1;
    pos = model.getstate();
    if loop > 5
        loop = 0;
        model.visualize();
    end

    if islanded(pos)
       break; 
    end
    pause(dt);

end

function [w, thrust] = geometricCtrl2D(obj, ref)
    [position, attitude, velocity] = obj.getstate();
    
    error_pos = position - ref;
    error_vel = velocity - [0.0, 0.0];
    
    %% Calculate desired acceleration
    Kp = -2.0;
    Kv = -9.0;

    desired_acc = Kp * error_pos + Kv * error_vel + [0.0, 9.8];
    
    %% Calculate desired angular velocity
    tau = 0.1;
    
    desired_att = atan2(desired_acc(2), desired_acc(1)) - 0.5 * pi();
    
    error_att = desired_att - attitude;
    w = (1/tau) * error_att;
    thrust = obj.m * norm(desired_acc);

end