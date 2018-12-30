%% Autonomous dynamic soaring simulator
% Written by: Jaeyoung Lim jalim@student.ethz.ch

%% Initialize
clc; clear all;

pos = [];
loop = 0;

w = 0.1;
thrust = 1.0;

dt = 0.01;

model = MODEL_2D_Multirotor();

figure(1);
while true

    model = model.step(w, thrust, dt);
    model.visualize();
    loop = loop + 1;
    pos = model.getstate();
    if islanded(pos)
       break; 
    end
end