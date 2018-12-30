s%% Autonomous dynamic soaring simulator
% Written by: Jaeyoung Lim jalim@student.ethz.ch

%% Initialize
clc; clear all;

pos = [];
loop = 0;

w = 0.0;
thrust = 0.0;

model = MODEL_2D_Multirotor();

while true
    model.step(w, thrust);
    loop = loop + 1;
end