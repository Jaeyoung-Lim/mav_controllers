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
model = MODEL_2D_Multirotor(); % Generic 2D Multirotor Model
% model = MODEL_2D_Multirotor_Tethered(); % Tethered 2D Multirotor Model

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

