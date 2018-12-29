s%% Autonomous dynamic soaring simulator
% Written by: Jaeyoung Lim jalim@student.ethz.ch

%% Initialize
clc; clear all;


state = struct('q', [0.0, 0.0, 0.0, 0.0] , ...
               'w', [0.0, 0.0, 0.0], ...
               'pos', [0.0, 0.0, -1.0], ...
               'vel', [10.0, 0.0, 0.0]);

pos = [];
loop = 0;

while true
    state = step(state, roll_rate, pitch_rate, Jw);
    
    [~, ~, ~, pos1] = getstate(state);
    pos =[pos; pos1];
    figure(1);
    
    plot3(pos(:, 1), pos(:, 2), -pos(:, 3), 'r-');hold on; axis equal; grid on; %zlim([-1, 3]);
    xlabel('x Position [m]'); ylabel('y Position [m]'); zlabel('z Position [m]');
    if islanded(state)
       break; 
    end
    loop = loop + 1;
end