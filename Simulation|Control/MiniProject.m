%% MiniProject.m
%
% In this matlab section we simulate a step response of our open loop
% system 
%

s = tf('s');

velocityTF = (22.89377288) / (s + 11.9047619);
positionTF = velocityTF/s;
step(velocityTF);
step(positionTF)