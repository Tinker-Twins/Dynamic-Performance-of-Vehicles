%% Clear Workspace
close all;
clear;
clc;

%% 4A.A
W = 143*14.594*9.81; % Total vehicle weight
WFront = 0.54; % Front weight ratio
WRear = 1-WFront; % Rear weight ratio
WLeft = 0.52; % Left weight ratio
WRight = 1-WLeft; % Right weight ratio
D = 500; % Diagonal weight (N)
syms LF RF LR RR
assume(LF>=0 & RF>=0 & LR>=0 & RR>=0); % Assume all static loads are positive
equation1 = LF+RF+LR+RR-W; % W = LF+RF+LR+RR 
equation2 = LF+RF-(W*WFront); % WFront = ((LF+RF)/W)
equation3 = RF+RR-(W*WRight); % WRight = ((RF+RR)/W)
equation4 = (LF+RR)-(RF+LR)-D; % D = (LF+RR)-(RF+LR)
solution = solve(equation1,equation2,equation3,equation4); % Solve 4 equations simultaneously
LF = double(solution.LF); % Left front wheel load (N)
RF = double(solution.RF); % Right front wheel load (N)
LR = double(solution.LR); % Left rear wheel load (N)
RR = double(solution.RR); % Right rear wheel load (N)
fprintf('Ans 4A.A: The static wheel loads for given vehicle parameters are LF = %.2f N, RF = %.2f N, LR = %.2f N, RR = %.2f N\n\n',LF,RF,LR,RR)

%% 4A.B
% dWx = (m*ax*h)/l = (W*Ax*h)/l
fprintf('Ans 4A.B: Expression "dWx = (m*ax*h)/l = (W*Ax*h)/l" gives longitudinal load transfer while acceleration/braking\n\n')

%% 4A.C
m = 143*14.594; % Vehicle mass (kg)
ax = 0.5*9.81; % Longitudinal (braking) acceleration (m/s^2)
h = 2.3/3.281; % Height of CG (m)
l = 9.45/3.281; % Wheelbase (m)
dWx = (m*ax*h)/l; % Longitudinal load transfer
fprintf('Ans 4A.C: The load transferred from the rear tires to the front tires when the vehicle is braking at 0.5g is %f N\n\n',dWx)

%% 4A.D
l = 9.45/3.281; % Wheelbase (m)
g = 9.81; % Acceleration due to gravity (m/s^2)
a = (1-0.54)*l; % Distance of CG from front axle (m)
h = 2.3/3.281; % Height of CG (m)
ax = (g*a)/h;
fprintf('Ans 4A.D: At %f m/s^2 longitudinal acceleration all the load will be transferred to front from rear in a straight-line motion\n\n',ax)