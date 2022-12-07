%% Clear Workspace

close all;
clear;
clc;

%% Given Data

W = 20e3; % N
t = 1.6; % m

%% Problem 2A

ay = 0.9*9.81; % m/s^2
h = (9.81*t)/(2*ay);
fprintf('2A: The height of CG must be %f m to cause untripped rollover of the vehicle at 0.9g lateral acceleration\n\n',h)

%% Problem 2B

Wy = (W*(ay/9.81)*(h/2))/t;
fprintf('2B: Total load transfer in vehicle when CG height is half of the rollover height will be %.2f N\n\n',Wy)

%% Problem 2C

Krf_aux_factor = 0.5*1.25*t^2 - 0.5*t^2;
fprintf('2C: Since the lateral load transfer is to be evenly distributed between front and rear, and since suspension\nsprings in rear are 25%% (1.25 times) stiffer than ones in the front, anti-roll bar needs to be added in the\nfront (to compensate for less stiff springs) with torsional stiffness of %.2f times front spring stiffness (Kf)\n\n',Krf_aux_factor)

%% Problem 2D

l = 3; % m
WFront = 0.58;
WRear = 1-WFront;
WLeft = 0.52;
WRight = 1-WLeft;
D = 25; % N

Xcg = l*WFront;
Ycg = t*WRight;

syms LF RF LR RR
assume(LF>=0 & RF>=0 & LR>=0 & RR>=0); % Assume all static loads are positive
equation1 = LF+RF+LR+RR-W; % W = LF+RF+LR+RR 
equation2 = LF+RF-(W*Xcg/l); % Xcg = l*((LF+RF)/W)
equation3 = RF+RR-(W*Ycg/t); % Ycg = t*((RF+RR)/W)
equation4 = (LF+RR)-(RF+LR)-D; % D = (LF+RR)-(RF+LR)
solution = solve(equation1,equation2,equation3,equation4);

LF = double(solution.LF); % N
RF = double(solution.RF); % N
LR = double(solution.LR); % N
RR = double(solution.RR); % N

fprintf('2D: The static wheel loads for given vehicle configuration are LF = %.2f N, RF = %.2f N, LR = %.2f N, RR = %.2f N\n\n',LF,RF,LR,RR)