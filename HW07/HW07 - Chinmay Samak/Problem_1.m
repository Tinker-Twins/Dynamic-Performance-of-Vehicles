%% Clear Workspace

close all;
clear;
clc;

%% Given Data

l = 9/3.281; % m
t = 4.5/3.281; % m
LF = 850*4.4482189159; % N
RF = 825*4.4482189159; % N
LR = 725*4.4482189159; % N
RR = 750*4.4482189159; % N

%% Problem 1A

W = LF+RF+LR+RR; % N
Xcg = l*((LF+RF)/W); % m
Ycg = t*((RF+RR)/W); % m
fprintf('1A: The vehicle''s CG is at (%f m , %f m) as measured from the rear axle and left side\n\n',Xcg,Ycg)

%% Problem 1B

D = (LF+RR)-(RF+LR); % N
fprintf('1B: The vehicle''s diagonal weight (a.k.a. crossweight or wedge) is %f N\n\n',D)