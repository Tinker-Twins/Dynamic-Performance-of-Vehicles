%% Clear Workspace
close all;
clear;
clc;

%% Given Data
l = 10/3.281; % Wheelbase (m)
a = 0.6*l; % Length of CG from front axle (m) for 40% front load
b = 0.4*l; % Length of CG from rear axle (m) for 40% front load
v = 66/3.281; % Velocity (m/s)
d = deg2rad(-40); % Steering wheel angle (rad)
G = 16; % Steering ratio
M = 3200*1.3558; % Stabilizing moment (Nm)
r = deg2rad(-12); % Yaw rate (rad/s)

%% 1.5
R = abs(l/tan(d/G));
fprintf('Ans 1.5: The path radius (R) is %f m\n\n',R)

%% 1.3
Nr = M/r;
fprintf('Ans 1.3: The yaw damping derivative (Nr) is %f Nms/rad\n\n',Nr)

%% 1.4
C = [a,-b;(a^2/v),(b^2/v)]\[0;Nr];
Cf = C(1);
Cr = C(2);
fprintf('Ans 1.4: The front (Cf) and rear (Cr) cornering stiffnesses are %f and %f N/rad\n\n',Cf,Cr)

%% 1.2
Nd = -a*Cf;
fprintf('Ans 1.2: The control moment derivative (Nd) is %f Nm/rad\n\n',Nd)

%% 1.1
NB = a*Cf - b*Cr;
fprintf('Ans 1.1: The static stability derivative (NB) is %f Nm/rad\n\n',NB)