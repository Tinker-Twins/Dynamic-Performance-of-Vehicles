%% Clear Workspace
close all;
clear;
clc;

%% Given Data
w = 4e4*4.448; % Weight (N)
m = w/9.81; % Mass (kg)
l = 15/3.281; % Wheelbase (m)
t = 9/3.281; % Trackwidth (m)
h = 4.7/3.281; % Height of CG (m)
a = 0.6*l; % Length of CG from front axle (m) for 40% front load
b = 0.4*l; % Length of CG from rear axle (m) for 40% front load
d = 4/3.281; % Length of front axle from snowplow (m)
v = 50/2.237; % Velocity (m/s)
R = 500/3.281; % Turning radius
Fs = 1000*4.448; % Snow force (N)

%% 3B.A
dA = atan2(l,R);
fprintf('Ans 3B.A: The Ackerman steering angle (dA) is %f rad\n\n',dA)

%% 3B.B
ay = v^2/R;
fprintf('Ans 3B.B: The lateral acceleration (ay) is %f m/s^2\n\n',ay)

%% 3B.C
r = v/R;
fprintf('Ans 3B.C: The yaw rate (r) is %f rad/s\n\n',r)

%% 3B.D
Fy = m*ay;
Fyfr = [1,1;-a,b]\[(Fs+Fy);-Fs*(a+d)];
Fyf = Fyfr(1);
fprintf('Ans 3B.D: The front lateral force (Fyf) is %f N\n\n',Fyf)

%% 3B.E
Fyr = Fyfr(2);
fprintf('Ans 3B.E: The rear lateral force (Fyr) is %f N\n\n',Fyr)

%% 3B.F
Sf = (w*0.4)/2; % 40% front (static) load distribution
Fzfl = Sf - ((m*ay*h)/t)*0.45;
Fzfr = Sf + ((m*ay*h)/t)*0.45;
Cfl = (0.2196*((Fzfl/4.448)) - (7.35e-6*((Fzfl/4.448))^2))*4.448*(180/pi);
Cfr = (0.2196*((Fzfr/4.448)) - (7.35e-6*((Fzfr/4.448))^2))*4.448*(180/pi);
Cf = Cfl + Cfr;
af = Fyf/Cf;
fprintf('Ans 3B.F: The front slip angle (af) is %f rad\n\n',af)

%% 3B.G
Sr = (w*0.6)/2; % 60% rear (static) load distribution
FZrl = Sr - ((m*ay*h)/t)*0.55;
FZrr = Sr + ((m*ay*h)/t)*0.55;
Crl = (0.2196*((FZrl/4.448)) - (7.35e-6*((FZrl/4.448))^2))*4.448*(180/pi);
Crr = (0.2196*((FZrr/4.448)) - (7.35e-6*((FZrr/4.448))^2))*4.448*(180/pi);
Cr = Crl + Crr;
ar = Fyr/Cr;
fprintf('Ans 3B.G: The rear slip angle (ar) is %f rad\n\n',ar)

%% 3B.H
Mz = a*Fyf - b*Fyr;
fprintf('Ans 3B.H: The total yaw moment (Mz) is %f Nm\n\n',Mz)

%% 3B.I
B = b/R - ar;
fprintf('Ans 3B.I: The vehicle sideslip angle (B) is %f rad\n\n',B)

%% 3B.J
d = dA + af - ar;
fprintf('Ans 3B.J: The steering angle (d) is %f rad\n\n',d)