%% Clear Workspace
close all;
clear;
clc;

%% Given Data
w = 4e4*4.448; % Weight (N)
m = w/9.81; % Mass (kg)
l = 15/3.281; % Wheelbase (m)
a = 0.6*l; % Length of CG from front axle (m) for 40% front load
b = 0.4*l; % Length of CG from rear axle (m) for 40% front load
d = 4/3.281; % Length of front axle from snowplow (m)
v = 50/2.237; % Velocity (m/s)
R = 500/3.281; % Turning radius
Fs = 1000*4.448; % Snow force (N)

%% 3A.A
dA = atan2(l,R);
fprintf('Ans 3A.A: The Ackerman steering angle (dA) is %f rad\n\n',dA)

%% 3A.B
ay = v^2/R;
fprintf('Ans 3A.B: The lateral acceleration (ay) is %f m/s^2\n\n',ay)

%% 3A.C
r = v/R;
fprintf('Ans 3A.C: The yaw rate (r) is %f rad/s\n\n',r)

%% 3A.D
Fy = m*ay;
Fyfr = [1,1;-a,b]\[(Fs+Fy);-Fs*(a+d)];
Fyf = Fyfr(1);
fprintf('Ans 3A.D: The front lateral force (Fyf) is %f N\n\n',Fyf)

%% 3A.E
Fyr = Fyfr(2);
fprintf('Ans 3A.E: The rear lateral force (Fyr) is %f N\n\n',Fyr)

%% 3A.F
Cf = 2*(0.2196*(0.5*0.4*(w/4.448)) - (7.35e-6*(0.5*0.4*(w/4.448))^2))*4.448*(180/pi);
af = Fyf/Cf;
fprintf('Ans 3A.F: The front slip angle (af) is %f rad\n\n',af)

%% 3A.G
Cr = 2*(0.2196*(0.5*0.6*(w/4.448)) - (7.35e-6*(0.5*0.6*(w/4.448))^2))*4.448*(180/pi);
ar = Fyr/Cr;
fprintf('Ans 3A.G: The rear slip angle (ar) is %f rad\n\n',ar)

%% 3A.H
Mz = a*Fyf - b*Fyr;
fprintf('Ans 3A.H: The total yaw moment (Mz) is %f Nm\n\n',Mz)

%% 3A.I
B = b/R - ar;
fprintf('Ans 3A.I: The vehicle sideslip angle (B) is %f rad\n\n',B)

%% 3A.J
d = dA + af - ar;
fprintf('Ans 3A.J: The steering angle (d) is %f rad\n\n',d)