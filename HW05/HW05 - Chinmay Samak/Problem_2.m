%% Clear workspace
close all;
clear;
clc;

%% Given data
m = 120*14.594; % kg
l = 9/3.281; % m
C = 350*4.4482189159*(180/pi); % N/rad
R = 320/3.281; % m
a1 = 3.6/3.281; b1 = 5.4/3.281; % 60% front load distribution
a2 = 4.5/3.281; b2 = 4.5/3.281; % 50% front load distribution
a3 = 5.4/3.281; b3 = 3.6/3.281; % 40% front load distribution

%% Question A
v = 30/2.237; % m/s
% Damping in Sideslip (YB)
YB1 = -(2*C);
YB2 = -(2*C);
YB3 = -(2*C);
% Lateral Force / Yaw Coupling (Yr)
Yr1 = (C*(b1-a1))/v;
Yr2 = (C*(b2-a2))/v;
Yr3 = (C*(b3-a3))/v;
% Control Force Derivative (Yd)
Yd1 = C;
Yd2 = C;
Yd3 = C;
% Static Directional Stability (NB)
NB1 = C*(b1-a1);
NB2 = C*(b2-a2);
NB3 = C*(b3-a3);
% Yaw Damping (Nr)
Nr1 = -(C*(b1^2+a1^2))/v;
Nr2 = -(C*(b2^2+a2^2))/v;
Nr3 = -(C*(b3^2+a3^2))/v;
% Control Moment Derivative (Nd)
Nd1 = a1*C;
Nd2 = a2*C;
Nd3 = a3*C;
% Stability Derivatives
SD = [60 YB1 Yr1 Yd1 NB1 Nr1 Nd1;
      50 YB2 Yr2 Yd2 NB2 Nr2 Nd2;
      40 YB3 Yr3 Yd3 NB3 Nr3 Nd3];
fprintf('2A\n');
fprintf('Front Load   YB         Yr        Yd        NB        Nr        Nd\n');
disp(SD);
fprintf('\n')

%% Question B
UG1 = (m*(b1-a1))/(l*C);
UG2 = (m*(b2-a2))/(l*C);
UG3 = (m*(b3-a3))/(l*C);
K1 = UG1/l;
K2 = UG2/1;
K3 = UG3/l;
fprintf('2B\n');
fprintf('Stability factor at 60%% front load is %f rad.s^2.m^-2\n', K1);
fprintf('Stability factor at 50%% front load is %f rad.s^2.m^-2\n', K2);
fprintf('Stability factor at 40%% front load is %f rad.s^2.m^-2\n', K3);
fprintf('\n')

%% Question C
vc1 = sqrt(-1/K1);
vc2 = sqrt(-1/K2);
vc3 = sqrt(-1/K3);
fprintf('2C\n');
fprintf('Critical speed at 60%% front load is (%f + %f j) m/s (i.e. DOES NOT EXIST!)\n', real(vc1), imag(vc1));
fprintf('Critical speed at 50%% front load is (%f + %f j) m/s (i.e. DOES NOT EXIST!)\n', real(vc2), imag(vc2));
fprintf('Critical speed at 40%% front load is (%f + %f j) m/s (i.e. %f m/s)\n', real(vc3), imag(vc3), vc3);
fprintf('\n')

%% Question D
d = l/2;
fprintf('2D\n');
fprintf('Distance from the neutral steer point to the front tire is %f m\n', d);
fprintf('\n')

%% Question E
SM1 = (d-a1)/l;
SM2 = (d-a2)/l;
SM3 = (d-a3)/l;
fprintf('2E\n');
fprintf('Stability factor at 60%% front load is %f\n', SM1);
fprintf('Stability factor at 50%% front load is %f\n', SM2);
fprintf('Stability factor at 40%% front load is %f\n', SM3);
fprintf('\n')

%% Question F
v = 0:(1/2.237):(75/2.237); % m/s
r = v/R; % m
B1 = (b1/R)-((a1*m*v.^2)/(l*R*C));
B2 = (b2/R)-((a2*m*v.^2)/(l*R*C));
B3 = (b3/R)-((a3*m*v.^2)/(l*R*C));
Nr1 = -(C*(b1^2+a1^2))./v;
Nr2 = -(C*(b2^2+a2^2))./v;
Nr3 = -(C*(b3^2+a3^2))./v;
VSM1 = (NB1*B1)+(Nr1.*r);
VSM2 = (NB2*B2)+(Nr2.*r);
VSM3 = (NB3*B3)+(Nr3.*r);
figure(1)
plot(v,VSM1,'color','red')
hold on
plot(v,VSM2,'color','blue')
plot(v,VSM3,'color','green')
xlabel("Vehicle Velocity (m/s)")
ylabel("Theoretical Vehicle Stabilizing Moment (N-m)")
legend("60% Front Load", "50% Front Load", "40% Front Load", "Location","NW")

%% Question G
dA = l/R; % Ackermann Steering Angle (rad)
d1 = dA+(UG1*(v.^2/R));
d2 = dA+(UG2*(v.^2/R));
d3 = dA+(UG3*(v.^2/R));
VCM1 = Nd1*d1;
VCM2 = Nd2*d2;
VCM3 = Nd3*d3;
figure(2)
plot(VSM1,VCM1,'color','red')
hold on
plot(VSM2,VCM2,'*','color','blue')
plot(VSM3,VCM3,'color','green')
xlabel("Vehicle Stabilizing Moment (N-m)")
ylabel("Vehicle Control Moment (N-m)")
legend("60% Front Load", "50% Front Load", "40% Front Load", "Location","NE")