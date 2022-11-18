clc; clear; close all;
%% Time Vector
T = 75; % Maneuver1=67 + 60 seconds; Maneuver2=29 
seconds;
delta_t = 0.001;
time = linspace(0,T,(T/delta_t)+1);
%% Vehicle Parameters
%global m Iz l a b cf cr Fzf Fzr
m = 1637; %kg 1150
Iz = 3326; %kg-m^2 1850 
l = 2.736; %m 2.66
a = 0.4*l; %m 1.064
b = 0.6*l; %m 1.596
cf = 802*180/pi; %N/rad 2*1312*180/pi
cr = 785*180/pi; %N/rad 2*984*180/pi
v = 70*0.44704; %m/s, maneuver1=70mph; maneuver2=50mph
Fzf = m*9.81*b/l; %N, Front tire normal load
Fzr = m*9.81-Fzf; %N, Rear tire normal load
%% Generate Steering Input
% Maneuver 1
delta_sw = zeros(1,(T/delta_t)+1);
index1 = (7/delta_t)+1;
index2 = (67/delta_t)+1;
index3= (125/delta_t)+1;
delta_sw(index1+1:index2) = 45/15;
%delta_sw(index2+5000:index3) = -45/15;
deltavec = delta_sw*pi/180;
%% Simulating model
% Choose tire model
x=zeros(2,1);
xDot=zeros(2,length(x));
for i = 1:(T/delta_t) 
 xdot = bicyclemodel_linear(x(:,i),deltavec(i),v);
 x(:,i+1) = x(:,i) + delta_t * xdot;
 xDot(:,i)=xdot;
end
alphaf=deltavec-x(2,:)*a/v-x(1,:);
alphar=x(2,:)*b/v-x(1,:);
%% Plots
k=1;
figure(k);
k=k+1;
clf;
subplot(2,1,1);
plot(time,deltavec,'b');
grid on; grid minor;
xlabel('Time (s)');
ylabel('Steering input, \delta (radian)')
title('Steering Input');
subplot(2,1,2);
plot(time,delta_sw,'b');
grid on; grid minor;
xlabel('Time (s)');
ylabel('Steering input, \delta (degree)');
title('Steering Input');
figure(k);
k=k+1;
clf;
subplot(2,1,1);
plot(time,x(1,:));
grid on; grid minor;
xlabel('Time (s)');
ylabel('x_1');
legend('Beta','Location','Best');
title('Body slip angle (rad)')
subplot(2,1,2);
plot(time,x(2,:));
grid on; grid minor;
xlabel('Time (s)');
ylabel('x_2');
legend('Psi_{dot}','Location','Best');
title('Yaw rate (rad/s)')
figure(k)
k=k+1;
clf;
subplot(2,1,1);
plot(time,alphaf);
grid on; grid minor;
xlabel('Time (s)');
ylabel('Front slip angle');
legend('Beta','Location','Best');
title('Front slip angle (rad/s)')
subplot(2,1,2);
plot(time,alphar);
grid on; grid minor;
xlabel('Time (s)');
ylabel('Rear slip angle');
legend('Psi_{dot}','Location','Best');
title('Rear slip angle (rad)')
function xdot=bicyclemodel_linear(x,delta,v)
 m = 1637; %kg 1150
 Iz = 3326; %kg-m^2 1850 
 l = 2.736; %m 2.66
 a = 0.4*l; %m 1.064
 b = 0.6*l; %m 1.596
 cf = 802*180/pi; %N/rad 2*1312*180/pi
 cr = 785*180/pi; %N/rad 2*984*180/pi
 a11 = -((cr + cf)/(m*v));
 a12 = ((cr*b - cf*a)/(m*v^2))-1;
 a21 = (cr*b - cf*a)/Iz;
 a22 = -(cr*(b^2) + cf*(a^2))/(Iz*v);
 b11 = cf/(m*v);
 b21 = (cf*a)/Iz;
 beta=x(1);
 r=x(2); 
 betadot = a11*beta + a12*r + b11*delta;
 rdot = a21*beta + a22*r + b21*delta;
 xdot = [betadot;...
 rdot];
end
