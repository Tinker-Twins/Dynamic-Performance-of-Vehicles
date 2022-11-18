%% Clear workspace
close all;
clear;
clc;

%% Given data
m = 1637; % kg
Iz = 3326; % kg-m^2
l = 2.736; % m
a = 0.4*l; % 60% front load distribution
b = 0.6*l; % 60% front load distribution
Cf = 802*(180/pi); % N/rad
Cr = 785*(180/pi); % N/rad
G = 15;
v = 74/2.237; % m/s

%% Simulation
global A B U;
% System matrix A
A = [-((Cf+Cr)/(m*v)) ((Cr*b-Cf*a)/(m*v^2)-1);
     (Cr*b-Cf*a)/Iz   -((Cr*b^2+Cf*a^2)/(Iz*v))];
% Control matrix B
B = [Cf/(m*v); (Cf*a)/Iz];

% Simulate from t=0 to t=7
tspan = [0 7]; % Timespan
inicon = [0; 0]; % Initial condition [beta0; r0]
U = 0; % Control input (@ 0 deg steering wheel angle)
[T1, X1] = ode45(@dynamic_bicycle, tspan, inicon);

% Simulate from t=7 to t=67
tspan = [7 67]; % Timespan
inicon = [X1(end,1); X1(end,2)]; % Initial condition [beta1; r1]
U = (45*(pi/180))/G; % Control input (@ 45 deg steering wheel angle)
[T2, X2] = ode45(@dynamic_bicycle, tspan, inicon);

% Simulate from t=67 to t=100
tspan = [67 100]; % Timespan
inicon = [X2(end,1); X2(end,2)]; % Initial condition [beta2; r2]
U = 0; % Control input (@ 0 deg steering wheel angle)
[T3, X3] = ode45(@dynamic_bicycle, tspan, inicon);

t = [T1; T2; T3]; % Concatenate time arrays
u = [zeros(size(T1)); ones(size(T2))*(45*(pi/180))/G; zeros(size(T3))]; % Construct & concatenate steering angle arrays
x1 = [X1(:,1); X2(:,1); X3(:,1)]; % Extract & concatenate beta arrays
x2 = [X1(:,2); X2(:,2); X3(:,2)]; % Extract & concatenate r arrays

%% Question A
figure(1)
plot(t,x1,'color','red')
xlabel('${t}$ (sec)','interpreter','latex')
ylabel('${\beta}$ (rad)','interpreter','latex')

%% Question B
figure(2)
plot(t,x2,'color','blue')
xlabel('${t}$ (sec)','interpreter','latex')
ylabel('${\dot\psi}$ (rad/sec)','interpreter','latex')

%% Question C
af = u - (x2*a)/v - x1;
figure(3)
plot(t,af,'color','green')
xlabel('${t}$ (sec)','interpreter','latex')
ylabel('${\alpha_f}$ (rad)','interpreter','latex')

%% Question D
ar = (x2*b)/v - x1;
figure(4)
plot(t,ar,'color','magenta')
xlabel('${t}$ (sec)','interpreter','latex')
ylabel('${\alpha_r}$ (rad)','interpreter','latex')

%% Dynamic bicycle model to solve using ODE45
function Xdot = dynamic_bicycle(T, X)
global A B U;
Xdot = A*X + B*U;
end