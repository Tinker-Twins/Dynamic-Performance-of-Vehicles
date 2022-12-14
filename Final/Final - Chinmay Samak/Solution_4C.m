%% 4C.A
close all;
clear;
clc;

% Data assumed from 4A
m = 143*14.594; % Vehicle mass (kg)
g = 9.81; % Acceleration due to gravity (m/s^2)
h = 2.3/3.281; % Height of CG (m)
L = 9.45/3.281; % Wheelbase (m)
W = 5.25/3.281; % Trackwidth (m)
a = (1-0.54)*L; % Distance of CG from front axle (m)
b = L-a; % Distance of CG from rear axle (m)

% Data given in 4C
ax = 2; % Longitudinal acceleration (m/s^2) (braking assumed positive in 4B)
hpg = 1.5/3.281; % Height of PC from ground (m)
Kf = 45e3; % Front suspension stiffness (N/m)
Kr = 52e3; % Rear suspension stiffness (N/m)
Cf = 0; % Front suspension damping (Ns/m)
Cr = 0; % Rear suspension damping (Ns/m)

% Intermediate variables
hp = h - hpg; % Height of PC from CG (m)
Iyy = 0.4*((m/12)*(L^2+W^2)); % Pitch MOI about CG (kg-m^2)
Itheta = Iyy + m*hp^2; % Pitch MOI about PC (kg-m^2)

% Simulation settings
ms = 1*m; % Sprung mass assumed to be 100% of total vehicle mass (kg)
theta_sim = [0]; % Initial pitch angle assumption (rad)
theta_dot = [0]; % Initial pitch velocity assumption (rad/s)
dt = 1e-6; % Simulation timestep (s)
i = 1; % Iterator count

% Simulate from i=0 to i=5 (@ 2 m/s^2 braking acceleration)
for t = 0+dt:dt:5
    theta_ddot(i) = 1/Itheta * (ms*ax*hp + ms*g*hp*theta_sim(i) - Kf*a^2*theta_sim(i) - Kr*b^2*theta_sim(i) - Cf*a^2*theta_dot(i) - Cr*b^2*theta_dot(i));
    theta_dot(i+1) = theta_dot(i) + theta_ddot(i)*dt;
    theta_sim(i+1) = theta_sim(i) + theta_dot(i)*dt;
    i = i+1;
end

% Plot theta vs t
figure()
plot((0:dt:5),theta_sim,'.','color','red')
xlabel('${t}$ (sec)','interpreter','latex')
ylabel('${\theta}$ ($rad$)','interpreter','latex')

% Plot theta_dot vs t
figure()
plot((0:dt:5),theta_dot,'.','color','blue')
xlabel('${t}$ (sec)','interpreter','latex')
ylabel('${\dot\theta}$ ($rad/s$)','interpreter','latex')

% Plot theta_ddot vs t
figure()
plot((dt:dt:5),theta_ddot,'.','color','magenta')
xlabel('${t}$ (sec)','interpreter','latex')
ylabel('${\ddot\theta}$ ($rad/s^2$)','interpreter','latex')

%% 4C.B
clear;
clc;

% Data assumed from 4A
m = 143*14.594; % Vehicle mass (kg)
g = 9.81; % Acceleration due to gravity (m/s^2)
h = 2.3/3.281; % Height of CG (m)
L = 9.45/3.281; % Wheelbase (m)
W = 5.25/3.281; % Trackwidth (m)
a = (1-0.54)*L; % Distance of CG from front axle (m)
b = L-a; % Distance of CG from rear axle (m)

% RECORDED DATA
load('FinalExamData.mat'); % Load given data
theta_rec = asin((Rh_lr-Rh_lf)./L);

% (SUB)OPTIMAL ESTIMATION OF SUSPENSION DAMPING COEFFICIENTS (ASSUMING THEY ARE EQUAL)
ax = Acc_x; % Longitudinal acceleration (m/s^2) (braking assumed positive in 4B)
hpg = 1.5/3.281; % Height of PC from ground (m)
Kf = 45e3; % Front suspension stiffness (N/m)
Kr = 52e3; % Rear suspension stiffness (N/m)
hp = h - hpg; % Height of PC from CG (m)
Iyy = 0.4*((m/12)*(L^2+W^2)); % Pitch MOI about CG (kg-m^2)
Itheta = Iyy + m*hp^2; % Pitch MOI about PC (kg-m^2)
ms = 1*m; % Sprung mass assumed to be 100% of total vehicle mass (kg)
theta_sim = [0]; % Initial pitch angle assumption (rad)
theta_dot = [0]; % Initial pitch velocity assumption (rad/s)
dt = time(2) - time(1); % Simulation timestep (s)
i = 1; % Iterator count
C_optimal = 0;
error_minimum = Inf;
for C = 0:10:1e6
    i = 1;
    for t = 0+dt:dt:time(end)
        theta_ddot(i) = 1/Itheta * (ms*ax(i)*hp + ms*g*hp*theta_sim(i) - Kf*a^2*theta_sim(i) - Kr*b^2*theta_sim(i) - C*a^2*theta_dot(i) - C*b^2*theta_dot(i));
        theta_dot(i+1) = theta_dot(i) + theta_ddot(i)*dt;
        theta_sim(i+1) = theta_sim(i) + theta_dot(i)*dt;
        i = i+1;
    end
    error = sqrt((sum((theta_rec - theta_sim).^2))/length(theta_rec)); % RMSE
    if error < error_minimum
        error_minimum = error;
        C_optimal = C;
    end
end
fprintf(['Ans 4C.B: The estimated suspension damping coefficients (assumung they are equal)\n' ...
         'considering RMSE between given data and simulation results are %f Ns/m\n\n'],C_optimal)

% SIMULATION
ax = Acc_x; % Longitudinal acceleration (m/s^2) (braking assumed positive in 4B)
hpg = 1.5/3.281; % Height of PC from ground (m)
Kf = 45e3; % Front suspension stiffness (N/m)
Kr = 52e3; % Rear suspension stiffness (N/m)
Cf = C_optimal; % Front suspension damping (Ns/m)
Cr = C_optimal; % Rear suspension damping (Ns/m)
hp = h - hpg; % Height of PC from CG (m)
Iyy = 0.4*((m/12)*(L^2+W^2)); % Pitch MOI about CG (kg-m^2)
Itheta = Iyy + m*hp^2; % Pitch MOI about PC (kg-m^2)
ms = 1*m; % Sprung mass assumed to be 100% of total vehicle mass (kg)
theta_sim = [0]; % Initial pitch angle assumption (rad)
theta_dot = [0]; % Initial pitch velocity assumption (rad/s)
dt = time(2) - time(1); % Simulation timestep (s)
i = 1; % Iterator count
for t = 0+dt:dt:time(end)
    theta_ddot(i) = 1/Itheta * (ms*ax(i)*hp + ms*g*hp*theta_sim(i) - Kf*a^2*theta_sim(i) - Kr*b^2*theta_sim(i) - Cf*a^2*theta_dot(i) - Cr*b^2*theta_dot(i));
    theta_dot(i+1) = theta_dot(i) + theta_ddot(i)*dt;
    theta_sim(i+1) = theta_sim(i) + theta_dot(i)*dt;
    i = i+1;
end

% Plot theta vs t
figure()
plot((0:dt:time(end)),theta_rec,'.','color','blue')
hold on
plot((0:dt:time(end)),theta_sim,'.','color','red')
xlabel('${t}$ (sec)','interpreter','latex')
ylabel('${\theta}$ ($rad$)','interpreter','latex')
legend('${\theta}_{recorded}$','${\theta}_{simulated}$','interpreter','latex')
hold off

% Plot theta_dot vs t
figure()
plot((0:dt:time(end)),theta_dot,'.','color','blue')
xlabel('${t}$ (sec)','interpreter','latex')
ylabel('${\dot\theta}$ ($rad/s$)','interpreter','latex')

% Plot theta_ddot vs t
figure()
plot((dt:dt:time(end)),theta_ddot,'.','color','magenta')
xlabel('${t}$ (sec)','interpreter','latex')
ylabel('${\ddot\theta}$ ($rad/s^2$)','interpreter','latex')