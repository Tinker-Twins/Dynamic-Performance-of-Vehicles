%% Clear Workspace

close all;
clear;
clc;

%% Given Data

m = 1637; % kg
g = 9.81; % m/s^2
Sf = (m*g*0.6)/2; % 60% front (static) load distribution
Sr = (m*g*0.4)/2; % 60% front (static) load distribution
Iz = 3326; % kg-m^2
l = 2.736; % m
t = 1.7; % m
a = 0.4*l; % 60% front load distribution
b = 0.6*l; % 60% front load distribution
G = 15;
h = 2.4/3.281; % m

C_array = [848 972 1088 1195 1292 1381 1461 1531 1593 1645 1689 1723 1748].*(180/pi); % N/rad
FZ_array = [2200 2600 3000 3400 3800 4200 4600 5000 5400 5800 6200 6600 7000]; % N
FZ_matrix = [FZ_array' (FZ_array.^2)'];
C_matrix = C_array';
ab = FZ_matrix\C_matrix;

%% Maneuver 1

% Simulation parameters
v = 74/2.237; % m/s
dt = 0.001; % s
X = [];
X(:,1) = [0;0];
Xdot = [];
Xint(:,1) = [0;0];
Bdot = 0;
V = [];
P = [0;0];
ay = [];
j = 1;

% Simulate from t=0 to t=7
U = 0; % Control input (@ 0 deg steering wheel angle)
for i = 0+dt:dt:7
    ay(j) = v*(Bdot+X(2,j));
    FZfl = Sf - ((m*ay(j)*h)/t)*0.6;
    FZfr = Sf + ((m*ay(j)*h)/t)*0.6;
    FZrl = Sr - ((m*ay(j)*h)/t)*0.4;
    FZrr = Sr + ((m*ay(j)*h)/t)*0.4;
    Cfl = ab(1)*FZfl + ab(2)*FZfl^2;
    Cfr = ab(1)*FZfr + ab(2)*FZfr^2;
    Cf = Cfl+Cfr;
    Crl = ab(1)*FZrl + ab(2)*FZrl^2;
    Crr = ab(1)*FZrr + ab(2)*FZrr^2;
    Cr = Crl+Crr;
    A = [-((Cf+Cr)/(m*v)) ((Cr*b-Cf*a)/(m*v^2)-1);
         (Cr*b-Cf*a)/Iz   -((Cr*b^2+Cf*a^2)/(Iz*v))];
    B = [Cf/(m*v); (Cf*a)/Iz];
    Xdot(:,j) = A*X(:,j)+B*U;
    X(:,j+1) = X(:,j) + Xdot(:,j)*dt;
    Xint(:,j+1) = Xint(:,j) + X(:,j)*dt;
    V(:,j) = [v*cos(X(1,j)+Xint(2,j)); v*sin(X(1,j)+Xint(2,j))];
    P(:,j+1) = P(:,j) + V(:,j)*dt;
    Bdot = Xdot(1,j);
    j = j+1;
end

% Simulate from t=7 to t=67
U = (45*(pi/180))/G; % Control input (@ 45 deg steering wheel angle)
for i = 7+dt:dt:67
    ay(j) = v*(Bdot+X(2,j));
    FZfl = Sf - ((m*ay(j)*h)/t)*0.6;
    FZfr = Sf + ((m*ay(j)*h)/t)*0.6;
    FZrl = Sr - ((m*ay(j)*h)/t)*0.4;
    FZrr = Sr + ((m*ay(j)*h)/t)*0.4;
    Cfl = ab(1)*FZfl + ab(2)*FZfl^2;
    Cfr = ab(1)*FZfr + ab(2)*FZfr^2;
    Cf = Cfl+Cfr;
    Crl = ab(1)*FZrl + ab(2)*FZrl^2;
    Crr = ab(1)*FZrr + ab(2)*FZrr^2;
    Cr = Crl+Crr;
    A = [-((Cf+Cr)/(m*v)) ((Cr*b-Cf*a)/(m*v^2)-1);
         (Cr*b-Cf*a)/Iz   -((Cr*b^2+Cf*a^2)/(Iz*v))];
    B = [Cf/(m*v); (Cf*a)/Iz];
    Xdot(:,j) = A*X(:,j)+B*U;
    X(:,j+1) = X(:,j) + Xdot(:,j)*dt;
    Xint(:,j+1) = Xint(:,j) + X(:,j)*dt;
    V(:,j) = [v*cos(X(1,j)+Xint(2,j)); v*sin(X(1,j)+Xint(2,j))];
    P(:,j+1) = P(:,j) + V(:,j)*dt;
    Bdot = Xdot(1,j);
    j = j+1;
end

% Simulate from t=67 to t=100
U = 0; % Control input (@ 0 deg steering wheel angle)
for i = 67+dt:dt:100
    ay(j) = v*(Bdot+X(2,j));
    FZfl = Sf - ((m*ay(j)*h)/t)*0.6;
    FZfr = Sf + ((m*ay(j)*h)/t)*0.6;
    FZrl = Sr - ((m*ay(j)*h)/t)*0.4;
    FZrr = Sr + ((m*ay(j)*h)/t)*0.4;
    Cfl = ab(1)*FZfl + ab(2)*FZfl^2;
    Cfr = ab(1)*FZfr + ab(2)*FZfr^2;
    Cf = Cfl+Cfr;
    Crl = ab(1)*FZrl + ab(2)*FZrl^2;
    Crr = ab(1)*FZrr + ab(2)*FZrr^2;
    Cr = Crl+Crr;
    A = [-((Cf+Cr)/(m*v)) ((Cr*b-Cf*a)/(m*v^2)-1);
         (Cr*b-Cf*a)/Iz   -((Cr*b^2+Cf*a^2)/(Iz*v))];
    B = [Cf/(m*v); (Cf*a)/Iz];
    Xdot(:,j) = A*X(:,j)+B*U;
    X(:,j+1) = X(:,j) + Xdot(:,j)*dt;
    Xint(:,j+1) = Xint(:,j) + X(:,j)*dt;
    V(:,j) = [v*cos(X(1,j)+Xint(2,j)); v*sin(X(1,j)+Xint(2,j))];
    P(:,j+1) = P(:,j) + V(:,j)*dt;
    Bdot = Xdot(1,j);
    j = j+1;
end

% Plot trajectory
PosX = P(1,:);
PosX_sec = PosX(1:1000:end-1000);
PosY = P(2,:);
PosY_sec = PosY(1:1000:end-1000);
VelX = V(1,:);
VelX_sec = VelX(1:1000:end);
VelY = V(2,:);
VelY_sec = VelY(1:1000:end);
figure(1)
plot(PosX,PosY,'color','red')
%comet(PosX,PosY)
hold on
quiver(PosX_sec,PosY_sec,VelX_sec,VelY_sec,'color','blue')
xlabel('${x}$ (m)','interpreter','latex')
ylabel('${y}$ (m)','interpreter','latex')

% Plot ay vs t
figure(2)
plot((0:dt:100),[0 ay],'.','color','red')
xlabel('${t}$ (sec)','interpreter','latex')
ylabel('${a_y}$ (m/sec$^2$)','interpreter','latex')

%% Maneuver 2

% Simulation parameters
v = 50/2.237; % m/s
dt = 0.001; % s
X = [];
X(:,1) = [0;0];
Xdot = [];
Xint(:,1) = [0;0];
Bdot = 0;
V = [];
P = [0;0];
ay = [];
j = 1;

% Simulate from t=0 to t=5
U = 0; % Control input (@ 0 deg steering wheel angle)
for i = 0+dt:dt:5
    ay(j) = v*(Bdot+X(2,j));
    FZfl = Sf - ((m*ay(j)*h)/t)*0.6;
    FZfr = Sf + ((m*ay(j)*h)/t)*0.6;
    FZrl = Sr - ((m*ay(j)*h)/t)*0.4;
    FZrr = Sr + ((m*ay(j)*h)/t)*0.4;
    Cfl = ab(1)*FZfl + ab(2)*FZfl^2;
    Cfr = ab(1)*FZfr + ab(2)*FZfr^2;
    Cf = Cfl+Cfr;
    Crl = ab(1)*FZrl + ab(2)*FZrl^2;
    Crr = ab(1)*FZrr + ab(2)*FZrr^2;
    Cr = Crl+Crr;
    A = [-((Cf+Cr)/(m*v)) ((Cr*b-Cf*a)/(m*v^2)-1);
         (Cr*b-Cf*a)/Iz   -((Cr*b^2+Cf*a^2)/(Iz*v))];
    B = [Cf/(m*v); (Cf*a)/Iz];
    Xdot(:,j) = A*X(:,j)+B*U;
    X(:,j+1) = X(:,j) + Xdot(:,j)*dt;
    Xint(:,j+1) = Xint(:,j) + X(:,j)*dt;
    V(:,j) = [v*cos(X(1,j)+Xint(2,j)); v*sin(X(1,j)+Xint(2,j))];
    P(:,j+1) = P(:,j) + V(:,j)*dt;
    Bdot = Xdot(1,j);
    j = j+1;
end

% Simulate from t=5 to t=23.621
U = 0:((14.5*(pi/180))/G)*dt:(270*(pi/180))/G; % Control input (@ 0-270 deg steering wheel angle)
for i = 5+dt:dt:23.621
    ay(j) = v*(Bdot+X(2,j));
    FZfl = Sf - ((m*ay(j)*h)/t)*0.6;
    FZfr = Sf + ((m*ay(j)*h)/t)*0.6;
    FZrl = Sr - ((m*ay(j)*h)/t)*0.4;
    FZrr = Sr + ((m*ay(j)*h)/t)*0.4;
    Cfl = ab(1)*FZfl + ab(2)*FZfl^2;
    Cfr = ab(1)*FZfr + ab(2)*FZfr^2;
    Cf = Cfl+Cfr;
    Crl = ab(1)*FZrl + ab(2)*FZrl^2;
    Crr = ab(1)*FZrr + ab(2)*FZrr^2;
    Cr = Crl+Crr;
    A = [-((Cf+Cr)/(m*v)) ((Cr*b-Cf*a)/(m*v^2)-1);
         (Cr*b-Cf*a)/Iz   -((Cr*b^2+Cf*a^2)/(Iz*v))];
    B = [Cf/(m*v); (Cf*a)/Iz];
    Xdot(:,j) = A*X(:,j)+B*U(j-5000);
    X(:,j+1) = X(:,j) + Xdot(:,j)*dt;
    Xint(:,j+1) = Xint(:,j) + X(:,j)*dt;
    V(:,j) = [v*cos(X(1,j)+Xint(2,j)); v*sin(X(1,j)+Xint(2,j))];
    P(:,j+1) = P(:,j) + V(:,j)*dt;
    Bdot = Xdot(1,j);
    j = j+1;
end

% Simulate from t=23.621 to t=27.621
U = (270*(pi/180))/G; % Control input (@ 270 deg steering wheel angle)
for i = 23.621+dt:dt:27.621
    ay(j) = v*(Bdot+X(2,j));
    FZfl = Sf - ((m*ay(j)*h)/t)*0.6;
    FZfr = Sf + ((m*ay(j)*h)/t)*0.6;
    FZrl = Sr - ((m*ay(j)*h)/t)*0.4;
    FZrr = Sr + ((m*ay(j)*h)/t)*0.4;
    Cfl = ab(1)*FZfl + ab(2)*FZfl^2;
    Cfr = ab(1)*FZfr + ab(2)*FZfr^2;
    Cf = Cfl+Cfr;
    Crl = ab(1)*FZrl + ab(2)*FZrl^2;
    Crr = ab(1)*FZrr + ab(2)*FZrr^2;
    Cr = Crl+Crr;
    A = [-((Cf+Cr)/(m*v)) ((Cr*b-Cf*a)/(m*v^2)-1);
         (Cr*b-Cf*a)/Iz   -((Cr*b^2+Cf*a^2)/(Iz*v))];
    B = [Cf/(m*v); (Cf*a)/Iz];
    Xdot(:,j) = A*X(:,j)+B*U;
    X(:,j+1) = X(:,j) + Xdot(:,j)*dt;
    Xint(:,j+1) = Xint(:,j) + X(:,j)*dt;
    V(:,j) = [v*cos(X(1,j)+Xint(2,j)); v*sin(X(1,j)+Xint(2,j))];
    P(:,j+1) = P(:,j) + V(:,j)*dt;
    Bdot = Xdot(1,j);
    j = j+1;
end

% Simulate from t=27.621 to t=40
U = 0; % Control input (@ 0 deg steering wheel angle)
for i = 27.621+dt:dt:40
    ay(j) = v*(Bdot+X(2,j));
    FZfl = Sf - ((m*ay(j)*h)/t)*0.6;
    FZfr = Sf + ((m*ay(j)*h)/t)*0.6;
    FZrl = Sr - ((m*ay(j)*h)/t)*0.4;
    FZrr = Sr + ((m*ay(j)*h)/t)*0.4;
    Cfl = ab(1)*FZfl + ab(2)*FZfl^2;
    Cfr = ab(1)*FZfr + ab(2)*FZfr^2;
    Cf = Cfl+Cfr;
    Crl = ab(1)*FZrl + ab(2)*FZrl^2;
    Crr = ab(1)*FZrr + ab(2)*FZrr^2;
    Cr = Crl+Crr;
    A = [-((Cf+Cr)/(m*v)) ((Cr*b-Cf*a)/(m*v^2)-1);
         (Cr*b-Cf*a)/Iz   -((Cr*b^2+Cf*a^2)/(Iz*v))];
    B = [Cf/(m*v); (Cf*a)/Iz];
    Xdot(:,j) = A*X(:,j)+B*U;
    X(:,j+1) = X(:,j) + Xdot(:,j)*dt;
    Xint(:,j+1) = Xint(:,j) + X(:,j)*dt;
    V(:,j) = [v*cos(X(1,j)+Xint(2,j)); v*sin(X(1,j)+Xint(2,j))];
    P(:,j+1) = P(:,j) + V(:,j)*dt;
    Bdot = Xdot(1,j);
    j = j+1;
end

% Plot trajectory
PosX = P(1,:);
PosX_sec = PosX(1:1000:end-1000);
PosY = P(2,:);
PosY_sec = PosY(1:1000:end-1000);
VelX = V(1,:);
VelX_sec = VelX(1:1000:end);
VelY = V(2,:);
VelY_sec = VelY(1:1000:end);
figure(3)
plot(PosX,PosY,'color','red')
%comet(PosX,PosY)
hold on
quiver(PosX_sec,PosY_sec,VelX_sec,VelY_sec,'color','blue')
xlabel('${x}$ (m)','interpreter','latex')
ylabel('${y}$ (m)','interpreter','latex')

% Plot ay vs t
figure(4)
plot((0:dt:40),[0 ay],'.','color','red')
xlabel('${t}$ (sec)','interpreter','latex')
ylabel('${a_y}$ (m/sec$^2$)','interpreter','latex')