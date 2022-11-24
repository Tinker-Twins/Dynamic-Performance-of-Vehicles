%% Clear Workspace

close all;
clear;
clc;

%% Given Data

m = 1637; % kg
g = 9.81; % m/s^2
Iz = 3326; % kg-m^2
l = 2.736; % m
t = 1.7; % m
a = 0.4*l; % 60% front load distribution
b = 0.6*l; % 60% front load distribution
G = 15;
h = 2.4/3.281; % m

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
Sf = (m*g*0.6)/2; % 60% front (static) load distribution
Sr = (m*g*0.4)/2; % 60% front (static) load distribution
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
    af(j) = U - (X(2,j)*a)/v - X(1,j);
    ar(j) = (X(2,j)*b)/v - X(1,j);
    FYfl = -nonlintire(af(j),FZfl,v);
    FYfr = -nonlintire(af(j),FZfr,v);
    FYf(j) = FYfl+FYfr;
    FYrl = -nonlintire(ar(j),FZrl,v);
    FYrr = -nonlintire(ar(j),FZrr,v);
    FYr(j) = FYrl+FYrr;
    Xdot(1,j) = (FYf(j)+FYr(j))/(m*v) - X(2,j);
    Xdot(2,j) = (FYf(j)*a-FYr(j)*b)/(Iz);
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
    af(j) = U(j-5000) - (X(2,j)*a)/v - X(1,j);
    ar(j) = (X(2,j)*b)/v - X(1,j);
    FYfl = -nonlintire(af(j),FZfl,v);
    FYfr = -nonlintire(af(j),FZfr,v);
    FYf(j) = FYfl+FYfr;
    FYrl = -nonlintire(ar(j),FZrl,v);
    FYrr = -nonlintire(ar(j),FZrr,v);
    FYr(j) = FYrl+FYrr;
    Xdot(1,j) = (FYf(j)+FYr(j))/(m*v) - X(2,j);
    Xdot(2,j) = (FYf(j)*a-FYr(j)*b)/(Iz);
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
    af(j) = U - (X(2,j)*a)/v - X(1,j);
    ar(j) = (X(2,j)*b)/v - X(1,j);
    FYfl = -nonlintire(af(j),FZfl,v);
    FYfr = -nonlintire(af(j),FZfr,v);
    FYf(j) = FYfl+FYfr;
    FYrl = -nonlintire(ar(j),FZrl,v);
    FYrr = -nonlintire(ar(j),FZrr,v);
    FYr(j) = FYrl+FYrr;
    Xdot(1,j) = (FYf(j)+FYr(j))/(m*v) - X(2,j);
    Xdot(2,j) = (FYf(j)*a-FYr(j)*b)/(Iz);
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
    af(j) = U - (X(2,j)*a)/v - X(1,j);
    ar(j) = (X(2,j)*b)/v - X(1,j);
    FYfl = -nonlintire(af(j),FZfl,v);
    FYfr = -nonlintire(af(j),FZfr,v);
    FYf(j) = FYfl+FYfr;
    FYrl = -nonlintire(ar(j),FZrl,v);
    FYrr = -nonlintire(ar(j),FZrr,v);
    FYr(j) = FYrl+FYrr;
    Xdot(1,j) = (FYf(j)+FYr(j))/(m*v) - X(2,j);
    Xdot(2,j) = (FYf(j)*a-FYr(j)*b)/(Iz);
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

% Concatenate array of steering angle inputs
d = [zeros(size(0:dt:5)) ones(size(5+dt:dt:23.621))*0:((14.5*(pi/180))/G)*dt:(270*(pi/180))/G ones(size(23.621+dt:dt:27.621))*(270*(pi/180))/G zeros(size(27.621+dt:dt:40))];

% Plot d vs ay
figure(2)
plot([0 ay],d,'.','color','red')
xlabel('${a_y}$ (m/sec$^2$)','interpreter','latex')
ylabel('${\delta}$ (rad)','interpreter','latex')

% Calculate sublimit UG (from linear region of d vs. ay plot)
d_ay = polyfit(ay(5020:8300),d(5020:8300),1);
figure(3)
plot(ay(5200:8300),d(5200:8300),'.','color','red')
hold on
plot(ay(5200:8300),polyval(d_ay,ay(5200:8300)),'.','color','blue')
legend('Simulated Data','1st Order Polyfit','Location','NW')
xlabel('${a_y}$ (m/sec$^2$)','interpreter','latex')
ylabel('${\delta}$ (rad)','interpreter','latex')
UG = polyder(d_ay)-(l/v^2);
fprintf('UG for Maneuver 2 with Non-Linear Tire Model and Load Transfer is %f rad sec^2 m^-1\n',UG)

% Analyze af and ar to comment on limit behavior
figure(4)
plot(af(5200:8300),FYf(5200:8300),'.','color','red')
hold on
plot(ar(5200:8300),FYr(5200:8300),'.','color','blue')
legend('Front Tire','Rear Tire','Location','NW')
xlabel('${\alpha_{yf/r}}$ (rad)','interpreter','latex')
ylabel('${F_{yf/r}}$ (N)','interpreter','latex')
fprintf(['\nLooking at variation of lateral force vs. slip angle plots for front & rear tires (with load transfer), and assuming\n' ...
         'same tire quality (i.e., same breakaway slip angle), we can observe that since the slope of the lateral force vs. slip\n' ...
         'angle plot in the linear region (i.e., cornering stiffness) of rear tire is less than that of the front tire, it will\n' ...
         'saturate (reach maximum lateral force) before the front tire, indicating that the vehicle will be limit oversteer.\n\n'])