% Given data
m = 2100; % kg
l = 3.3; % m
Cf = 78.9*1e3; % N/rad
Cr = 77.5*1e3; % N/rad
R = 144; % m
v = linspace(0,78*0.44704); % m/s
a1 = 2.046; b1 = 1.254; % m (38% front weight distrubution)
a2 = 1.650; b2 = 1.650; % m (50% front weight distrubution)
a3 = 1.254; b3 = 2.046; % m (62% front weight distrubution)
% Lateral acceleration
ay = (v.^2)/R; % m/s^2
% Front slip angles
af1 = (b1/l)*(m*ay/Cf); % rad
af2 = (b2/l)*(m*ay/Cf); % rad
af3 = (b3/l)*(m*ay/Cf); % rad
figure(1)
plot(ay,af1,'Color','red')
hold on
plot(ay,af2,'Color','green')
plot(ay,af3,'Color','blue')
xlabel('Lateral Acceleration (m/s^2)')
ylabel('Front Slip Angle (rad)')
legend('38% Front Weight Distribution','50% Front Weight Distribution','62% Front Weight Distribution','Location','NW')
hold off
% Rear slip angles
ar1 = (a1/l)*(m*ay/Cr); % rad
ar2 = (a2/l)*(m*ay/Cr); % rad
ar3 = (a3/l)*(m*ay/Cr); % rad
figure(2)
plot(ay,ar1,'Color','red')
hold on
plot(ay,ar2,'Color','green')
plot(ay,ar3,'Color','blue')
xlabel('Lateral Acceleration (m/s^2)')
ylabel('Rear Slip Angle (rad)')
legend('38% Front Weight Distribution','50% Front Weight Distribution','62% Front Weight Distribution','Location','NW')
hold off
% Vehicle sideslip angles
B1 = (b1/R)-abs(ar1); % rad
B2 = (b2/R)-abs(ar2); % rad
B3 = (b3/R)-abs(ar3); % rad
figure(3)
plot(ay,B1,'Color','red')
hold on
plot(ay,B2,'Color','green')
plot(ay,B3,'Color','blue')
xlabel('Lateral Acceleration (m/s^2)')
ylabel('Vehicle Sideslip Angle (rad)')
legend('38% Front Weight Distribution','50% Front Weight Distribution','62% Front Weight Distribution','Location','SW')
hold off
% Steering angles
d1 = (l/R)-abs(af1)+abs(ar1); % rad
d2 = (l/R)-abs(af2)+abs(ar2); % rad
d3 = (l/R)-abs(af3)+abs(ar3); % rad
figure(4)
plot(ay,d1,'Color','red')
hold on
plot(ay,d2,'Color','green')
plot(ay,d3,'Color','blue')
xlabel('Lateral Acceleration (m/s^2)')
ylabel('Steering Angle (rad)')
legend('38% Front Weight Distribution','50% Front Weight Distribution','62% Front Weight Distribution','Location','SW')
hold off