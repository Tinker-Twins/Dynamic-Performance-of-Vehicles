%% Clear Workspace
close all;
clear;
clc;

%% Given Data
FZ = 4000; % Vertical load (N)
SA = deg2rad(4); % Slip angle (rad)
u = 0.6; % Coeff. of friction
a = 0.16/2; % Half patch length (m)
C = 3.5e6; % Cornering stiffness (N/m^2)

%% Calculate Intermediate Terms
S = tan(SA); % Lateral slip
T = (2*a^2*C)/(3*u*FZ); % Theta
xs = 2*a*(1-T*S); % Begining of sliding region
Gxs = (2*a-xs)/(2*a*T); % Gamma(xs)/xs

%% Question A
FY1 = []; FY2 = []; FY3 = []; FY4 = [];
for lw = linspace(0,2*a,100)
    if lw < xs % 0 <= lw < xs
        dFY1 = @(x) C.*x.*Gxs;
        FY1(end+1) = integral(dFY1,lw,xs);
        dFY2 = @(x) (C.*x.*(2.*a-x))/(2.*a.*T);
        FY2(end+1) = integral(dFY2,xs,2*a);
        FY3(end+1) = FY1(end) + FY2(end);
    else % xs <= lw <= 2a
        dFY4 = @(x) (C.*x.*(2.*a-x))/(2.*a.*T);
        FY4(end+1) = integral(dFY4,lw,2*a);
    end
end
FY = [FY3 FY4];
lw = linspace(0,100,100);
figure(1)
plot(lw,FY,'color','red')
xlabel('${l_w}$ (\%)','interpreter','latex')
ylabel('${F_y}$ (N)','interpreter','latex')

%% Question B
MZ1 = []; MZ2 = []; MZ3 = []; MZ4 = [];
for lw = linspace(0,2*a,100)
    if lw < xs % 0 <= lw < xs
        dMZ1 = @(x) C.*x.*Gxs.*(a-x);
        MZ1(end+1) = -integral(dMZ1,lw,xs);
        dMZ2 = @(x) (C.*x.*(2.*a-x).*(a-x))./(2.*a.*T);
        MZ2(end+1) = -integral(dMZ2,xs,2*a);
        MZ3(end+1) = MZ1(end) + MZ2(end);
    else % xs <= lw <= 2a
        dMZ4 = @(x) (C.*x.*(2.*a-x).*(a-x))/(2.*a.*T);
        MZ4(end+1) = -integral(dMZ4,lw,2*a);
    end
end
MZ = [MZ3 MZ4];
lw = linspace(0,100,100);
figure(2)
plot(lw,MZ,'color','blue')
xlabel('${l_w}$ (\%)','interpreter','latex')
ylabel('${M_z}$ (Nm)','interpreter','latex')

%% Question C
e = MZ./FY;
lw = linspace(0,100,100);
figure(3)
plot(lw,e,'color','magenta')
xlabel('${l_w}$ (\%)','interpreter','latex')
ylabel('${e}$ (m)','interpreter','latex')