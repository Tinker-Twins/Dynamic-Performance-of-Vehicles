%% Problem 4 (A)
% Load data
load('LargeFWD1.mat')
% Plot B values
v_wind = 4.8*0.447;
v_x = Vx + v_wind;
T = t(length(t));
v_0 = v_x(1)+v_wind;
t_T = zeros(1, length(t));
vt_v0 = zeros(1, length(v_x));
B = [1, 1.4, 1.8, 2.2, 2.6, 3.0];
for j = 1:length(B)
    for i = 1:length(t)
        t_T(i) = t(i)/T;
        v_t = ((v_0/B(j)) * tan(1-(t(i)/T)*atan(B(j)))) + v_wind;
        vt_v0(i) = v_t/v_0;
    end
    figure(1)
    plot(t_T, vt_v0);
    xlabel('t/T'); ylabel('v_t/v_0');
    hold on;
end
for i = 1:length(t)
    t_T(i) = t(i)/T;
    vt_v0(i) = v_x(i)/v_0;
end
plot(t_T, vt_v0);
legend('\beta = 1','\beta = 1.4','\beta = 1.8','\beta = 2.2','\beta = 2.6','\beta = 3.0','Real Data');
hold off;
%% Problem 4 (B)
% Compute CD and Rx
B = 1.8; % Chosen by inspecting the plot
m = 78.8*14.5939;
Af = 22.17*0.0929;
rho = 1.225;
CD = (2*m*B*atan(B))/((v_0)*T*rho*Af)
Rx = (v_0*m*atan(B))/(B*T)