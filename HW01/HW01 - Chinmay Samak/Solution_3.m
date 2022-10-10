%% Problem 3 (C)
% Given data
R = (0.35/pi)*1609.344;
ay = 1.2*9.81;
% Compute v_corner
v_corner = abs(sqrt(R*ay))

%% Problem 3 (E)
% Given data
Pw = 370*745.6998;
CDA = 7.5*0.0929;
m = 80*14.5939;
a_brake = -1.2*9.81;
rho = 1.225;
L = 0.4*1609.344;
delta_t = 0.001;
% Euler Forward
a_accel = [];
v = [];
S = [];
v(1) = v_corner;
S(1) = 0;
i = 1;
while S < L
    a_accel(i) = (Pw-((0.5*rho*v(i)^2*CDA)*v(i)))/(m*v(i));
    v(i+1) = v(i)+delta_t*a_accel(i);
    S(i+1) = S(i)+delta_t*v(i);
    i = i+1;
    if abs(v(i)^2 - (v_corner^2 + 2*abs(a_brake)*(L-S(i)))) <= 1
        break;
    end
end
ST = S(length(S))

%% Problem 3 (D)
% Compute v_max
v_max = v(length(v))

%% Problem 3 (B)
% Compute v_avg
v_avg = (mean(v) + (v_max+v_corner)/2 + v_corner) / 3

%% Problem 3 (A)
% Given data
d = 1.5*1609.344;
% Compute t_min
t_min = d/v_avg