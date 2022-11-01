% Given data
Fz = 5200; % N
a = 0.1/2; % m
sigma_y = 0.0775;
c_y = 1250*10000; % N/m^2
u = 0.91;

%% Question A
% Generate gamma(x) vs x curves for uniform, parabolic and sinusoidal normal force distributions
uniform_gamma=[];
sinusoidal_gamma=[];
min_diff = inf;
xs = 0;
i=1;
for x=linspace(0,2*a,1000)
    uniform_gamma(i) = sigma_y*x;
    parabolic_gamma(i) = ((3*u*Fz)/(4*c_y*a^3))*(x*(2*a-x));
    sinusoidal_gamma(i) = ((pi*u*Fz)/(4*a*c_y))*(sin((pi*x)/(2*a)));
    % Try to find the intersection point of uniform and sinusoidal normal force distributions
    diff = abs(uniform_gamma(i)-sinusoidal_gamma(i));
    if x~=0 && diff < min_diff
        min_diff = diff;
        xs = x;
        xs_idx = i;
    end
    i=i+1;
end
xs
% Plot gamma(x) vs x curves for uniform, parabolic and sinusoidal normal
% force distributions with x_s marked for the sinusoidal normal force distribution
figure()
plot(linspace(0,2*a,1000),uniform_gamma,'Color','green')
hold on
plot(linspace(0,2*a,1000),parabolic_gamma,'Color','blue')
plot(linspace(0,2*a,1000),sinusoidal_gamma,'Color','red')
plot(xs,sigma_y*xs,'or')
plot(xs,0,'or')
plot(xs*ones(10),linspace(0,sigma_y*xs,10),'--r')
text(xs+0.001,0,strcat('x_s',num2str(xs)),'Color','red','VerticalAlignment','bottom','HorizontalAlignment','left')
xlabel('x')
ylabel('\gamma(x)')
legend('Uniform Normal Force Distribution','Parabolic Normal Force Distribution','Sinusoidal Normal Force Distribution','Location','northwest')
hold off
% Compute Fy for sinusoidal normal force distribution
Fy_sinusoidal = trapz(linspace(0,xs,xs_idx), c_y*uniform_gamma(1:xs_idx)) + trapz(linspace(xs,2*a,(1000-xs_idx)), c_y*sinusoidal_gamma(xs_idx+1:end))

%% Question B
% Compute Fy for parabolic normal force distribution
theta = (2*c_y*a^2)/(3*u*Fz);
Fy_parabolic = u*Fz*(3*theta*sigma_y - 3*theta^2*sigma_y^2 + theta^3*sigma_y^3)
% Compute Fy for parabolic and sinusoidal normal force distributions for 5
% different slip angles
Fy_par_arr = [];
Fy_sin_arr = [];
alpha_arr = 1:5;
for j=1:5
    % Compute Fy for parabolic normal force distribution
    sigma_y = tand(alpha_arr(j));
    theta = (2*c_y*a^2)/(3*u*Fz);
    Fy_par_arr(j) = u*Fz*(3*theta*sigma_y - 3*theta^2*sigma_y^2 + theta^3*sigma_y^3);
    % Compute Fy for sinusoidal normal force distribution
    %uniform_gamma=[];
    %sinusoidal_gamma=[];
    min_diff = inf;
    xs = 0;
    i=1;
    for x=linspace(0,2*a,1000)
        uniform_gamma(i) = sigma_y*x;
        sinusoidal_gamma(i) = ((pi*u*Fz)/(4*a*c_y))*(sin((pi*x)/(2*a)));
        diff = abs(uniform_gamma(i)-sinusoidal_gamma(i));
        if x~=0 && diff < min_diff
            min_diff = diff;
            xs = x;
            xs_idx = i;
        end
        i=i+1;
    end
    Fy_sin_arr(j) = trapz(linspace(0,xs,xs_idx), c_y*uniform_gamma(1:xs_idx)) + trapz(linspace(xs,2*a,(1000-xs_idx)), c_y*sinusoidal_gamma(xs_idx+1:end));
end
% Plot Fy vs slip angle curves for parabolic and sinusoidal normal force
% distributions for 5 different slip angles
figure()
plot(alpha_arr,Fy_par_arr,'Color','blue')
hold on
plot(alpha_arr,Fy_sin_arr,'Color','red')
xlabel('\alpha (deg)')
ylabel('F_y (N)')
legend('Parabolic Normal Force Distribution','Sinusoidal Normal Force Distribution','Location','northwest')
hold off