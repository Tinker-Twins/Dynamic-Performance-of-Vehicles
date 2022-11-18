clc; clear; close all;
% Problem 1:
L=3.1; % wheel base in m
V=[80 120]*(5/18); % velocity limits in m/s
UG=L./V.^2; % understeer gradient
SR=19; % Steering ratio
ay=5; % Lateral acceleration in m/s^2
Vchar=(60:1:130)*5/18; % chartacteristic velcity in m/s
Steer_Sen=V(2)/(2*L); % Maximum steering sensitivity
R=Vchar.^2/ay; % Radius of turn
U=L./Vchar.^2; % UG
dsteer=zeros(1,length(Vchar));
for i=1:length(Vchar)
    dsteer(i)=SR*(L/R(i)+U(i)*ay);
end
figure(1)
plot(Vchar,dsteer,'Linewidth',2)
xlabel('Characteristic velocity in m/s')
ylabel('Steering angle')
title('Steering angle vs Charctersitic velocity')
grid on
%%
% All units converted to SI system
mass=120*14.5939; %kg
cf=350*4.44822*180/pi; %N/rad
cr=350*4.44822*180/pi; %N/rad

Radius=320*0.3048; l=9*0.3048;
a=[3.6*0.3048,4.5*0.3048,5.4*0.3048];
b=[5.4*0.3048,4.5*0.3048,3.6*0.3048];
yb=zeros(1,3); yr=zeros(1,3); yd=zeros(1,3);
nb=zeros(1,3); nr=zeros(1,3); nd=zeros(1,3);
k=zeros(1,3); ug=zeros(1,3); v_critical=zeros(1,3);
nsp=zeros(1,3); sm=zeros(1,3); 

vx=30*0.44704;
for i=1:3
   yb(i)=-(cf+cr); % Lateral force damping
   yr(i)=(b(i)*cr-a(i)*cf)/vx; % Lateral force damping
   yd(i)=cf; % Lateral force control
   nb(i)=b(i)*cr-a(i)*cf; % Yaw coupling/Static directional stability
   nr(i)=-(a(i)^2*cf+b(i)^2*cr)/vx; % yaw damping
   nd(i)=a(i)*cf; % yaw control 
   ug(i)=mass*(cr*b(i)-cf*a(i))/(l*cr*cf); % understeer gradient
   k(i)=ug(i)/l; 
   v_critical(i)=sqrt(-1/k(i)); % Critical velocity
%    nsp(i)=((a(i)/l)-((nb(i)/yb(i))*(1/l)))*l;
   nsp(i)=a(i)-(nb(i)/yb(i)); % neutral steer point
   sm(i)=(nsp(i)-a(i))/l; % static margin
end

vel_range=linspace(0,75*0.44704);len=length(vel_range);
veh_stab_moment=zeros(3,len); veh_cont_moment=zeros(3,len);
dA=l/Radius;
beta=zeros(3,len); Nbeta=zeros(3,len);
r=zeros(3,len); Nr=zeros(3,len);
delta=zeros(3,len); Ndelta=zeros(3,len); 

for i=1:len
    beta(1,i)=(b(1)/Radius)-(a(1)*mass/(l*cr))*(vel_range(i)^2/Radius);
    beta(2,i)=(b(2)/Radius)-(a(2)*mass/(l*cr))*(vel_range(i)^2/Radius);
    beta(3,i)=(b(3)/Radius)-(a(3)*mass/(l*cr))*(vel_range(i)^2/Radius);
    
    Nbeta(1,i)=b(1)*cr-a(1)*cf;
    Nbeta(2,i)=b(2)*cr-a(2)*cf;
    Nbeta(3,i)=b(3)*cr-a(3)*cf;
    
    r(1,i)=vel_range(i)/Radius;
    r(2,i)=vel_range(i)/Radius;
    r(3,i)=vel_range(i)/Radius;
    
    Nr(1,i)=-(a(1)^2*cf+b(1)^2*cr)/vel_range(i);
    Nr(2,i)=-(a(2)^2*cf+b(2)^2*cr)/vel_range(i);
    Nr(3,i)=-(a(3)^2*cf+b(3)^2*cr)/vel_range(i);
    
    delta(1,i)=dA+ug(1)*(vel_range(i)^2/Radius);
    delta(2,i)=dA+ug(2)*(vel_range(i)^2/Radius);
    delta(3,i)=dA+ug(3)*(vel_range(i)^2/Radius);
    
    Ndelta(1,i)=a(1)*cf;
    Ndelta(2,i)=a(2)*cf;
    Ndelta(3,i)=a(3)*cf;
    
    veh_stab_moment(1,i)=Nbeta(1,i)*beta(1,i)+Nr(1,i)*r(1,i);
    veh_stab_moment(2,i)=Nbeta(2,i)*beta(2,i)+Nr(2,i)*r(2,i);
    veh_stab_moment(3,i)=Nbeta(3,i)*beta(3,i)+Nr(3,i)*r(3,i);
    
    veh_cont_moment(1,i)=Ndelta(1,i)*delta(1,i);
    veh_cont_moment(2,i)=Ndelta(2,i)*delta(2,i);
    veh_cont_moment(3,i)=Ndelta(3,i)*delta(3,i);
end

figure(2)
plot(vel_range,veh_stab_moment(1,:),'r','LineWidth',1.2)
hold on
plot(vel_range,veh_stab_moment(2,:),'b','LineWidth',1.2)
plot(vel_range,veh_stab_moment(3,:),'k','LineWidth',1.2)
grid on; grid minor;
xlabel('Velocity (m/s)')
ylabel('Vehicle Stabilizing moment (Nm)');
title('Theoretical vehicle stabilizing moment v/s velocity')
legend('@60% a/l','@50% a/l','@40% a/l','Location','Best')

figure(3)
plot(veh_stab_moment(1,:),veh_cont_moment(1,:),'r','LineWidth',1.2)
hold on
plot(veh_stab_moment(2,:),veh_cont_moment(2,:),'bo','LineWidth',1.2)
plot(veh_stab_moment(3,:),veh_cont_moment(3,:),'k','LineWidth',1.2)
grid on; grid minor;
xlabel('Vehicle Stabilizing moment (Nm)')
ylabel('Vehicle Control moment (Nm)');
title('Vehicle control moment v/s vehicle stabilizing moment')
legend('@60% a/l','@50% a/l','@40% a/l','Location','Best')
delta_d=delta*rad2deg(1);

%% mid term Part 5
x_1=-0.07*exp(-2.88*0.04)+0.07*exp(-34.68*0.04);
x_2=-0.07*exp(-2.88*0.04*2)+0.07*exp(-34.68*0.04*2);
a_1=-0.07*(2.88^2)*exp(-2.88*0.04*0)+0.07*(34.68^2)*exp(-34.68*0.04*0);
a_2=-0.07*(2.88^2)*exp(-2.88*0.04)+0.07*(34.68^2)*exp(-34.68*0.04);
a_3=-0.07*(2.88^2)*exp(-2.88*0.04*2)+0.07*(34.68^2)*exp(-34.68*0.04*2);

