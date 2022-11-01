% Load data
load('tireF15.mat')
% Convert SA values to radians
SA = deg2rad(SA);
% Preprocess ET array for 0 deg. inclination angle, 12 psi inflation pressure,
% 25 mph roadway speed and positive slip angles (i.e., left turn data only)
IA_0 = 0; P_12 = 12*6.895; V_25 = 25*1.609344;
ET_preprocess = [];
for i = 1:length(ET)
    if abs(IA(i)-IA_0)<=0.25
        if abs(P(i)-P_12)<=10
            if abs(V(i)-V_25)<=10
                if SA(i) > 0
                    ET_preprocess(length(ET_preprocess)+1) = i;
                end
            end
        end
    end
end
% Preprocess FZ array into different groups
FZ_050_lbs = -50*4.448;
FZ_100_lbs = -100*4.448;
FZ_150_lbs = -150*4.448;
FZ_250_lbs = -250*4.448;
FZ_300_lbs = -300*4.448;
FZ_350_lbs = -350*4.448;
FZ_050_preprocess = find(abs(FZ)>100 & abs(FZ)<300);
FZ_100_preprocess = find(abs(FZ)>350 & abs(FZ)<550);
FZ_150_preprocess = find(abs(FZ)>550 & abs(FZ)<750);
FZ_250_preprocess = find(abs(FZ)>1000 & abs(FZ)<1200);
FZ_300_preprocess = find(abs(FZ)>1200 & abs(FZ)<1450);
FZ_350_preprocess = find(abs(FZ)>1450 & abs(FZ)<1650);
% Segregate data based on normal load values
FY_050_lbs=[];FY_100_lbs=[];FY_150_lbs=[];FY_250_lbs=[];FY_300_lbs=[];FY_350_lbs=[];
MZ_050_lbs=[];MZ_100_lbs=[];MZ_150_lbs=[];MZ_250_lbs=[];MZ_300_lbs=[];MZ_350_lbs=[];
SA_050_lbs=[];SA_100_lbs=[];SA_150_lbs=[];SA_250_lbs=[];SA_300_lbs=[];SA_350_lbs=[];
for i = 1:length(ET_preprocess)
    if ismember(ET_preprocess(i), FZ_050_preprocess)
        FY_050_lbs(length(FY_050_lbs)+1) = FY(ET_preprocess(i));
        MZ_050_lbs(length(MZ_050_lbs)+1) = MZ(ET_preprocess(i));
        SA_050_lbs(length(SA_050_lbs)+1) = SA(ET_preprocess(i));
    elseif ismember(ET_preprocess(i), FZ_100_preprocess)
        FY_100_lbs(length(FY_100_lbs)+1) = FY(ET_preprocess(i));
        MZ_100_lbs(length(MZ_100_lbs)+1) = MZ(ET_preprocess(i));
        SA_100_lbs(length(SA_100_lbs)+1) = SA(ET_preprocess(i));
    elseif ismember(ET_preprocess(i), FZ_150_preprocess)
        FY_150_lbs(length(FY_150_lbs)+1) = FY(ET_preprocess(i));
        MZ_150_lbs(length(MZ_150_lbs)+1) = MZ(ET_preprocess(i));
        SA_150_lbs(length(SA_150_lbs)+1) = SA(ET_preprocess(i));
    elseif ismember(ET_preprocess(i), FZ_250_preprocess)
        FY_250_lbs(length(FY_250_lbs)+1) = FY(ET_preprocess(i));
        MZ_250_lbs(length(MZ_250_lbs)+1) = MZ(ET_preprocess(i));
        SA_250_lbs(length(SA_250_lbs)+1) = SA(ET_preprocess(i));
    elseif ismember(ET_preprocess(i), FZ_300_preprocess)
        FY_300_lbs(length(FY_300_lbs)+1) = FY(ET_preprocess(i));
        MZ_300_lbs(length(MZ_300_lbs)+1) = MZ(ET_preprocess(i));
        SA_300_lbs(length(SA_300_lbs)+1) = SA(ET_preprocess(i));
    elseif ismember(ET_preprocess(i), FZ_350_preprocess)
        FY_350_lbs(length(FY_350_lbs)+1) = FY(ET_preprocess(i));
        MZ_350_lbs(length(MZ_350_lbs)+1) = MZ(ET_preprocess(i));
        SA_350_lbs(length(SA_350_lbs)+1) = SA(ET_preprocess(i));
    end
end

%% Question 1
% u values (u = abs(peak(FY)/FZ))
u_050_lbs = abs(min(FY_050_lbs)/FZ_050_lbs);
u_100_lbs = abs(min(FY_100_lbs)/FZ_100_lbs);
u_150_lbs = abs(min(FY_150_lbs)/FZ_150_lbs);
u_250_lbs = abs(min(FY_250_lbs)/FZ_250_lbs);
u_350_lbs = abs(min(FY_350_lbs)/FZ_350_lbs);
% c values (derivative of FY vs SA at 0)
FYSA050_poly = polyfit(SA_050_lbs,FY_050_lbs,7);
FYSA100_poly = polyfit(SA_100_lbs,FY_100_lbs,7);
FYSA150_poly = polyfit(SA_150_lbs,FY_150_lbs,7);
FYSA250_poly = polyfit(SA_250_lbs,FY_250_lbs,7);
FYSA350_poly = polyfit(SA_350_lbs,FY_350_lbs,7);
d_FYSA050_poly = polyder(FYSA050_poly);
d_FYSA100_poly = polyder(FYSA100_poly);
d_FYSA150_poly = polyder(FYSA150_poly);
d_FYSA250_poly = polyder(FYSA250_poly);
d_FYSA350_poly = polyder(FYSA350_poly);
c_050_lbs = polyval(d_FYSA050_poly, 0);
c_100_lbs = polyval(d_FYSA100_poly, 0);
c_150_lbs = polyval(d_FYSA150_poly, 0);
c_250_lbs = polyval(d_FYSA250_poly, 0);
c_350_lbs = polyval(d_FYSA350_poly, 0);
% Fbar values (Fbar = FY/u*FZ) (nondimensional)
Fbar_050_lbs = FY_050_lbs/(u_050_lbs*FZ_050_lbs);
Fbar_100_lbs = FY_100_lbs/(u_100_lbs*FZ_100_lbs);
Fbar_150_lbs = FY_150_lbs/(u_150_lbs*FZ_150_lbs);
Fbar_250_lbs = FY_250_lbs/(u_250_lbs*FZ_250_lbs);
Fbar_350_lbs = FY_350_lbs/(u_350_lbs*FZ_350_lbs);
Fbar = [Fbar_050_lbs,Fbar_100_lbs,Fbar_150_lbs,Fbar_250_lbs,Fbar_350_lbs];
% Abar values (Abar = c*tan(SA)/u*FZ) (nondimensional)
Abar_050_lbs = (c_050_lbs*tan(SA_050_lbs))/(u_050_lbs*FZ_050_lbs);
Abar_100_lbs = (c_100_lbs*tan(SA_100_lbs))/(u_100_lbs*FZ_100_lbs);
Abar_150_lbs = (c_150_lbs*tan(SA_150_lbs))/(u_150_lbs*FZ_150_lbs);
Abar_250_lbs = (c_250_lbs*tan(SA_250_lbs))/(u_250_lbs*FZ_250_lbs);
Abar_350_lbs = (c_350_lbs*tan(SA_350_lbs))/(u_350_lbs*FZ_350_lbs);
Abar = [Abar_050_lbs,Abar_100_lbs,Abar_150_lbs,Abar_250_lbs,Abar_350_lbs];
% Plot Fbar vs Abar
figure(1)
plot(Abar_050_lbs,Fbar_050_lbs,'.','color','red')
hold on
plot(Abar_100_lbs,Fbar_100_lbs,'.','color','magenta')
plot(Abar_150_lbs,Fbar_150_lbs,'.','color','blue')
plot(Abar_250_lbs,Fbar_250_lbs,'.','color','#4DBEEE')
plot(Abar_350_lbs,Fbar_350_lbs,'.','color','green')
xlabel('$\overline{\alpha}$','Interpreter','Latex')
ylabel('$\overline{F}$','Interpreter','Latex')
legend("Raw Data @ |FZ| = 50 lbs","Raw Data @ |FZ| = 100 lbs","Raw Data @ |FZ| = 150 lbs","Raw Data @ |FZ| = 250 lbs","Raw Data @ |FZ| = 350 lbs",'Location','SE')
hold off

%% Question 2
% Magic formula
B = 0.6000;
C = 1.6667;
D = 1.0000;
E = 0.2000;
Fbar_magic = D*sin(C*atan((B*Abar)-E*((B*Abar)-atan((B*Abar)))));
% Plot Fbar vs Abar
figure(2)
plot(Abar,Fbar_magic, '-','color','black')
hold on
plot(Abar_050_lbs,Fbar_050_lbs,'.','color','red')
plot(Abar_100_lbs,Fbar_100_lbs,'.','color','magenta')
plot(Abar_150_lbs,Fbar_150_lbs,'.','color','blue')
plot(Abar_250_lbs,Fbar_250_lbs,'.','color','#4DBEEE')
plot(Abar_350_lbs,Fbar_350_lbs,'.','color','green')
xlabel('$\overline{\alpha}$','Interpreter','Latex')
ylabel('$\overline{F}$','Interpreter','Latex')
legend("Magic Formula","Raw Data @ |FZ| = 50 lbs","Raw Data @ |FZ| = 100 lbs","Raw Data @ |FZ| = 150 lbs","Raw Data @ |FZ| = 250 lbs","Raw Data @ |FZ| = 350 lbs",'Location','SE')
hold off

%% Question 3
% Segmented polyfit to Fbar vs Abar regions (linear, quadratic and flat)
FbarAbar_line_poly = polyfit(Abar(Abar>=0 & Abar<=0.4),Fbar(Abar>=0 & Abar<=0.4),1); % Linear region
FbarAbar_quad_poly = polyfit(Abar(Abar>=1.5 & Abar<=2.7),Fbar(Abar>=1.5 & Abar<=2.7),2); % Quadratic region
FbarAbar_flat_poly = mean(Fbar(Abar>=8 & Abar<=9)); % Flat region
% Derive Pacejka's coefficients
D = max(polyval(FbarAbar_quad_poly,Abar))
ys = polyval(FbarAbar_flat_poly,max(Abar));
C = 1+(1-((2/pi)*asin(ys/D)))
BCD = polyval(polyder(FbarAbar_line_poly),0);
B = BCD/(C*D)
xm = fzero(@(x) polyval(polyder(FbarAbar_quad_poly), x), 0);
E = ((B*xm)-tan(pi/(2*C)))/((B*xm)-atan(B*xm))
ys_check = D*sin((pi*C)/2);
% Plot Fbar vs Abar
figure(3)
plot(Abar,Fbar_magic,'-','color','black') % Magic formula
hold on
plot(Abar(Abar>=0.0 & Abar<=0.4),polyval(FbarAbar_line_poly,Abar(Abar>=0.0 & Abar<=0.4)),'-','color','red') % Actual linear fit
plot(Abar(Abar>=0.4 & Abar<=1.1),polyval(FbarAbar_line_poly,Abar(Abar>=0.4 & Abar<=1.1)),'.','color','red')
plot(Abar(Abar>=1.5 & Abar<=2.7),polyval(FbarAbar_quad_poly,Abar(Abar>=1.5 & Abar<=2.7)),'-','color','green') % Actual quadratic fit
plot(Abar(Abar>=0.8 & Abar<=1.5),polyval(FbarAbar_quad_poly,Abar(Abar>=0.8 & Abar<=1.5)),'.','color','green')
plot(Abar(Abar>=2.7 & Abar<=5.0),polyval(FbarAbar_quad_poly,Abar(Abar>=2.7 & Abar<=5.0)),'.','color','green')
plot(Abar(Abar>=8.0 & Abar<=9.0),polyval(FbarAbar_flat_poly,Abar(Abar>=8.0 & Abar<=9.0)),'-','color','blue') % Actual flat fit
plot(Abar(Abar>=4.2 & Abar<=8.0),polyval(FbarAbar_flat_poly,Abar(Abar>=4.2 & Abar<=8.0)),'.','color','blue')
plot(xm,D,'*','color','#7E2F8E')
plot(max(Abar),ys,'*','color','#EDB120')
xlabel('$\overline{\alpha}$','Interpreter','Latex')
ylabel('$\overline{F}$','Interpreter','Latex')
legend("Magic Formula","Linear Region Polyfit","","Quadratic Region Polyfit","","","Flat Region Polyfit","","Data Peak (xm,D)","Data Saturation (xs,ys)",'Location','SE')
hold off

%% Question 4
% FY values (FY = Fbar*u*FZ) (dimensional)
Fdim_050_lbs = Fbar_050_lbs*u_050_lbs*FZ_050_lbs;
Fdim_100_lbs = Fbar_100_lbs*u_100_lbs*FZ_100_lbs;
Fdim_150_lbs = Fbar_150_lbs*u_150_lbs*FZ_150_lbs;
Fdim_250_lbs = Fbar_250_lbs*u_250_lbs*FZ_250_lbs;
Fdim_350_lbs = Fbar_350_lbs*u_350_lbs*FZ_350_lbs;
% Plot measured and predicted values
figure(4)
plot(SA_050_lbs,FY_050_lbs,'.','color','red')
hold on
plot(SA_100_lbs,FY_100_lbs,'.','color','magenta')
plot(SA_150_lbs,FY_150_lbs,'.','color','blue')
plot(SA_250_lbs,FY_250_lbs,'.','color','#4DBEEE')
plot(SA_350_lbs,FY_350_lbs,'.','color','green')
plot(SA_050_lbs,Fdim_050_lbs,'o','color','red')
plot(SA_100_lbs,Fdim_100_lbs,'o','color','magenta')
plot(SA_150_lbs,Fdim_150_lbs,'o','color','blue')
plot(SA_250_lbs,Fdim_250_lbs,'o','color','#4DBEEE')
plot(SA_350_lbs,Fdim_350_lbs,'o','color','green')
xlabel('SA (rad)')
ylabel('FY (N)')
legend("Measured Data @ |FZ| = 50 lbs","Measured Data @ |FZ| = 100 lbs","Measured Data @ |FZ| = 150 lbs","Measured Data @ |FZ| = 250 lbs","Measured Data @ |FZ| = 350 lbs", ...
       "Predicted Data @ |FZ| = 50 lbs","Predicted Data @ |FZ| = 100 lbs","Predicted Data @ |FZ| = 150 lbs","Predicted Data @ |FZ| = 250 lbs","Predicted Data @ |FZ| = 350 lbs", ...
       'Location','NW')
set(gca,'YDir','reverse')
ylim([-4250, 0])
hold off

%% Question 5
% FZ values
FZ_200_lbs = -200*4.448;
FZ_300_lbs = -300*4.448;
FZ_400_lbs = -400*4.448;
% u values
uFZ_poly = polyfit([FZ_050_lbs FZ_100_lbs FZ_150_lbs FZ_250_lbs FZ_350_lbs], ...
                   [u_050_lbs u_100_lbs u_150_lbs u_250_lbs u_350_lbs],3);
u_200_lbs = polyval(uFZ_poly,FZ_200_lbs);
u_300_lbs = polyval(uFZ_poly,FZ_300_lbs);
u_400_lbs = polyval(uFZ_poly,FZ_400_lbs);
% FY values (FY = Fbar*u*FZ) (dimensional)
Fdim_200_lbs = Fbar*u_200_lbs*FZ_200_lbs;
Fdim_300_lbs = Fbar*u_300_lbs*FZ_300_lbs;
Fdim_400_lbs = Fbar*u_400_lbs*FZ_400_lbs;
% Plot measured and predicted values (for given as well as unknown data)
figure(5)
plot(SA_050_lbs,FY_050_lbs,'.','color','red')
hold on
plot(SA_100_lbs,FY_100_lbs,'.','color','magenta')
plot(SA_150_lbs,FY_150_lbs,'.','color','blue')
plot(SA_250_lbs,FY_250_lbs,'.','color','#4DBEEE')
plot(SA_350_lbs,FY_350_lbs,'.','color','green')
plot(SA_050_lbs,Fdim_050_lbs,'o','color','red')
plot(SA_100_lbs,Fdim_100_lbs,'o','color','magenta')
plot(SA_150_lbs,Fdim_150_lbs,'o','color','blue')
plot(SA_250_lbs,Fdim_250_lbs,'o','color','#4DBEEE')
plot(SA_350_lbs,Fdim_350_lbs,'o','color','green')
plot([SA_050_lbs,SA_100_lbs,SA_150_lbs,SA_250_lbs,SA_350_lbs],Fdim_200_lbs,'o','color','#000000')
plot([SA_050_lbs,SA_100_lbs,SA_150_lbs,SA_250_lbs,SA_350_lbs],Fdim_300_lbs,'o','color','#EDB120')
plot([SA_050_lbs,SA_100_lbs,SA_150_lbs,SA_250_lbs,SA_350_lbs],Fdim_400_lbs,'o','color','#7E2F8E')
xlabel('SA (rad)')
ylabel('FY (N)')
legend("Measured Data @ |FZ| = 50 lbs","Measured Data @ |FZ| = 100 lbs","Measured Data @ |FZ| = 150 lbs","Measured Data @ |FZ| = 250 lbs","Measured Data @ |FZ| = 350 lbs", ...
       "Predicted Data @ |FZ| = 50 lbs","Predicted Data @ |FZ| = 100 lbs","Predicted Data @ |FZ| = 150 lbs","Predicted Data @ |FZ| = 250 lbs","Predicted Data @ |FZ| = 350 lbs", ...
       "Predicted Data @ |FZ| = 200 lbs","Predicted Data @ |FZ| = 300 lbs","Predicted Data @ |FZ| = 400 lbs", ...
       'Location','NW')
set(gca,'YDir','reverse')
ylim([-4250, 0])
hold off