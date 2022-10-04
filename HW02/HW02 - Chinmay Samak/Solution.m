% Load data
load('tireF15.mat')

% Preprocess ET array for 0 deg. inclination angle, 12 psi inflation pressure and 25 mph roadway speed
IA_0 = 0; P_12 = 12*6.895; V_25 = 25*1.609344;
ET_preprocess = [];
for i = 1:length(ET)
    if abs(IA(i)-IA_0)<=0.25
        if abs(P(i)-P_12)<=10
            if abs(V(i)-V_25)<=10
                ET_preprocess(length(ET_preprocess)+1) = i;
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
% Polyfit
FYSA050_poly = polyfit(SA_050_lbs,FY_050_lbs,11);
FYSA100_poly = polyfit(SA_100_lbs,FY_100_lbs,11);
FYSA150_poly = polyfit(SA_150_lbs,FY_150_lbs,11);
FYSA250_poly = polyfit(SA_250_lbs,FY_250_lbs,11);
FYSA350_poly = polyfit(SA_350_lbs,FY_350_lbs,11);
% Plot FY vs SA
figure(1)
plot(SA_050_lbs,FY_050_lbs, '.','color','red')
hold on
plot(SA_100_lbs,FY_100_lbs, '.','color','magenta')
plot(SA_150_lbs,FY_150_lbs, '.','color','blue')
plot(SA_250_lbs,FY_250_lbs, '.','color','#4DBEEE')
plot(SA_350_lbs,FY_350_lbs, '.','color','green')
plot(SA_050_lbs,polyval(FYSA050_poly,SA_050_lbs),'color','red')
plot(SA_100_lbs,polyval(FYSA100_poly,SA_100_lbs),'color','magenta')
plot(SA_150_lbs,polyval(FYSA150_poly,SA_150_lbs),'color','blue')
plot(SA_250_lbs,polyval(FYSA250_poly,SA_250_lbs),'color','#4DBEEE')
plot(SA_350_lbs,polyval(FYSA350_poly,SA_350_lbs),'color','green')
xlabel("SA (deg)")
ylabel("FY (N)")
legend("Raw Data @ |FZ| = 50 lbs","Raw Data @ |FZ| = 100 lbs","Raw Data @ |FZ| = 150 lbs","Raw Data @ |FZ| = 250 lbs","Raw Data @ |FZ| = 350 lbs", ...
       "Polyfit @ |FZ| = 50 lbs","Polyfit @ |FZ| = 100 lbs","Polyfit @ |FZ| = 150 lbs","Polyfit @ |FZ| = 250 lbs","Polyfit @ |FZ| = 350 lbs", ...
       'Location','NE')
xlim([0, 15])
ylim([-4000, 0]) 
set(gca, 'YDir','reverse')
hold off

%% Question 2
% Polyder
d_FYSA050_poly = polyder(FYSA050_poly);
d_FYSA100_poly = polyder(FYSA100_poly);
d_FYSA150_poly = polyder(FYSA150_poly);
d_FYSA250_poly = polyder(FYSA250_poly);
d_FYSA350_poly = polyder(FYSA350_poly);
% Polyval at SA = 0 deg
c_050_lbs = abs(polyval(d_FYSA050_poly, 0));
c_100_lbs = abs(polyval(d_FYSA100_poly, 0));
c_150_lbs = abs(polyval(d_FYSA150_poly, 0));
c_250_lbs = abs(polyval(d_FYSA250_poly, 0));
c_350_lbs = abs(polyval(d_FYSA350_poly, 0));
% Table c vs FZ
FZ_values = [FZ_050_lbs FZ_100_lbs FZ_150_lbs FZ_250_lbs FZ_350_lbs];
c_values = [c_050_lbs  c_100_lbs  c_150_lbs  c_250_lbs  c_350_lbs];
fprintf("---------------------\n")
fprintf("FZ (N)\t\tc (N/deg)\n")
fprintf("---------------------\n")
for i=1:length(FZ_values)
    fprintf("%.4f\t%.4f\n", FZ_values(i), c_values(i))
end
fprintf("---------------------\n\n")
% Plot c vs FZ
figure(2)
plot(FZ_values, c_values,'-x','color','red')
xlabel("FZ (N)")
ylabel("c (N/deg)")
set(gca, 'XDir','reverse')

%% Question 3
% u = abs(peak(FY)/FZ)
u_050_lbs = abs(min(FY_050_lbs)/FZ_050_lbs);
u_100_lbs = abs(min(FY_100_lbs)/FZ_100_lbs);
u_150_lbs = abs(min(FY_150_lbs)/FZ_150_lbs);
u_250_lbs = abs(min(FY_250_lbs)/FZ_250_lbs);
u_350_lbs = abs(min(FY_350_lbs)/FZ_350_lbs);
% Table u vs FZ
FZ_values = [FZ_050_lbs FZ_100_lbs FZ_150_lbs FZ_250_lbs FZ_350_lbs];
u_values = [u_050_lbs  u_100_lbs  u_150_lbs  u_250_lbs  u_350_lbs];
fprintf("---------------------\n")
fprintf("FZ (N)\t\tu\n")
fprintf("---------------------\n")
for i=1:length(FZ_values)
    fprintf("%.4f\t%.4f\n", FZ_values(i), u_values(i))
end
fprintf("---------------------\n\n")
% Plot u vs FZ
figure(3)
plot(FZ_values, u_values,'-x','color','red')
xlabel("FZ (N)")
ylabel("u")
set(gca, 'XDir','reverse')

%% Question 4
% Polyfit
MZSA050_poly = polyfit(SA_050_lbs,MZ_050_lbs,11);
MZSA100_poly = polyfit(SA_100_lbs,MZ_100_lbs,11);
MZSA150_poly = polyfit(SA_150_lbs,MZ_150_lbs,11);
MZSA250_poly = polyfit(SA_250_lbs,MZ_250_lbs,11);
MZSA350_poly = polyfit(SA_350_lbs,MZ_350_lbs,11);
% Plot MZ vs SA
figure(4)
plot(SA_050_lbs,MZ_050_lbs, '.','color','red')
hold on
plot(SA_100_lbs,MZ_100_lbs, '.','color','magenta')
plot(SA_150_lbs,MZ_150_lbs, '.','color','blue')
plot(SA_250_lbs,MZ_250_lbs, '.','color','#4DBEEE')
plot(SA_350_lbs,MZ_350_lbs, '.','color','green')
plot(SA_050_lbs,polyval(MZSA050_poly,SA_050_lbs),'color','red')
plot(SA_100_lbs,polyval(MZSA100_poly,SA_100_lbs),'color','magenta')
plot(SA_150_lbs,polyval(MZSA150_poly,SA_150_lbs),'color','blue')
plot(SA_250_lbs,polyval(MZSA250_poly,SA_250_lbs),'color','#4DBEEE')
plot(SA_350_lbs,polyval(MZSA350_poly,SA_350_lbs),'color','green')
xlabel("SA (deg)")
ylabel("MZ (N-m)")
legend("Raw Data @ |FZ| = 50 lbs","Raw Data @ |FZ| = 100 lbs","Raw Data @ |FZ| = 150 lbs","Raw Data @ |FZ| = 250 lbs","Raw Data @ |FZ| = 350 lbs", ...
       "Polyfit @ |FZ| = 50 lbs","Polyfit @ |FZ| = 100 lbs","Polyfit @ |FZ| = 150 lbs","Polyfit @ |FZ| = 250 lbs","Polyfit @ |FZ| = 350 lbs", ...
       'Location','NE')
xlim([0, 15])
ylim([0, 100])
hold off

%% Question 5
% Preprocess ET array further for 0 deg. slip angle and 0 N lateral force
SA_0 = 0; % deg
FY_0 = 0; % N
ET_process = [];
for i = 1:length(ET_preprocess)
    if abs(SA(i)-SA_0)<1
        if abs(FY(i)-FY_0)<=250
            ET_process(length(ET_process)+1) = i;
        end
    end
end
% Polyfit
FZRL_poly = polyfit(RL(ET_process),FZ(ET_process),1);
% Polyder
slope = polyder(FZRL_poly);
fprintf("Vertical spring rate = %.4f N/cm @ FZ = 200 lbs\n", slope)
% Plot FZ vs RL
figure(5)
plot(RL(ET_process),FZ(ET_process), '.','color','red')
hold on
plot(RL(ET_process),polyval(FZRL_poly,RL(ET_process)),'color','red')
xlabel("RL (cm)")
ylabel("FZ (N)")
legend("Raw Data", "Polyfit")
set(gca, 'YDir','reverse')
hold off