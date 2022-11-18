%% Clear workspace
close all;
clear;
clc;

%% Given data
l = 3.1; % m
vc1 = 80/3.6; % m/s
vc2 = 120/3.6; % m/s
G = 19;
ay = 5; % m/s^2

%% Question A
UG1 = l/vc1^2;
UG2 = l/vc2^2;
fprintf('1A: UG should be between %f to %f rad.s^2.m^-1\n', UG2, UG1);

%% Question B
ss_max = vc2/(2*l);
fprintf('1B: Maximum steering sensitivity is %f s^-1\n', ss_max);

%% Question C
R1 = vc1^2/ay; % m
R2 = vc2^2/ay; % m
dA1 = l/R1; % rad
dA2 = l/R2; % rad
d1 = dA1+(UG1*ay); %rad
d2 = dA2+(UG2*ay); %rad
dsw1 = G*d1; % rad
dsw2 = G*d2; % rad
fprintf('1C: Steering wheel angles at 80 kph & 120 kph are %f & %f rad\n', dsw1, dsw2);