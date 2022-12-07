function [Fy]=nonlintire(alpha, Fz, vwx)

% by Matthias Schmid

% Nonlinear tire model based on Pacejka's Advanced Magic Formula from 'Tyre
% and Vehicle Dynamics', 2nd edition, chapter 4 by H.B. Pacejka
%
% Tire Data
% Size:             205/60R15 91V
% Pressure:         2.2 bar
% Sign Convention:  ISO (negative cornering stiffnesses)
% Nominal Radius:   R0 = 0.313 m
% Nominal Load:     Fz0 = 4000 N
% Mass:             m0 = 9.3 kg
% V0:               16.67 m/sec
%
% Adapted for slightly more pronounced peak characteristic and tire load
% sensitivity:
% Original Pcy1:    1.193
% Adapted Pcy1:     1.35
% Original Pdy2:    0.145
% Adapted Pdy2:     0.25
%
% Camber effects neglected
% No scaling factors
%
% Input Parameter:  slip angle (alpha) in radians, vertical force (Fz) in 
%                   Newton, longitudinal velocity of the wheel center (vwx)
%                   in m/sec
% Output Parameter: lateral force (Fy) in Newton
%

% Magic Formula Coefficients

    pcy1 = 1.35;
    pdy1 = -0.990;
    pdy2 = 0.25;
    pdy3 = -11.23;
    pey1 = -1.003;
    pey2 = -0.537;
    pey3 = -0.083;
    pey4 = -4.787;
    pky1 = -14.95;
    pky2 = 2.130;
    pky3 = -0.028;
    pky4 = 2;
    pky5 = 0;
    phy1 = 0.003;
    phy2 = -0.001;
    pvy1 = 0.045;
    pvy2 = -0.024;
    pvy3 = -0.532;
    pvy4 = 0.039;
    pky6 = -0.92;
    pky7 = -0.24;

% Vertical Load Deviation

    Fz0 = 4000;
    dfz = (Fz-Fz0)/Fz0;
    
% Adapted Slip for Large Angles    
    
    alpha_star = tan(alpha)*sign(vwx);    

% Vertical and Horizontal Shifts

    Svy = Fz*(pvy1 + pvy2*dfz);
    Shy = phy1 + phy2*dfz; 

% Magic Formula Parameter    
    
    x = alpha_star + Shy;
%    x = alpha_star;
    mu = (pdy1 + pdy2*dfz);
    
    C = pcy1;
    D = mu*Fz;            
    E = (pey1 + pey2 * dfz)*(1- pey3 *sign(x)); 
    BCD = pky1*Fz0*sin(pky4*atan(Fz/(pky2*Fz0)));  
    B = BCD/(C*D);

% Lateral Force    
    
    y = D*sin(C*atan(B*x-E*(B*x-atan(B*x))));

%   Fy = y;
    Fy = y + Svy;

end