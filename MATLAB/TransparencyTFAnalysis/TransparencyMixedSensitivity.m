%% Mixed Sensitivity study on the transparency problem
close all
clearvars
clc
% CONSTANTS:
s = tf('s');
g   = 9.80665;       % m/ss
K_s  = 104;          % Nm/rad
% % ATC and MTC params
preK = 2.5;
Kp = 100.0;
Kd = 0.0225;
stiffness_d = K_s/50;
damping_d   = 0.1;

K_a  = K_s/20;       % Nm/rad
J_h  = 0.0511;       % Kg.m^2 from "10.1109/ACCESS.2019.2927515" (doi)
B_h  = 3.5;          % N.m s/rad
K_h  = 38;           % N.m/rad
J_r  = 4.742065*0.432^2; % Kg.m^2      
B_s = 60;            % N.m.s/rad
B_a = 3.0;           % N.m.s/rad

% Dynamic parameters:
wn_h = sqrt(K_h/J_h);
zeta_h = B_h/(2*sqrt(J_h*K_h));
wn_g = sqrt(K_a/J_r);
zeta_g = B_a/(2*wn_g);

% % System definitions:
% With acc feedforward
G2 = 0*s;
G1 = 1/(J_r*s + B_a + K_a/s);
Gd = 1/(J_h*s + B_h + K_h/s);
% Without acc feedforward
Gd_noff = 1/(J_h*s + B_h + K_h/s)*((B_a + K_a/s)/(J_r*s + B_a + K_a/s));
Lambda_ = (J_r*s^2 + B_a*s + K_a)/(K_a + B_a*s);

% % Integral Transparency Controller:
% % Zatt_high = -B_a/(J_r*s);
% closed-loop dominant pole | 1/T (time constant):
% psigma = 3*K_a/B_a; 
% zeta = 0.7;
% omega = wn_h;
% psigma = 2*zeta*omega - K_a/B_a;
% alpha = 4; % era 2.0
% Gain = B_a*wn_h*alpha; % omega em rad/s
% K0 = Lambda_1*Gain/(s + psigma)/(s);

% % Pole Placement (sisotool):
% K0 = 24.168*(s^2 + 52.47*s + 700.2)/s;

% % Lag Transparency Controller:
% DC = 10^(73.86/20);
% N = 2;
% wp = .034; wz = 14;
% Lag = (s + wz)/(s + wp)*(wp/wz);
% K0 = Lambda_1*(Lag^N)*10^(73.86/20)*wz/(s + wz);

% % "Notch Filter v2":
% alpha = 10;
% wp = wn_g*2; wz = wp*alpha;
% Lag = (s + wz)/(s + wp)*(wp/wz);
% wz = 80; wp = wz*alpha;
% Lead = (s + wz)/(s + wp)*(wp/wz);
% Notch = Lag*Lead;
% pole = wn_g/1.37;
% V = (s^2 + 2*0.05*wn_h*s + wn_h^2)/s^2*(s/(s+wn_h))^2;
% K0 = Lambda_*18/s^2*(s + pole)/pole*V; % USAR COMO REFERENCIA DE PERFORMANCE

% % Simple approach...
% wz = 4.3;
% wp = 0.5;
% K1 = 10/((s+.2))*(s+0.4)/s*(s+0.4)/s

FeedbackPD = Kp + Kd*s;
Admittance = (1 - stiffness_d/K_s)*s/(s*damping_d + stiffness_d);
EPOS = 11.9 + 1.19/s;
ATC = EPOS*Admittance*FeedbackPD;
MTC = FeedbackPD;
K0 = ATC;
K1 = MTC;

% Transparency Definition:
TransparencyTF = -K0/Lambda_;
[Gm,Pm,Wcg,Wcp] = margin(K0*G1);
Gm
Pm
% name = 'Feedforward effect on Disturbance';
% bodeb(name,{0.01,1e3},Gd,'b', Gd_noff,'--r')
% legend('Gd(ff)','Gd')

%% Closed Loop and Disturbance step responses:
T0 = feedback(K0*G1,1);
S0 = 1 - T0;
T = feedback(K1*G1, 1);
S = 1 - T;

% % Step Responses (Ref and Dis)
% t_end = 0.3*1e2; %segundos
% name = 'Step Responses';
% stepb(name,t_end, T0,'b',T,'b--',S0*Gd,'r',S*Gd,'r--')
% hold on
% yline(0.632,'--'), yline(0.95,'--')
% legend('ATC','MTC')

freq_range = {1e-3,1e3};

name = 'Sensitivity';
bodemagb(name,freq_range, T0,'b', T,'r', S0,'b--', S,'r--')
legend('$T_{ATC}$','$T_{MTC}$','$S_{ATC}$','$S_{MTC}$',...
       'Interpreter','latex')

% Transparency TF eh Gamma(s) no artigo:
Gamma0 = -K0/Lambda_;
Gamma1 = -K1/Lambda_;
GammaMin = -1/Lambda_;
bodeb('Transparency',freq_range,Gamma0,'b',Gamma1,'r',GammaMin,'k--')
legend('$\Gamma_{ATC}$','$\Gamma_{MTC}$','$\Gamma_{min}$',...
       'Interpreter','latex')
    

%% Parameters Analysis:
EPOS = 11.9 + 1.19/s;
FeedbackPD = 100.0 + 0.0225*s;
stiffness_d = K_s/2;
Admittance = (1 - stiffness_d/K_s)*s/(s*0.1 + stiffness_d);
K0 = EPOS*Admittance*FeedbackPD;
stiffness_d = K_s/10;
Admittance = (1 - stiffness_d/K_s)*s/(s*0.1 + stiffness_d);
K1 = EPOS*Admittance*FeedbackPD;
stiffness_d = K_s/100;
Admittance = (1 - stiffness_d/K_s)*s/(s*0.1 + stiffness_d);
K2 = EPOS*Admittance*FeedbackPD;

FeedbackPD = 100.0 + 2.00*s;
K3 = FeedbackPD;
FeedbackPD = 100.0 + 0.20*s;
K4 = FeedbackPD; 
FeedbackPD = 100.0 + 0.0225*s;
K5 = FeedbackPD;

% close all
clc

controllers = [K0 K1 K2];
%{
name = 'Sensitivity';
figure('Name', name, 'Color',[1 1 1])
for i = 1:3
    T = feedback(controllers(i)*G1, 1);
    S = 1 - T;
    color = [(255-80*(i-1)) 10*i 80*(i)]/255;
%     color = [(255-60*(i-1)) 10*i 80*(i)]/255;
    bodemaghold(name,freq_range, T,color,'-', S,color,'-.')
    hold on
end
legend('$k_{s}^d = 52.0$','$k_{s}^d = 52.0$',...
       '$k_{s}^d = 10.4$','$k_{s}^d = 10.4$',...
       '$k_{s}^d = 1.04$','$k_{s}^d = 1.04$',...
       'Interpreter','latex','NumColumns',3)
hold off

controllers = [K3 K4 K5];
name = 'Sensitivity';
figure('Name', name, 'Color',[1 1 1])
for i = 1:3
    T = feedback(controllers(i)*G1, 1);
    S = 1 - T;
    color = [(255-80*(i-1)) 10*i 80*(i)]/255;
%     color = [(255-60*(i-1)) 10*i 80*(i)]/255;
    bodemaghold(name,freq_range, T,color,'-', S,color,'-.')
    hold on
end
legend('$k_d = 2.00$','$k_d = 2.00$',...
       '$k_d = 0.20$','$k_d = 0.20$',...
       '$k_d = 0.02$','$k_d = 0.02$',...
       'Interpreter','latex','NumColumns',3,'Color','none')
hold off
%}
% Transparency TF eh Gamma(s) no artigo:
Gamma0 = -K0/Lambda_;
Gamma1 = -K1/Lambda_;
Gamma2 = -K2/Lambda_;
Gamma3 = -K3/Lambda_;
Gamma4 = -K4/Lambda_;
Gamma5 = -K5/Lambda_;

Gamma = [Gamma0 Gamma1 Gamma2 Gamma3 Gamma4 Gamma5];
name = 'Transparency';
figure('Name', name, 'Color',[1 1 1])
for i = 1:6
    color = [(255-51*(i-1)) 10*i 51*(i)];
    color = color/max(color);
    linestyle = '-';
    if (i == 7)
        linestyle = '-.';
        color = [0 0 0];
    elseif (i > 3)
        linestyle = '--';
    end
    bodemaghold(name,freq_range,Gamma(i),color,linestyle)
    hold on
end
ylabel ('\textbf{$|F_i(s)/V_h(s)|$ (dB)}','Interpreter','latex')
legend('$k_{Y} = 52.0$','$k_{Y} = 10.4$','$k_{Y} = 1.04$',...
       '$d_C = 2.00$', '$d_C = 0.20$', '$d_C = 0.02$',...
       'Interpreter','latex','NumColumns',2)
hold off

%% Parameters Analysis (Only T(s)):
EPOS = 11.9 + 1.19/s;
FeedbackPD = 100.0 + 0.0225*s;
stiffness_d = K_s/2;
Admittance = (1 - stiffness_d/K_s)*s/(s*0.1 + stiffness_d);
K0 = EPOS*Admittance*FeedbackPD;
stiffness_d = K_s/10;
Admittance = (1 - stiffness_d/K_s)*s/(s*0.1 + stiffness_d);
K1 = EPOS*Admittance*FeedbackPD;
stiffness_d = K_s/100;
Admittance = (1 - stiffness_d/K_s)*s/(s*0.1 + stiffness_d);
K2 = EPOS*Admittance*FeedbackPD;

FeedbackPD = 100.0 + 2.00*s;
K3 = FeedbackPD;
FeedbackPD = 100.0 + 0.20*s;
K4 = FeedbackPD; 
FeedbackPD = 100.0 + 0.0225*s;
K5 = FeedbackPD;

close all
clc

freq_range = {5e-3,5e3};
plt_hndler = [];
controllers = [K0 K1 K2 K3 K4 K5];
name = 'Complementary Sensitivity';
figure('Name', name, 'Color',[1 1 1])
for i = 1:6
    T = feedback(controllers(i)*G1, 1);
    if (i <= 3)
        color = [(255-80*(i-1)) 10*i 80*(i)]/255;
        bodemaghold(name,freq_range, T,color,'-')
    else
        color = [(255-80*(i-4)) 10*(i-3) 80*(i-3)]/255;
        bodemaghold(name,freq_range, T,color,'-.')
    end
    hold on
end

legend('$k_{Y} = 52.0$','$k_{Y} = 10.4$','$k_{Y} = 1.04$',...
       '$d_C = 2.00$', '$d_C = 0.20$', '$d_C = 0.02$','Interpreter','latex','NumColumns',2)
hold off

%...