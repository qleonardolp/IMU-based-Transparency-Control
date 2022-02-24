%% Mixed Sensitivity study on the transparency problem
close all
clearvars
clc
% Constants
Ts = 0.008;   % s
kp = 15.00;
g   = 9.80665;       % m/ss
K_s  = 104;          % Nm/rad
K_a  = K_s/20;       % Nm/rad
J_h  = 0.0511;       % Kg.m^2 from "10.1109/ACCESS.2019.2927515" (doi)
B_h  = 3.5;          % N.m s/rad
K_h  = 38;           % N.m/rad
J_r  = 4.742065*0.432^2; % Kg.m^2      
B_s = 60;            % N.m.s/rad
B_a = 3.0;

% Dynamic parameters:
wn_h = sqrt(K_h/J_h);
zeta_h = B_h/(2*sqrt(J_h*K_h));
wn_g = sqrt(K_a/J_r);
zeta_g = B_a/(2*wn_g);

s = tf('s');

% With acc feedforward
G2 = 0*s;
G1 = 1/(J_r*s + B_a + K_a/s);
Gd = 1/(J_h*s + B_h + K_h/s);
% Without acc feedforward
Gd_noff = 1/(J_h*s + B_h + K_h/s)*((B_a + K_a/s)/(J_r*s + B_a + K_a/s));
K0 = kp + kp*Ts/s; % Original acc-PI

Zatt_high = -B_a/(J_r*s);
psigma = 3*K_a/B_a; % closed-loop dominant pole | 1/T (time constant)
zeta = 0.7;
omega = wn_h;
% psigma = 2*zeta*omega - K_a/B_a;
alpha = 4; % era 2.0
Gain = B_a*wn_h*alpha; % omega em rad/s
Lambda_1 = (J_r*s^2 + B_a*s + K_a)/(K_a + B_a*s);

% Integral Transparency Controller:
% K0 = Lambda_1*Gain/(s + psigma)/(s);

% Pole Placement (sisotool):
% K0 = 24.168*(s^2 + 52.47*s + 700.2)/s;

% Lag Transparency Controller:
DC = 10^(73.86/20);
N = 2;
wp = .034; wz = 14;
Lag = (s + wz)/(s + wp)*(wp/wz);
K0 = Lambda_1*(Lag^N)*10^(73.86/20)*wz/(s + wz);

% "Notchs Filter":
wz = 5;
peak = 1;
damp = 0.4;
P = peak^2/(s^2 + 2*damp*peak*s + peak^2);
V = (s^2 + 2*damp*wn_g*s + wn_g^2)/s^2;
Dc = (s/(s + peak));
K0 = .5*Lambda_1*P*V;
K1 = .23*Lambda_1*V*wz/(s + wz);


% "Notch 2":
alpha = 10;
wp = wn_g*2; wz = wp*alpha;
Lag = (s + wz)/(s + wp)*(wp/wz);
wz = 80; wp = wz*alpha;
Lead = (s + wz)/(s + wp)*(wp/wz);
Notch = Lag*Lead;
pole = wn_g/1.37;
V = (s^2 + 2*0.05*wn_h*s + wn_h^2)/s^2*(s/(s+wn_h))^2;
K0 = Lambda_1*18/s^2*(s + pole)/pole*V; % USAR COMO REFERENCIA DE PERFORMANCE

% Simple approach...
wz = 4.3;
wp = 0.5;
K1 = 10/((s+.2))*(s+0.4)/s*(s+0.4)/s

% Transparency Definition:
TransparencyTF = -(K_a + B_a*s)/(J_r*s^2 + B_a*s + K_a)*K0;
[Gm,Pm,Wcg,Wcp] = margin(K0*G1);
Gm
Pm

% name = 'Feedforward effect on Disturbance';
% bodeb(name,{0.01,1e3},Gd,'b', Gd_noff,'--r')
% legend('Gd(ff)','Gd')

% Podemos analisar com rlocus tbm...
% Ls = -s/(K_a + s*B_a)*Gain/(s + psigma);
% L_high = Gain/(B_a*s);
% figure, bodemag(Ls, L_high,{1e-3,1e3})
% hold on, xline(wn_h/(2*pi))
% grid on
adj_p = 229.6;
% K1 = 5880.5*(s^2 + 3.39*s + 5.876) / ( (s+adj_p)*(s+0.178)*(s+0.1694) );

%% Mixed Sensitivity synthesis
%{
% Using Zh(s)-- Ah(s) definition.
% Admittance band attenuation:
omg_upper = 1000/4;
% desired disturbance attenuation inside bandwidth:
A_s = 1/(norm(freqresp(Gd,0.01*wn_h)));
A_t = 1/(norm(freqresp(Gd,omg_upper,'Hz')));

% Weighting functions:(Skogestad pg 62)
M = 2; % sensitivity max peak
omg_s = wn_h;
omg_t = omg_upper*2*pi;

W1 = makeweight(A_s,omg_s,1/M);
W3 = makeweight(1/M,omg_t,A_t);
W2 = [];

% Reprojeto:
% W1 = makeweight(10,[wn_h 1],0.01);
% W3 = makeweight(0.01,[4*wn_h 1],10);
% W3 = 0.01*W3;
 
name = 'Weighting Functions';
bodemagb(name,{1e-2,1e6},W1,'r',W3,'b')
%bodemag(W1,'r',W3,'b',Gd,'--k',W1*Gd,'--r')
legend('W1','W3', 'Location','SouthWest')

[Kinf, CLinf, gamma] = mixsyn(G1, W1, W2, W3);

[AK, BK, CK, DK] = ssdata(Kinf);
[nK, dK] = ss2tf(AK, BK, CK, DK);
K1 = tf(nK, dK);

T0 = feedback(K0*G1,1);
S0 = 1 - T0;
T = feedback(K1*G1, 1);
S = 1 - T;

% sigma(S, 'b', T, 'r', gamma/W1, 'b-.', gamma/W3, 'r-.', {1e-5,1e5})
% hold on, sigma(S0,'g--', T0, 'm--')
name = 'Singular Values';
bodemagb(name,{1e-6,1e6},S,'b',T,'r',gamma/W1,'b-.',gamma/W3,'r-.',...
         S0,'g--',T0,'m--', Gd, 'k--')
legend('S', 'T','\gamma/W1', '\gamma/W3','Si','Ti','Gd','Location','SouthWest')
grid on
%}
%% Closed Loop and Disturbance step responses:
T0 = feedback(K0*G1,1);
S0 = 1 - T0;
T = feedback(K1*G1, 1);
S = 1 - T;

t_end = 0.3*1e2; %segundos
name = 'Step Responses';
stepb(name,t_end, T,'b',T0,'b--',S*Gd,'r',S0*Gd,'r--')
hold on
yline(0.632,'--'), yline(0.95,'--')

% disturbance rms:
% [dist_k1, ] = step(S*Gd,t_end);
% D1_rms = rms(dist_k1);
% [dist_k0, ] = step(S0*Gd,t_end);
% D0_rms = rms(dist_k0);
% [D0_rms D1_rms]
name = 'Complementary Sensitivity';
bodeb(name,{1e-4,1e4}, T,'b', T0,'b--')
legend('K1','K0')

% Transparency TF eh Gamma(s) no artigo:
Gamma0 = -K0/Lambda_1;
Gamma1 = -K1/Lambda_1;
GammaMin = -1/Lambda_1;
bodeb('Transparency',{1e-4,1e4},Gamma1,'b',Gamma0,'r',GammaMin,'r--')
hold on, xline(0.01*wn_h,'--'), xline(wn_h,'--')
legend('K1','K0')

% Forca de Interacao: (for paper only... 'simulation')
% t = 0:0.01:5;
% u = sin(wn_h*t);
% figure, lsim(Gamma1,'b',Gamma0,'r',u,t)
% hold on
% lsim(GammaMin,'k--',u,t)
% grid on

% Transparency metric:
wl = log10(0.01*wn_h);
wu = log10(5*wn_h); 
[mag, phs, w] = bode(Gamma0, {1e-4,1e4});
% [mag_ctrl,~,~] = bode(K0, {1e-4,1e4});
w = log10(w);
kl = find(abs(w - wl) < 0.01);
ku = find(abs(w - wu) < 0.05);
tau_gamma0 = trapz( mag(:,:,kl:ku) )/(10^wu - 10^wl);
[mag, phs, w] = bode(Gamma1, {1e-4,1e4});
% [mag_ctrl,~,~] = bode(K1, {1e-4,1e4});
w = log10(w);
kl = find(abs(w - wl) < 0.05);
ku = find(abs(w - wu) < 0.05);
tau_gamma1 = trapz( mag(:,:,kl:ku) )/(10^wu - 10^wl);
[tau_gamma0 tau_gamma1]

%% Controller simplification and analysis (robustness):
%{
disp('Mixed Sensitivity K(s):')
zpk(K1)
disp('Integral Transparency K(s):')
zpk(K0)

[Z,P,K] = zpkdata(K1);
Z_new = [Z{1}(2) Z{1}(3)]';
P_new = [P{1}(2) P{1}(3) P{1}(4)]';

[knum, kden] = zp2tf(Z_new, P_new, K);
K1_new = tf(knum, kden);
K1_z = tf(knum, kden, 0.001, 'Variable','z^-1');

L = K1_new*G1;
T = feedback(L, 1);
S = 1 - T;
%}
%% Disk Margins:
%{
DM1 = diskmargin(G1,K1_new);
DM0 = diskmargin(G1,K0);
figure('Name','Range of gain and phase variations'), 
diskmarginplot([DM0.GainMargin; DM1.GainMargin]), title('')
%}