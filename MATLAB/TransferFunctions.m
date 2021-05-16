%% Transfer Functions Analysis
% Constants
W = 4.742065; % Kg
g   = 9.80665;       % m/ss
N   = 150;           % Gear Ratio
KI  = 0.0603;        % Nm/A
Ks  = 104;           % Nm/rad
Ka  = Ks/20;         % Nm/rad (???)
% Ka  = 1387.6;
Jh  = 0.0437;        % Kg.m^2   (check KneeJointParameters.m)
Le  = 0.432;          % m      (???)
Je  = W*Le^2;        % Kg.m^2 
We  = W*g;           % N      
Beq = 60;            % N.m.s/rad (maybe is 30, I.O.)
Ja  = 0.47;          % Kg.m^2

% My leg lenght is about 44 cm
% My body mass  is about 62 Kg
I_zz = 0.048*62*(0.433*0.28)^2;

dt = 0.001;
CURRENT_MAX = pi;

zeta = 0.06;
Kp_acc = 0.5436;
Ki_acc = 11.560;
epos_Ki = 1.190; 
epos_Kp = 11.900;

%% User leg natural frequency estimation:
clc, close all
% What if...
KneeRoM = 80; % deg
% passKneeStiff = passiveKneeStiffness;
% isoKneeStiff = isometricKneeStiffness(2);
% Ka - Je*w_n^2 > 0 -> Ka/je > w_n^2 -> Ka/Je > Kk/I_zz, then:
% kneeStiffness < Ka/Je * I_zz
Ka = 104*63;
kneeStiffness_max = Ka/Je * I_zz;
w_max = sqrt(Ka/Je);
%For 1.53 Hz hopping the knee stiffness is:
K_j = @(o) 110.0 + 169.3*deg2rad(o - 0);
Eta = @(o) Ka./(110.0 + 169.3*deg2rad(o - 0));
w_j_des = @(o) sqrt(110.0/I_zz + 169.3*deg2rad(o - 0)/I_zz);

offset = 10;
ang = linspace(-offset, KneeRoM-offset, 300);
figure, plot(ang, w_j_des(ang)), grid on, axis tight
title('Knee \omega_n using first order variable stiffness')
ylabel('\omega_n (Hz)'), xlabel('Leg flexion (deg)')

figure, plot(ang, K_j(ang)), hold on
plot(ang, Eta(ang)), grid on
yline(kneeStiffness_max,'--',{'k_k^{max}'})
legend('k_{knee}','\eta')

P  = @(n) -Je + n.*I_zz;
I  = @(n) 2*zeta*sqrt(n.*Ka*I_zz);

figure,
plot(ang, P(Eta(ang))), hold on
plot(ang, I(Eta(ang))), grid on
title('PI tuning from knee dynamics')
legend('K_p','K_i'), xlabel('Leg flexion (deg)'), axis tight

% Using the Impedance (F/X) Transfer Function (Xa = 0):
Z_r = @(tht) tf([P(Eta(tht)) I(Eta(tht)) 0],[(Je/Ka) 0 1]);
Z_h = @(tht) tf([K_j(tht)],[1]);
A_r = @(tht) tf([(Je/Ka) 0 1], [P(Eta(tht)) I(Eta(tht)) 0]);
A_h = @(tht) tf([1],[K_j(tht)]);

% Using the Impedance (F/X) Transfer Function (DX = 0):
Z_act = tf([-Ja 0 0],[(Je/Ka) 0 1]);
A_act = 1/Z_act;

ang_step = 30;
ang1 = -5;
ang2 = ang1 +   ang_step;
ang3 = ang1 + 2*ang_step;

figure, 
bode(Z_r(ang1), Z_h(ang1), Z_r(ang2), Z_h(ang2), Z_r(ang3), Z_h(ang3)), grid on
hold on, bode(tf([Ka*P(Eta(ang1))/Je],[1]),'--k')
bode(Z_act,'--b')
title('Impedance Bode')

legend(['Z_i(',num2str(ang1),')'],['Z_k(',num2str(ang1),')'],...
       ['Z_i(',num2str(ang2),')'],['Z_k(',num2str(ang2),')'],...
       ['Z_i(',num2str(ang3),')'],['Z_k(',num2str(ang3),')'])

figure, 
bode(A_r(ang1), A_h(ang1), A_r(ang2), A_h(ang2), A_r(ang3), A_h(ang3)), grid on
hold on, bode(tf([1],[Ka*P(Eta(ang1))/Je]),'--k')
bode(A_act,'--b')
title('Admittance Bode')

legend(['A_i(',num2str(ang1),')'],['A_k(',num2str(ang1),')'],...
       ['A_i(',num2str(ang2),')'],['A_k(',num2str(ang2),')'],...
       ['A_i(',num2str(ang3),')'],['A_k(',num2str(ang3),')'])

   
% MISO transfer function:
% Z_all = [Z_i(ang2) Z_act];
% figure, bode(Z_all), grid on

%%
% Check signal Z = -F/V...
% Using the Impedance (F/V) Transfer Function (Xa = 0):
Z_r = @(tht) tf([P(Eta(tht)) I(Eta(tht))],[(Je/Ka) 0 1]);
Z_h = @(tht) tf([K_j(tht)],[1 0]);
A_r = @(tht) tf([(Je/Ka) 0 1], [P(Eta(tht)) I(Eta(tht))]);
A_h = @(tht) tf([1 0],[K_j(tht)]);

% Using the Impedance (F/V) Transfer Function (DX = 0):
Z_act = tf([-Ja 0],[(Je/Ka) 0 1]);
A_act = 1/Z_act;

%{
figure, 
bode(A_r(ang1), A_h(ang1), A_r(ang2), A_h(ang2), A_r(ang3), A_h(ang3)), grid on
% xline(w_max,'--r',{'\omega_{max}'})
title('Exo and Knee Admittance')

legend(['A_r(',num2str(ang1),')'],['A_h(',num2str(ang1),')'],...
       ['A_r(',num2str(ang2),')'],['A_h(',num2str(ang2),')'],...
       ['A_r(',num2str(ang3),')'],['A_h(',num2str(ang3),')'])
%}

figure, 
bode(Z_r(ang1), Z_h(ang1), Z_r(ang2), Z_h(ang2), Z_r(ang3), Z_h(ang3)), grid on
% xline(w_max,'--r',{'\omega_{max}'})
title('Exo and Knee Impedance')

legend(['Z_r(',num2str(ang1),')'],['Z_h(',num2str(ang1),')'],...
       ['Z_r(',num2str(ang2),')'],['Z_h(',num2str(ang2),')'],...
       ['Z_r(',num2str(ang3),')'],['Z_h(',num2str(ang3),')'])
   
% Limitations (using 25 deg):
figure, bode(Z_r(ang2)), hold on
bode(tf([I(Eta(ang2))],[1]),'--k')  % s -> 0
bode(Z_h(ang2))                     % Des Imp
bode(tf([Je 0],[1]),'--g')          % Pure Mass Imp
bode(tf([Ks],[1 0]),'--r')          % Pure SEA Imp
bode(tf([Ka],[1 0]),'--m')          % Pure Ka Imp
grid on
legend(['Z_r(',num2str(ang2),')'],...
        'Z_r(s\rightarrow0)',...
        ['Z_h(',num2str(ang2),')'],...
        'J_e','K_{sea}','K_a')
    
% ver efeito de Ka e Je sobre a impedancia...

%% Symbolic math to obtain s-domain roots
clc
% clear all

syms s J_a J_e K_I k_s k_a K_p K_i S D g l W_e

Cv = K_I*(K_p + K_i/s);
Adm = (1 - S/k_s)*s / (s*D + S);
G = J_a*s^2 + k_s*(1 + Cv*Adm);
Den = G + Cv*s

% Find roots with respect to s:
% Sol = solve(Den,s)
factors = factor(Den,s);

% Note from factors(2) that S must be > 0

s_roots = solve(factors(1),s,'MaxDegree',3);

s1 = latex(simplify(s_roots(1)));
s2 = latex(simplify(s_roots(2)));
s3 = latex(simplify(s_roots(3)));

%%

Cv = KI*(epos_Kp + epos_Ki/s);
Adm = (1 - S/Ks)*s / (s*D + S);
G = Ja*s^2 + Ks*(1 + Cv*Adm);
Den = G + Cv*s

% Find roots with respect to s:
% Sol = solve(Den,s)
factors = factor(Den,s);

% Note from factors(3) that S must be > 0

s_roots = solve(factors(2),s,'MaxDegree',3);

s1_adm = s_roots(1);
s2_adm = s_roots(2);
s3_adm = s_roots(3);

% fplot(subs(s1_adm, D, Ks/10)) % caution!