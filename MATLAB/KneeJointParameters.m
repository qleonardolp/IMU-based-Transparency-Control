%% Estimations based on:
%  _A Model of the Lower Limb for Analysis of Human Movement_ (2010)
%  note: knee spans from 0 to 100 deg.

% From Fig. 5.b, representing passive joint moments (N.m x deg):
p1 = [deg2rad(10)   30];
p2 = [deg2rad(102) -30];

passiveKneeStiffness = abs((p2(2) - p1(2))/(p2(1) - p1(1))) % delta_y/delta_x / Flexion-Extension

% From Fig. 7, representing maximum isometric knee joint moments (N.m x deg):
% Using the graph points with higher derivatives, the inclination can be
% described by:
isometricKneeStiffness = [0 0 0];

p1 = [deg2rad(54.2857)   150];
p2 = [deg2rad(100)    32.143];
isometricKneeStiffness(1) = abs((p2(2) - p1(2))/(p2(1) - p1(1)));   % at ~78 deg / Flexion
p1 = [deg2rad(0)     -157.143];
p2 = [deg2rad(7.143) -192.857];
isometricKneeStiffness(2) = abs((p2(2) - p1(2))/(p2(1) - p1(1)));   % at ~0 deg / Extension
p1 = [deg2rad(40)       -235.7143];
p2 = [deg2rad(84.2857)  -92.8571];
isometricKneeStiffness(3) = abs((p2(2) - p1(2))/(p2(1) - p1(1)));   % at ~62 deg / Extension

% From Fig. 7, using the range from 0 to 50 deg:
p1 = [deg2rad(0)   110.7143];
p2 = [deg2rad(60)  130.7143];
isometricKneeStiffness(1) = abs((p2(2) - p1(2))/(p2(1) - p1(1)));   % / Flexion
p1 = [deg2rad(1.4286)     -160.7143];
p2 = [deg2rad(37.1429)    -207.1429];
isometricKneeStiffness(2) = abs((p2(2) - p1(2))/(p2(1) - p1(1)));   % / Extension

% What about using a variable desired stiffness according to the flexion-extension angle?
% Check below...


%% _Constant and Variable Stiffness and Damping of the Leg Joints in Human Hopping_ (2003)
% Herein was suggested a first order variable stiffness with the joint
% angle and a constant damping. For 1.53 Hz hopping the knee means shows:
% K_j = k0_j + k1_j(p_j - p0_j)
% KneeStiffness(ang) = 110.0 + 169.3(ang - ang_0)
% KneeDampingRatio = 0.06 % (6%)

% This work suggests a linear variable knee stiffness. As the subject
% flexes the leg. Using the leg moment of inertia, the coupled-system
% natural frequency desired is
% w_n = sqrt(K_j/J_h) = sqrt( (k0_j + k1_j(p_j - p0_j))/J_h )

%% _Adjustments to McConville et al. and Young et al. body segment inertial parameters_ (2006)
% My leg lenght is about 44 cm
% My body mass  is about 62 Kg
body_mass  = 62;  % Kg

% According to this work, for a young male
% From Tab. 2, the scaling factor for I_zz is 28%. 
% The leg scale factor for mass is 4.8% and the leg length is 433 mm
L_leg = 0.433; % m
mass_scale = 0.048;
r_zz = 0.28;
I_zz = mass_scale*body_mass*(L_leg*r_zz)^2;
%% User leg natural frequency estimation:
clc, close all
% What if...
KneeRoM = 80; % deg
% passKneeStiff = passiveKneeStiffness;
% isoKneeStiff = isometricKneeStiffness(2);
% Ka - Je*w_n^2 > 0 -> Ka/je > w_n^2 -> Ka/Je > Kk/I_zz, then:
% kneeStiffness < Ka/Je * I_zz
Ka = 6500; Je = 0.8850;
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

% zeta = 0.025;
zeta = 0.06;
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
