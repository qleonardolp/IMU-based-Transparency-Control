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
Kadm = .104;
Dadm = 0.03;
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
   
%%
close all

Zr = tf([Kp_acc Ki_acc],[(Je/Ka) 0 1])
Cv = KI*tf([epos_Kp epos_Ki],[1 0]);
As = (1 - 10.4/Ks)*tf([1],[10.36 10.4]);
Num = tf([1 0],[1])*Cv + (Cv*As*Je - Ja)*tf([1 0 0], [1]);  % deducao no caderno
Zadm = Ka*Num*tf([1], [Ja 0 0])
% Limitations (using 25 deg Knee Stiffness):
figure('Name','Impedance Bode Diagram','Color',[1 1 1]),
ax = subplot(2,1,1);    % configure Mag

bd_opt = bodeoptions;
bd_opt.PhaseVisible = 'off';
bd_opt.Title.String = '';
bd_opt.XLabel.Color = [1 1 1];  % sumir com o eixo X...

bodeplot(Zr, bd_opt); hold on
bodeplot(Zadm, bd_opt);
bodeplot(tf([I(Eta(ang2))],[1]),'--k',bd_opt)  % s -> 0
bodeplot(Z_h(ang2), bd_opt)                     % Des Imp
bodeplot(tf([Je 0],[1]),'--g', bd_opt)          % Pure Mass Imp
bodeplot(tf([Ks],[1 0]),'--r', bd_opt)          % Pure SEA Imp
bodeplot(tf([Ka],[1 0]),'--m', bd_opt)          % Pure Ka Imp

ax.FontSize = 12; ax.LineWidth = 0.8; ax.GridAlpha = 0.6;
grid on

ax = subplot(2,1,2);    % configure Phase

bd_opt = bodeoptions;
bd_opt.MagVisible = 'off';
bd_opt.Title.String = '';
bd_opt.XLabel.FontSize = 12;

bodeplot(Zr, bd_opt); hold on
bodeplot(Zadm, bd_opt);
bodeplot(tf([I(Eta(ang2))],[1]),'--k',bd_opt)  % s -> 0
bodeplot(Z_h(ang2), bd_opt)                     % Des Imp
bodeplot(tf([Je 0],[1]),'--g', bd_opt)          % Pure Mass Imp
bodeplot(tf([Ks],[1 0]),'--r', bd_opt)          % Pure SEA Imp
bodeplot(tf([Ka],[1 0]),'--m', bd_opt)          % Pure Ka Imp

ax = gca;
ax.FontSize = 12; ax.LineWidth = 0.8; ax.GridAlpha = 0.6;
grid on

legend('Z_{r}', 'Z_{adm}', 'Z_{r}(0)', 'Z_h', 'Z_{J_e}','Z_{K_s}','Z_{K_a}',...
        'Orientation','horizontal','FontSize',12)

fig = gcf; 
axes_handle = fig.Children;

axes_handle(3).String(1) = {'Z_{r}'};
axes_handle(3).String(2) = {'Z_{adm}'};
axes_handle(3).String(2) = {'Z_{adm}'};
axes_handle(3).String(3) = {'Z_{r}(0)'};
axes_handle(3).String(4) = {'Z_{h}'};
axes_handle(3).String(4) = {'Z_{h}'};
axes_handle(3).String(5) = {'Z_{Je}'};
axes_handle(3).String(5) = {'Z_{Je}'};
axes_handle(3).String(6) = {'Z_{Ks}'};
axes_handle(3).String(6) = {'Z_{Ks}'};
axes_handle(3).String(7) = {'Z_{Ka}'};
axes_handle(3).String(7) = {'Z_{Ka}'};

for i = 4:7
    for k = 1:6
        axes_handle(i).Children(k).Children.LineWidth = 1.0;
    end
end    
    
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

%% Impedance/Admittance Transfer Functions Analysis (Loop Shaping, Complementary Stability)
Kp_acc = 0.54;
Ki_acc = 11.560;
Kadm = 25.0;
Dadm = .14;
Is = 34.00;
Id = 0.08;

Kpf = 0.0030;
Kdf = 0.0005;

% Check Feedforward
% Kp_acc = 0;
% Ki_acc = 0;

Je  = W*Le^2; % muito pesado
% Je = 0.5*Jh;

s = tf('s');
Cv = (epos_Kp + epos_Ki/s);
Adm = (1 - Kadm/Ks)*s / (s*Dadm + Kadm);

w_z = 0.1;
w_p = w_z*1e-3;
LagComp = (w_p/w_z)*tf([1 w_z],[1 w_p]);
w_z = 100;
w_p = w_z*1e1;
LeadComp = (w_p/w_z)*tf([1 w_z],[1 w_p]);
figure,
bode(LagComp*LeadComp, LeadComp), grid on

% Das deduções de Zh realizadas em MTC e ATC, e da Zh desejada do usuário:
% FT do erro de Zh:
Em = Ki_acc + (Je + Kp_acc)*s %MTC
Ea = Cv*(Adm*Je*s + 1)        %ATC
Ei = Je*s + Is/s + Id         %ITC
Ef = -(Kpf + s*Kdf)*(Ka/s)          %FTC (PD)
% Ef = -(s*Kpf + s*s*Kdf)*(Ka/s)    %FTC ("Ideal")
% Ef = -(LeadComp)*(Ka/s)           %FTC (Nao funciona)


Zh = Jh*s;
Zm = Zh - Em;
Za = Zh - Ea;
Zi = Zh - Ei;
Zf = Zh - Ef;


% figure,
% bode(Em, Ea, Ef), grid on
% legend('Em', 'Ea', 'Ef')

figure,
bode(Zm, Za, Zi, Zf, Zh), grid on
legend('Zm', 'Za', 'Zi', 'Zf', 'Zh')

% Condições de Transparência e Passividade
%MTC: Ki > 0,  0 < Kp < 0.5*Jh - Je => (Je < 0.5*Jh) 
%%
Kadm = 2.0;
Dadm = 12;
C2 = (1 - Kadm/Ks) / (0.0012*Kadm + Dadm)
C1 = Dadm / (Dadm + Kadm*0.0012)
