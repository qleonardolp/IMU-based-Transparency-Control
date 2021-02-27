%% State Space formulation for Transparency Control with Rotary SEA.
% velHum as a control input
% Leonardo Felipe L. S. dos Santos, 2020, EESC@USP

% Constants
W = 4.742065; % Kg
g   = 9.80665;       % m/ss
N   = 150;           % Gear Ratio
KI  = 0.0603;        % Nm/A
Ks  = 104;           % Nm/rad
Ka  = Ks/20;          % Nm/rad (???)
Jh  = 0.05;          % Kg.m^2 (???)
Le  = 0.40;          % m      (???)
Je  = W*Le^2;        % Kg.m^2 
We  = W*g;           % N      
Beq = 60;            % N.m.s/rad (maybe is 30, I.O.)
Ja  = 0.47;          % Kg.m^2

% State vector is  [x_h x_e x_a \dot{x_e} \dot{x_a}]
% Sensor vector is [\dot{x_h} x_e x_m \dot{x_e} \dot{x_m}]
state_dim  = 5;
sensor_dim = 5;
control_dim = 3;

% State Space matrices

A = [zeros(1,state_dim); ...
    0 0 0 1 0; ...
    0 0 0 0 1; ...
    Ka/Je -(Ks + Ka)/Je Ks/Je 0 0; ...
    0 Ks/Ja -Ks/Ja 0 -Beq/Ja];

B = [1 0 0; zeros(2,control_dim); 0 -1/Je 0; 0 0 N*KI/Ja];

C = [eye(sensor_dim)]; C(1,1) = 0; C(3,3) = N; C(5,5) = N;
D = zeros(sensor_dim, control_dim); D(1,1) = 1;

poles = eig(A)  % check open-loop stability

% Check Controllability & Observability:
if (rank(ctrb(A,B)) == state_dim) && (rank(obsv(A,C)) == state_dim)
    disp('System is controllable and observable.')
elseif (rank(ctrb(A,B)) == state_dim) && (rank(obsv(A,C)) ~= state_dim)
    disp('System is controllable and not observable.')
elseif (rank(ctrb(A,B)) ~= state_dim) && (rank(obsv(A,C)) == state_dim)
    disp('System is not controllable and is observable.')
else
    disp('System is neither controllable and observable.')
end
%% State Space definition, 
% considering the different sensors 
% sample rate as output delay:
imu_delay = 8; % [ms]
Sys = ss(A,B,C,D,'TimeUnit','milliseconds','OutputDelay',[imu_delay 0 imu_delay 0 0]');
% Discretization
Ts = 1; % [ms]
% sea_bw_freq = 9.6/1000; % [kHz]
% disOpt = c2dOptions('Method','tustin','PrewarpFrequency',2*pi*sea_bw_freq);
% discreteSys = c2d(Sys,Ts,disOpt)
discreteSys = c2d(Sys,Ts)

%% Control Design, CAC
epos_Ki = 1.190; epos_Kp = 11.900;

damping_d = 0.900;
stiffness_d = 3.874;
stiffness_lower = 3.8422;

eposVelocityController = pid(epos_Kp,epos_Ki,0,0,0.001);

s = tf('s')

CAC = (1 - stiffness_d/Ks)*s / (stiffness_d + s*damping_d);
CAC = c2d(CAC,0.001);

stiffness_lower = damping_d*(epos_Ki/epos_Kp - damping_d/(Ja*(1 - stiffness_d/Ks)) - epos_Kp/Ja)

%% Tuning from the characteristic equation (acceleration-based)

w_n = 2.165;      % Hz
T_s = 4/(1*w_n);  % s

w_max = sqrt(Ka/Je);
% From the Canonical form:
Kp_acc = (Ka - Je*w_n^2)/(w_n^2);
% Forcing critically damped response (zeta = 1):
Ki_acc = 2*sqrt((Kp_acc + Je)*Ka);

fprintf("Setting time: %.4f\nMax natural freq: %.4f\nKp: %.4f\nKi: %.4f\n",T_s,w_max,Kp_acc,Ki_acc);

%% Frequency Response tuning
clc, close all

% Kp_acc = 0.3507;
% Kp_acc = 0.0243;
w_n = 2.165;  % Hz
zeta = 0.01;
% From the Canonical form:
Kp_acc = (Ka - Je*w_n^2)/(w_n^2);
Ki_acc = 2*zeta*sqrt((Kp_acc + Je)*Ka);

f = logspace(-2,2,1e4);
m = 2;

% s^2 + (Ki/(Kp + Je))*s + Ka/(Kp + Je)     (Eq)
% Re = w_n^2 - w^2
% Im = 2*zeta *w_n *w
Re = @(w,Kp) Ka./(Kp + Je) - w.^2;
Im = @(w,Kp) (2*zeta*sqrt((Kp + Je)*Ka)./(Kp + Je)).*w;
P  = @(w) (Ka - Je*w.^2)./(w.^2);

Re_w0 = (Ka/(Kp_acc + Je));
% semilogx(f, 20*log10(Re_w0)*ones(1,length(f)),'--k')
%{
figure, semilogx(logspace(-1,1,1000), P(logspace(-1,1,1000))), hold on
xline(w_max,'--r',{'\omega_{max}'})
ylabel('K_p'), xlabel('Damped Frequency (Hz)'), grid on
%}

% Varying Kp
Cpx  = Re(f, Kp_acc)     + j*Im(f,Kp_acc);
Cpx2 = Re(f, Kp_acc/m)   + j*Im(f,Kp_acc/m);
Cpx3 = Re(f, Kp_acc/m^2) + j*Im(f,Kp_acc/m^2);
Cpx4 = Re(f, Kp_acc*m)   + j*Im(f,Kp_acc*m);
Cpx5 = Re(f, Kp_acc*m^2) + j*Im(f,Kp_acc*m^2);

w_PM = f( min(find(abs(abs(Cpx)-1) <= 0.007)) );      % Phase Margin freq
w_GM = f( min(find(angle(Cpx) > 0.995*pi)) );         % Gain Margin freq

PM = pi - angle(Re(w_PM, Kp_acc) + j*Im(w_PM,Kp_acc));
PM = rad2deg(PM);

GM = abs(Re(w_GM, Kp_acc) + j*Im(w_GM,Kp_acc));
GM = -20*log10(GM);

figure,
subplot(2,1,1)
semilogx(f, 20*log10(abs(Cpx3))), hold on
semilogx(f, 20*log10(abs(Cpx2)))
semilogx(f, 20*log10(abs(Cpx)))
semilogx(f, 20*log10(abs(Cpx4)))
semilogx(f, 20*log10(abs(Cpx5)))
xline(w_max,'--r',{'\omega_{max}'})
xline(1.00,'--b')
xline(w_GM,'--k',{'GM'})
ylabel('Mag (dB)'), 
title(['Frequency Response (Kp varying | ','\zeta = ',num2str(zeta), ')']), grid on % using absolute scale
legend('Kp/m^2','Kp/m','Kp','Kp*m','Kp*m^2'), axis tight

subplot(2,1,2)
semilogx(f, rad2deg(angle(Cpx3)))
hold on
semilogx(f, rad2deg(angle(Cpx2)))
semilogx(f, rad2deg(angle(Cpx)))
semilogx(f, rad2deg(angle(Cpx4)))
semilogx(f, rad2deg(angle(Cpx5)))
xline(w_max,'--r')
xline(1.00,'--b')
xline(w_PM,'--k',{'PM'})
ylabel('Phase (deg)'), xlabel('Frequency (Hz)'), grid on
legend(['GM ',num2str(GM),'dB'],['PM ',num2str(PM),'°']), legend('boxoff'), axis tight
%% Change w and zeta:
Re_w = @(Kp,w) Ka./(Kp + Je) - w.^2;
Im_w = @(Kp,w,d) (2.*d*sqrt((Kp + Je)*Ka)./(Kp + Je)).*w;

Kp_ = logspace(-3,3,1e4);
zeta = 0.3;

Cpx = Re_w(Kp_,w_max) + 1i*(Im_w(Kp_,w_max,zeta));
Cpx2 = Re_w(Kp_,2) + 1i*(Im_w(Kp_,2,zeta));
Cpx3 = Re_w(Kp_,1) + 1i*(Im_w(Kp_,1,zeta));

figure,
subplot(2,3,1)
semilogx(Kp_, 20*log10(abs(Cpx))), hold on
semilogx(Kp_, 20*log10(abs(Cpx2))), semilogx(Kp_, 20*log10(abs(Cpx3)))
xline(Kp_acc,'--b',{'K_p'})
ylabel('Mag (dB)'), title(['\zeta = ',num2str(zeta)]), grid on
legend('\omega_{max}','2.00','1.00'), axis tight

subplot(2,3,4)
semilogx(Kp_, rad2deg(angle(Cpx))), hold on
semilogx(Kp_, rad2deg(angle(Cpx2))), semilogx(Kp_, rad2deg(angle(Cpx3)))
xline(Kp_acc,'--b')
ylabel('Phase (deg)'), grid on, axis tight

zeta = 0.1;
Cpx = Re_w(Kp_,w_max) + 1i*(Im_w(Kp_,w_max,zeta));
Cpx2 = Re_w(Kp_,2) + 1i*(Im_w(Kp_,2,zeta));
Cpx3 = Re_w(Kp_,1) + 1i*(Im_w(Kp_,1,zeta));

subplot(2,3,2)
semilogx(Kp_, 20*log10(abs(Cpx))), hold on
semilogx(Kp_, 20*log10(abs(Cpx2))), semilogx(Kp_, 20*log10(abs(Cpx3)))
xline(Kp_acc,'--b',{'K_p'})
title(['\zeta = ',num2str(zeta)]), grid on, axis tight

subplot(2,3,5)
semilogx(Kp_, rad2deg(angle(Cpx))), hold on
semilogx(Kp_, rad2deg(angle(Cpx2))), semilogx(Kp_, rad2deg(angle(Cpx3)))
xline(Kp_acc,'--b')
xlabel('Kp'), grid on, axis tight

zeta = 0.01;
Cpx = Re_w(Kp_,w_max) + 1i*(Im_w(Kp_,w_max,zeta));
Cpx2 = Re_w(Kp_,2) + 1i*(Im_w(Kp_,2,zeta));
Cpx3 = Re_w(Kp_,1) + 1i*(Im_w(Kp_,1,zeta));

subplot(2,3,3)
semilogx(Kp_, 20*log10(abs(Cpx))), hold on
semilogx(Kp_, 20*log10(abs(Cpx2))), semilogx(Kp_, 20*log10(abs(Cpx3)))
xline(Kp_acc,'--b',{'K_p'})
title(['\zeta = ',num2str(zeta)]), grid on, axis tight

subplot(2,3,6)
semilogx(Kp_, rad2deg(angle(Cpx))), hold on
semilogx(Kp_, rad2deg(angle(Cpx2))), semilogx(Kp_, rad2deg(angle(Cpx3)))
xline(Kp_acc,'--b'), grid on, axis tight