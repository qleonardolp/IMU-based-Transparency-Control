%% State Space formulation for Transparency Control with Rotary SEA.
% velHum as a control input
% Leonardo Felipe L. S. dos Santos, 2020, EESC@USP
% clear all; close all; clc;
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

Threaded_Ts = 0.001;
CURRENT_MAX = pi;
%%
% State vector is  [x_h x_e x_a \dot{x_h} \dot{x_e} \dot{x_a}]
% Sensor vector is [tau_i \dot{x_h} x_e x_m \dot{x_e} \dot{x_m}]
t_i_idx = 1; v_h_idx = 2; p_e_idx = 3; p_m_idx = 4; v_e_idx = 5; v_m_idx = 6;
state_dim  = 6;
sensor_dim = 6;
control_dim = 3;

% State Space matrices

A = [0 0 0 1 0 0; ...
    0 0 0 0 1 0; ...
    0 0 0 0 0 1; ...
    -Ka/Jh Ka/Jh 0 0 0 0;...
    Ka/Je -(Ks + Ka)/Je Ks/Je 0 0 0; ...
    0 Ks/Ja -Ks/Ja 0 0 -Beq/Ja];

B = [zeros(3,control_dim); 1/Jh 0 0; 0 -1/Je 0; 0 0 N*KI/Ja];

C = [zeros(sensor_dim,state_dim)]; 
C(1,1) = -Ka; C(1,2) = Ka;
C(2,4) = 1;
C(3,2) = 1; C(4,3) = N; C(5,5) = 1; C(6,6) = N;
D = zeros(sensor_dim, control_dim);

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

% Discretization using Taylor expansion (F = exp(A*dt)):
dt = 0.001;
Fk = eye(state_dim) + A*dt + (A*dt)^2/prod(1:2) + (A*dt)^3/prod(1:3) + (A*dt)^4/prod(1:4);
Gk = dt*(eye(state_dim) + A*dt/prod(1:2) + (A*dt)^2/prod(1:3) + (A*dt)^3/prod(1:4) +...
        (A*dt)^4/prod(1:5))*B;
    
%% Kalman Estimation model:
% State vector is  [tau_i x_h x_e x_a \dot{x_e} \dot{x_a}]
% Sensor vector is [tau_i \dot{x_h} x_e x_m \dot{x_e} \dot{x_m}]
% Control vector is [\dot{x_h} tau_w I_m]
kf_ste_dim  = 6;
kf_snr_dim = 6;
kf_ctrl_dim = 3;

% State Space matrices

A_kf = [0 0 0 0 Ka 0; ...
    0 0 0 0 0 0; ...
    0 0 0 0 1 0; ...
    0 0 0 0 0 1; ...
    0 Ka/Je -(Ks + Ka)/Je Ks/Je 0 0; ...
    0 0 Ks/Ja -Ks/Ja 0 -Beq/Ja];

B_kf = [-Ka 0 0; 1 0 0; zeros(2,kf_ctrl_dim); 0 -1/Je 0; 0 0 N*KI/Ja];

C_kf = [zeros(kf_snr_dim,kf_ste_dim)]; 
C_kf(1,1) = 1;
C_kf(3,3) = 1; C_kf(4,4) = N;
C_kf(5,5) = 1; C_kf(6,6) = N;

D_kf = zeros(kf_snr_dim, kf_ctrl_dim); D_kf(2,1) = 1;

poles = eig(A_kf)  % check open-loop stability

% Discretization using Taylor expansion (F = exp(A*dt)):
dt = 0.001;
Fk = eye(kf_ste_dim) + A_kf*dt + (A_kf*dt)^2/prod(1:2) + (A_kf*dt)^3/prod(1:3) + (A_kf*dt)^4/prod(1:4);
Gk = dt*(eye(kf_ste_dim) + A_kf*dt/prod(1:2) + (A_kf*dt)^2/prod(1:3) + (A_kf*dt)^3/prod(1:4) +...
        (A_kf*dt)^4/prod(1:5))*B_kf;
    
% Check Controllability & Observability:
if (rank(ctrb(Fk,Gk)) == kf_ste_dim) && (rank(obsv(Fk,C_kf)) == kf_ste_dim)
    disp('System is controllable and observable.')
elseif (rank(ctrb(Fk,Gk)) == kf_ste_dim) && (rank(obsv(Fk,C_kf)) ~= kf_ste_dim)
    disp('System is controllable and not observable.')
elseif (rank(ctrb(Fk,Gk)) ~= kf_ste_dim) && (rank(obsv(Fk,C_kf)) == kf_ste_dim)
    disp('System is not controllable and is observable.')
else
    disp('System is neither controllable and observable.')
end 

% sysr = minreal(ss(A_kf, B_kf, C_kf, D_kf))

mtw_cov = 5.476e-6;
Qk = eye(kf_ste_dim);
Qk(1,1) = (Ka*2*8*sqrt(mtw_cov)*dt)^2;
Qk(2,2) = 1e-7;
Qk(3,3) = 1e-8;
Qk(4,4) = 1e-7;
Qk(5,5) = ( 1/Je*(Ka*(1e-7) + (Ka + Ks)*(1e-8) + Ks*(1e-7)) )^2;
Qk(6,6) = ( 1/Ja*(Ks*1e-8 + Ks*1e-7 + Beq*0.6e-4) )^2;

Rk = eye(kf_snr_dim);
Rk(1,1) = (70*Ka*2*8*sqrt(mtw_cov)*dt)^2; %desconfiar mais da medida do que do processo...
Rk(2,2) = 64*mtw_cov;
Rk(3,3) = (143*2*pi/2048)^2; % 7% of 2048
Rk(4,4) = (2*pi/4096/N)^2;
Rk(5,5) = 64*mtw_cov;
Rk(6,6) = (2*2*pi/60/N)^2; %devpad 2 rpm

%Erro das IMUs em fi
Rk(1,2) = (8*sqrt(mtw_cov)*dt)^2;
Rk(1,3) = (8*sqrt(mtw_cov)*dt)^2;
%Erro da IMU em pos_exo
Rk(3,2) = (8*sqrt(mtw_cov)*dt)^2;

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

%% accBasedController
zeta = 0.06;
Kp_acc = 0.5436;
Ki_acc = 11.560;

%Ideal
% Kp = 340;
% Ki = 289;
% Kd = 0.034;

%Non Ideal
% Kp = 1.7;
% Ki = 195.5;
% Kd = 0.459;

%% CollocatedAdmittanceControl (CAC)
epos_Ki = 1.190; 
epos_Kp = 11.900;
damping_d = 60;
stiffness_d = 10.4;
k_bar = 1 - stiffness_d/Ks;
stiffness_lower = damping_d*(epos_Ki/epos_Kp - damping_d/(Ja*k_bar) - epos_Kp/Ja)

%Ideal
% damping_d = 60;
% stiffness_d = 10.4;

%Non Ideal
% damping_d = 60;
% stiffness_d = 10.4;

%% Data
% abc_data = importdata('abc_sem_ID/2021-03-27-21-59-14.txt');
t_begin = 32.5;
t_end = abc_data.data(end,1);
% t_end = 32.8300
t_end = t_begin+10

%% Results Analysis
% plotStruct = idealAbc_eval;
% plotStruct = nonidealAbc_eval;
% plotStruct = idealCac_eval;
% plotStruct = nonidealCac_eval;
figure,
subplot(2,1,1)
plot(plotStruct.time, [plotStruct.signals(1,1).values]), grid on
legend('\theta_h','\theta_e')
ylabel('Position (deg)')
subplot(2,1,2)
plot(plotStruct.time, [plotStruct.signals(1,2).values]), grid on
legend('\tau_h','\tau_i')
xlabel('time (s)'), ylabel('Torque (N.m)')

% idealAbcPID_eval % Avg abs pos error = 0.07005 rad
% idealCAC_eval % Avg abs pos error = 0.1818 rad
% nonidealAbcPID_eval % Avg abs pos error = 0.1564 rad
% nonidealCAC_eval % Avg abs pos error = 0.1804 rad

%RMS:
int_torque = idealAbcPID_eval(1).signals(2).values(:,2);
eval_rms(1) = rms(int_torque);
int_torque = idealCAC_eval(1).signals(2).values(:,2);
eval_rms(2) = rms(int_torque);
int_torque = nonidealAbcPID_eval(1).signals(2).values(:,2);
eval_rms(3) = rms(int_torque);
int_torque = nonidealCAC_eval(1).signals(2).values(:,2);
eval_rms(4) = rms(int_torque);

eval_rms

figure,
plot(idealAbcPID_eval(1).time,...
     idealAbcPID_eval(1).signals(2).values), hold on
plot(nonidealAbcPID_eval(1).time, nonidealAbcPID_eval(1).signals(2).values(:,2)), grid on
legend('T_h','T_i (ideal)','T_i (non-ideal)')
xlabel('time (s)'), ylabel('Torque (N.m)')
%% Results Comparison
figure,
subplot(3,2,1)
plot(idealAbcPID_eval(1).time, idealAbcPID_eval(1).signals(1).values(:,2)), hold on
plot(idealAbcPID_eval(1).time, idealAbcPID_eval(1).signals(1).values(:,3),'--'), grid on
title('ASTC (ideal sim)')
subplot(3,2,3)
plot(nonidealAbcPID_eval(1).time, nonidealAbcPID_eval(1).signals(1).values(:,2)), hold on
plot(nonidealAbcPID_eval(1).time, nonidealAbcPID_eval(1).signals(1).values(:,3),'--'), grid on
title('ASTC (sim)')
ylabel('Position (deg)')
subplot(3,2,5)
plot(abc_log_eval.time, abc_log_eval.signals(1).values(:,1)), hold on
plot(abc_log_eval.time, abc_log_eval.signals(1).values(:,2),'--'), grid on
title('ASTC')
xlabel('time (s)')

subplot(3,2,2)
plot(idealCAC_eval(1).time, idealCAC_eval(1).signals(1).values(:,2)), hold on
plot(idealCAC_eval(1).time, idealCAC_eval(1).signals(1).values(:,3),'--'), grid on
legend('\theta_h','\theta_e')
title('DSTC (ideal sim)')
subplot(3,2,4)
plot(nonidealCAC_eval(1).time, nonidealCAC_eval(1).signals(1).values(:,2)), hold on
plot(nonidealCAC_eval(1).time, nonidealCAC_eval(1).signals(1).values(:,3),'--'), grid on
title('DSTC (sim)')
subplot(3,2,6)
plot(cac_log_eval.time, cac_log_eval.signals(1).values(:,1)), hold on
plot(cac_log_eval.time, cac_log_eval.signals(1).values(:,2),'--'), grid on
title('DSTC')
xlabel('time (s)')

figure, % compare int torque:
subplot(2,2,1)
plot(abc_log_eval.time, abc_log_eval.signals(1).values(:,1)), hold on
plot(abc_log_eval.time, abc_log_eval.signals(1).values(:,2),'--'), grid on
title('ASTC')
ylabel('Position (deg)')
subplot(2,2,2)
plot(cac_log_eval.time, cac_log_eval.signals(1).values(:,1)), hold on
plot(cac_log_eval.time, cac_log_eval.signals(1).values(:,2),'--'), grid on
title('DSTC')
legend('\theta_h','\theta_e')
subplot(2,2,3)
plot(abc_log_eval.time, abc_log_eval.signals(2).values), grid on
xlabel('time (s)'), ylabel('Torque (N.m)')
subplot(2,2,4)
plot(cac_log_eval.time, cac_log_eval.signals(2).values), grid on
legend('T_i')
xlabel('time (s)')

%% Kalman Plot
close all

%States
figure,
subplot(2,3,1)
plot(kalmanStates.time, ...
    rad2deg([kalmanStates.signals(2).values(:,2) kalmanStates.signals(2).values(:,1)]) ), grid on
ylabel('deg'), title('Human Position')

subplot(2,3,2)
plot(kalmanStates.time, ...
    rad2deg([kalmanStates.signals(3).values(:,2) kalmanStates.signals(3).values(:,1)]) ), grid on
ylabel('deg'), title('Exo Position')

subplot(2,3,3)
plot(kalmanStates.time, ...
    rad2deg([kalmanStates.signals(4).values(:,2) kalmanStates.signals(4).values(:,1)]) ), grid on
ylabel('deg'), title('Actuator Position')

subplot(2,3,4)
plot(kalmanStates.time, ...
    [kalmanStates.signals(1).values(:,2) kalmanStates.signals(1).values(:,1)] ), grid on
ylabel('N.m'), title('Interaction Torque')

subplot(2,3,5)
plot(kalmanStates.time, ...
    rad2deg([kalmanStates.signals(5).values(:,2) kalmanStates.signals(5).values(:,1)]) ), grid on
ylabel('deg/s'), title('Exo Velocity')
xlabel('time (s)')

subplot(2,3,6)
plot(kalmanStates.time, ...
    rad2deg([kalmanStates.signals(6).values(:,2) kalmanStates.signals(6).values(:,1)]) ), grid on
ylabel('deg/s'), title('Actuator Velocity')

%Accelerations
figure,
subplot(1,2,1)
plot(kalmanAccHum.time,...
    rad2deg([kalmanAccHum.signals(2).values kalmanAccHum.signals(1).values]) ), grid on
xlabel('time (s)'), ylabel('deg/s^2'), title('Human Acceleration')

subplot(1,2,2)
plot(kalmanAccHum.time,...
    rad2deg([kalmanAccExo.signals(2).values kalmanAccExo.signals(1).values]) ), grid on
xlabel('time (s)'), title('Exo Acceleration')

%% Nelder-Mead algorithm
nm_window = 2.0;
alpha = 1.00;
gamma = 1.60;
rho   = 0.50;
sigma = 0.50;
