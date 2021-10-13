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
% Kp = 410;
% Ki = 330;
% Kd = 0.15;

%Non Ideal
% Kp = 346.80;
% Ki = 18;
% Kd = 4.92;

%Abc with Kf
% Kp = 346.80;
% Ki = 0;
% Kd = 4.92;

%% CollocatedAdmittanceControl (CAC)
epos_Ki = 1.190; 
epos_Kp = 11.900;
damping_d = 0.001;
stiffness_d = 0.7;
k_bar = 1 - stiffness_d/Ks;
stiffness_lower = damping_d*(epos_Ki/epos_Kp - damping_d/(Ja*k_bar) - epos_Kp/Ja);

[stiffness_d damping_d]
%Ideal
% damping_d = 0.03;
% stiffness_d = .104;

%Non Ideal
% damping_d = 0.001;
% stiffness_d = 0.7;

% CAC without Kf
% damping_d = 13;
% stiffness_d = 70;

% CAC with Kf
% damping_d = 12;
% stiffness_d = 50;