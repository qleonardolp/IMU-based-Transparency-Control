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
state_dim  = 6;
sensor_dim = 6;
control_dim = 3;

% State Space matrices

A = [0 0 0 1 0 0; ...
    0 0 0 0 1 0; ...
    0 0 0 0 0 1; ...
    -Ka/Jh Ka/Jh 0 0 0 0
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

%% CollocatedAdmittanceControl (CAC)
epos_Ki = 1.190; 
epos_Kp = 11.900;
damping_d = 10.0;
stiffness_d = 42.0;
k_bar = 1 - stiffness_d/Ks;
stiffness_lower = damping_d*(epos_Ki/epos_Kp - damping_d/(Ja*k_bar) - epos_Kp/Ja)

%% Data
% abc_data = importdata('abc_sem_ID/2021-03-27-21-59-14.txt');
t_begin = 32.5;
t_end = abc_data.data(end,1);
% t_end = 32.8300
t_end = t_begin+10