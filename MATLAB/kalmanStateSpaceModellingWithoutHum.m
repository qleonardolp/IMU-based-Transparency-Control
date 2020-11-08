%% State Space formulation for Transparency Control with Rotary SEA, without Human states
%   Leonardo Felipe L. S. dos Santos, 2020
%   EESC@USP

% Constants
N   = 150;           % Gear Ratio
KI  = 0.0603;        % Nm/A
Ks  = 104;           % N/rad
Ka  = Ks/20          % N/rad  (???)
% Jh  = 0.05;          % Kg.m^2 (???)
Je  = 0.47*2;        % Kg.m^2 (???)
We  = 2.00*9.80665;  % N      (???)
Le  = 0.40;          % m      (???)
Beq = 60;            % N.m.s/rad
Ja  = 0.47;          % Kg.m^2

% State vector is  [x_e \dot{x_e} x_a \dot{x_a}]
% Sensor vector is [x_e \dot{x_e} x_a \dot{x_a}]
state_dim  = 4;
sensor_dim = 4;
control_dim = 1;

% State Space matrices

A = [0 1 0 0; ...
     -(Ks + Ka - We*Le)/Je 0 Ks/Je 0; ...
     0 0 0 1; ...
     Ks/Ja 0 -Ks/Ja -Beq/Ja];

B = [zeros(3,control_dim); N*KI/Ja];

C = [eye(sensor_dim)]; C(3,3) = N; C(4,4) = N;
D = zeros(sensor_dim, control_dim);

%% State Space definition, 
% considering the different sensors 
% sample rate as output delay:
imu_delay = 8; % [ms]
Sys = ss(A,B,C,D,'TimeUnit','milliseconds','OutputDelay',[0 imu_delay 0 0]');
% Discretization
Ts = 1; % [ms]
sea_bw_freq = 9.6/1000; % [kHz]
disOpt = c2dOptions('Method','tustin','PrewarpFrequency',2*pi*sea_bw_freq);
discreteSys = c2d(Sys,Ts,disOpt)
% discreteSys = c2d(Sys,Ts)

%% Kalman
% Process Covariance Matrix, (pegar do codigo)
Q = []; 

% Measurement noite Covariance Matrix
R = [];
