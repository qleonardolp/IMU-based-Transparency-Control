%% State Space formulation for Transparency Control with Rotary SEA, without Human states
%   Leonardo Felipe L. S. dos Santos, 2020
%   EESC@USP

% Constants
N   = 150;           % Gear Ratio
KI  = 0.0603;        % Nm/A
Ks  = 104;           % Nm/rad
Ka  = Ks/50          % Nm/rad (???)
Je  = 0.47*5;        % Kg.m^2 (???)
We  = 3.5*9.80665;   % N      (???)
Le  = 0.626;         % m      (???)
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
% Measurement noite Covariance Matrix
Rk = eye(sensor_dim);
Rk(1,1) = (2*pi/2048)^2;
Rk(2,2) = (0.002340)^2; % Mtw Noise x sqrt(Bandwidth)
Rk(3,3) = (2*pi/4096)^2;
Rk(4,4) = (3*pi/30)^2;

% Process Covariance Matrix, (pegar do codigo)
Qk = eye(state_dim);
Qk(1,1) = (2*pi/2048 + 0.001*0.002340)^2;
Qk(2,2) = (0.914)^2;
Qk(3,3) = (2*pi/4096 + 0.001*7*pi/30)^2;
Qk(4,4) = (0.825)^2;