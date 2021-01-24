%% State Space formulation for Transparency Control with Rotary SEA, with only velHum as a control input
%   Leonardo Felipe L. S. dos Santos, 2020
%   EESC@USP

% Constants
N   = 150;           % Gear Ratio
KI  = 0.0603;        % Nm/A
Ks  = 104;           % Nm/rad
Ka  = Ks/20          % Nm/rad (???)
Jh  = 0.05;          % Kg.m^2 (???)
Je  = 0.47*2;        % Kg.m^2 (???)
We  = 2.00*9.80665;  % N      (???)
Le  = 0.40;          % m      (???)
Beq = 60;            % N.m.s/rad
Ja  = 0.47;          % Kg.m^2

% State vector is  [x_h x_e \dot{x_e} x_a \dot{x_a}]
% Sensor vector is [\dot{x_h} x_e \dot{x_e} x_a \dot{x_a}]
state_dim  = 5;
sensor_dim = 5;
control_dim = 2;

% State Space matrices

A = [0 0 0 0 0; ...
    0 0 1 0 0; ...
    Ka/Je -(Ks + Ka - We*Le)/Je 0 Ks/Je 0; ...
    0 0 0 0 1; ...
    0 Ks/Ja 0 -Ks/Ja -Beq/Ja];

B = [1 0; zeros(3,control_dim); 0 N*KI/Ja];

C = [eye(sensor_dim)]; C(4,4) = N; C(5,5) = N; C(1,1) = 0;
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

%% Control Design
epos_Ki = 1.190; epos_Kp = 11.900;

damping_d = 0.900;
stiffness_d = 3.874;
stiffness_lower = 3.8422;

eposVelocityController = pid(epos_Kp,epos_Ki,0,0,0.001);

s = tf('s')

CAC = (1 - stiffness_d/Ks)*s / (stiffness_d + s*damping_d);
CAC = c2d(CAC,0.001);
%%
stiffness_lower = damping_d*(epos_Ki/epos_Kp - damping_d/(Ja*(1 - stiffness_d/Ks)) - epos_Kp/Ja)