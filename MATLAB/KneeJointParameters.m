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
