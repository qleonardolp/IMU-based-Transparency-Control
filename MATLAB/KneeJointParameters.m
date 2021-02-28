%% Estimations based on:
%  _A Model of the Lower Limb for Analysis of Human Movement_ (2010)
%  

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

isometricKneeStiffness

% What about using a variable desired stiffness according to the flexion-extension angle?
% Check below...

%% _Constant and Variable Stiffness and Damping of the Leg Joints in Human Hopping_ (2003)
% Herein was suggested a first order variable stiffness with the joint
% angle and a constant damping. For 1.53 Hz hopping the knee means shows:
% KneeStiffness(ang) = 110.0 + 169.3(ang - ang_0)
% KneeDampingRatio = 0.06 % (6%)


%% _A Validated Three-Dimensional Computational Model of a Human Knee Joint_ (1999)
%