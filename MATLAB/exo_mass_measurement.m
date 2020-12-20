%% Collected data:

theta_l_deg = [-24.9609 -35.6835 -42.5390 -51.5039 -60.4687 -64.5117 -70.1367 -79.1015 -85.6054 -90.8789 -97.0312];
tsea = [-9.6518 -12.8531 -14.8153 -16.2799 -17.3136 -17.8550 -18.3496 -19.1728 -19.4833 -19.8428 -19.7535];
weight = [6.7935 9.3904 10.8841 12.5995 14.0071 14.5317 15.1407 15.8081 16.0511 16.0966 15.9774];

%Constants
LOWERLEGMASS = 3.6480;
GRAVITY = 9.8066;
L_CG    = 0.4320;

-LOWERLEGMASS*GRAVITY*L_CG

%% Fit Torque with sin(theta_l)...
myfit_type = fittype('-A*sin(x)','dependent',{'y'},'independent',{'x'},...
                     'coefficients',{'A'})

x = deg2rad(theta_l_deg);   y = tsea;

torque_w_fit = fit(x',y',myfit_type)

%Compare
figure,
plot(torque_w_fit, x, y,'--'), hold on
plot(x, -weight), grid on
title('Exo Leg Weight'), legend('Data','Fitted','Old Weight')
%Calcualte new mass
mass_fit = - torque_w_fit.A /(GRAVITY*L_CG);

fprintf('\nNew mass: %.5f Kg\n',mass_fit);