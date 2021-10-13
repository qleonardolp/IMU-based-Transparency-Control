close all
clear
clc

figure,
title('Velocities')
ax1 = gca;
h1 = animatedline(ax1,'MaximumNumPoints',500,'Color',[0 0.4470 0.7410]);
hold on
ax2 = ax1;
h2 = animatedline(ax2,'MaximumNumPoints',500,'Color',[0.8500 0.3250 0.0980]);
legend('vel hum','vel exo')
xlabel('time (s)'), ylabel('deg/s')
grid on

figure,
title('Accelerations')
h3 = animatedline('MaximumNumPoints',500,'Color',[0 0.4470 0.7410]);
hold on
h4 = animatedline('MaximumNumPoints',500,'Color',[0.8500 0.3250 0.0980]);
legend('acc hum','acc exo')
xlabel('time (s)'), ylabel('deg/ss')
grid on

figure,
title('Encoders')
h5 = animatedline('MaximumNumPoints',500,'Color',[0 0.4470 0.7410]);
hold on
h6 = animatedline('MaximumNumPoints',500,'Color',[0.8500 0.3250 0.0980]);
legend('pos motor','pos exo')
xlabel('time (s)'), ylabel('deg')
grid on

figure,
title('Motor')
h7 = animatedline('MaximumNumPoints',500,'Color',[0 0.4470 0.7410]);
hold on
h8 = animatedline('MaximumNumPoints',500,'Color',[0.8500 0.3250 0.0980]);
legend('vel motor','acc motor')
xlabel('time (s)'), ylabel('deg/s, deg/ss')
grid on

PORT = 2324;
%echotcpip('on', PORT)           % Tirar na execucao com o Exo
tcp = tcpip('127.0.0.1',PORT);
tcp.TransferDelay = 'off';
tcp.InputBufferSize = 64;
fopen(tcp);
tcp.Status
sendString = "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n";
receiveString = "%f, %f, %f, %f, %f, %f, %f, %f, %f\n";
sample_period = 0.05; 
%%
tic;
last_toc = toc;
flushinput(tcp);
while( true )
    %fprintf(tcp, sendString, rand(1)*ones(9,1)); % Tirar na execução
    if (toc - last_toc) > sample_period
        data = fscanf(tcp, receiveString);
        data(2:end) = rad2deg(data(2:end));
%         data
%         addpoints(h1, toc, sin(2*pi*toc));
%         addpoints(h2, toc, 1.2*sin(2*pi*toc));
%         addpoints(h3, toc, sin(2*pi*toc)+1);
%         addpoints(h4, toc, 1.2*sin(2*pi*toc)+1);
        addpoints(h1, toc, data(2));
        addpoints(h2, toc, data(3));
%         addpoints(h3, toc, data(4));
%         addpoints(h4, toc, data(5));
        addpoints(h5, toc, data(6));
        addpoints(h6, toc, data(7));
%         addpoints(h7, toc, data(8));
%         addpoints(h8, toc, data(9));
        drawnow
        last_toc = toc;
    end
    %str = input('','s');
end

%%
echotcpip('off')
