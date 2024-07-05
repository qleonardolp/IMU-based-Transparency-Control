function stepb(str, t_end, varargin)
% Step plot customization
% Allows axis and line width definition 
% e.g. stepb(figname,5,S*G1,'b', S*G2,'--r')
    figure('Name', str, 'Color',[1 1 1])
    for k=1:floor(length(varargin)/2)
        [Y,T] = step(varargin{2*k-1}, t_end);
        plot(T,squeeze(Y), varargin{2*k}, 'LineWidth', 1.5)
        ax = gca; ax.FontSize = 12; ax.LineWidth = 0.7; ax.GridAlpha = 0.6;
        xlabel('time (s)')
        ylabel('Amplitude')
        % axis tight
        hold on
        grid on
    end
end

