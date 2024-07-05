function bodemagb(str ,range, varargin)
% Bode magnitude only diagram customization (Bode brabo)
% Allows axis and line width definition
% Inputs are frequency range and sys with line string format: 
% e.g. bodeb(figname,{0.01,1000},G1,'b', G2,'--r')
       % Forcing x-ticks 10^2 delta:
    omg_ticks = floor(log10(range{1})):2:ceil(log10(range{end}));
    omg_ticks = 10.^(omg_ticks);
    figure('Name', str, 'Color',[1 1 1])
    for k=1:floor(length(varargin)/2)
        [mag,~,wout] = bode(varargin{2*k-1}, range);
        semilogx(wout, 20*log10(squeeze(mag)),varargin{2*k},'LineWidth',1.5)
        ax = gca; ax.FontSize = 12; ax.LineWidth = 0.7; ax.GridAlpha = 0.6;
        xticks(omg_ticks)
        xlabel('Frequency (rad/s)')
        ylabel('Magnitude (dB)')
        axis tight
        hold on
        grid on
    end
end

