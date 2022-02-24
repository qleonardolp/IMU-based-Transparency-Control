function bodeb(str ,range, varargin)
% Bode diagram customization (Bode brabo)
% Allows axis and line width definition
% Inputs are frequency range and sys with line string format: 
% e.g. bodeb(figname,{0.01,1000},G1,'b', G2,'--r')
    figure('Name', str, 'Color',[1 1 1])
    for k=1:floor(length(varargin)/2)
        [mag,phase,wout] = bode(varargin{2*k-1}, range);
        ax=subplot(2,1,1);
        semilogx(wout, 20*log10(squeeze(mag)),varargin{2*k},'LineWidth',1.5)
        ax.FontSize = 12; ax.LineWidth = 0.7; ax.GridAlpha = 0.6;
        ax.XAxis.Visible = 'off';
        ylabel('Magnitude (dB)')
        hold on
        grid on
        ax=subplot(2,1,2);
        semilogx(wout, squeeze(phase),varargin{2*k},'LineWidth',1.5)
        ax.FontSize = 12; ax.LineWidth = 0.7; ax.GridAlpha = 0.6;
        ylabel('Phase (deg)')
        xlabel('Frequency (rad/s)')
        hold on
        grid on
    end
end

