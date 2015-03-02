function [Pxx,F] = addPSD(ah);

% Custom function to update figures, i.e., to make a new figure to include 
% PSD stuff
% 
% Inputs:
%   ah: object handle to the data to process
%   nfh: new figure handle: handle to the figure where to plot the new data
%   no: number. Time domain will be plotted in 2*no-1, frequency domain
%   data in 2*no

figure;

%% Loop through values
ch = get(ah,'Children');
for ii = 1:length(ch);
    oh = ch(ii);        % Handle to object, i.e., the line containing the data
    
    %% Get data
    t = get(oh,'XData');
    y = get(oh,'YData');
    color = get(oh,'color');

    %% Compute PSD
    dt = mean(diff(t));
    Fs = 1/dt;

    % Power spectral density
    [Pxx,F] = pwelch(y,[],[],1024,Fs);

    % Cumulative power spectral density
    CPxx = 1/length(Pxx)*cumsum(mean(diff(F))*length(Pxx)^2 * Pxx).^.5;

    % Sanity check:
    sprintf('RMS time signal = %f, final value CPxx = %f', rms(y), CPxx(end))


    %set(gca,'YScale','log');

    % see http://www.mathworks.com/matlabcentral/answers/21850-calculate-psd-using-fft
    % see http://nl.mathworks.com/help/signal/ref/pwelch.html

    %% Alternative 
%     % (see www.mathworks.com/matlabcentral/answers43548-how-to-create-power-spectral-density-from-fft-fourier-transform
%     L = length(y);
%     ydft = fft(y);
%     
%     % Power spectral density
%     Pxx = 1/(L*Fs)*abs(ydft(1:length(y)/2+1)).^2;
%     F = 0:Fs/L:Fs/2;
%     
%     % Cumulative power spectral density
%     CPxx = 1/length(Pxx)*cumsum(mean(diff(F))*length(Pxx)^2 * Pxx).^.5;
% 
%     % Sanity check:
%     sprintf('RMS time signal = %f, final value CPxx = %f', rms(y), CPxx(end))

    %% Plot results
    
    subplot(3,1,1); hold on;
    plot(t,y,'color',color,'LineWidth',1.5);
    subplot(3,1,2); hold on;
    plot(F,Pxx,'color',color,'LineWidth',1.5);
    set(gca,'XScale','log','YScale','log');
    subplot(3,1,3); hold on;
    plot(F,CPxx,'color',color,'LineWidth',1.5);
    set(gca,'XScale','log','YScale','log');
end

%% Obsolete stuff
% Rf = mean(diff(F));
% fac = Rf*length(Pxx)^2
% clear pxx
% test = zeros(size(Pxx));
% for ii = 1:size(Pxx);test(ii) = fac*Pxx(ii);end;
% sum(test)
% 1/length(Pxx)*sqrt(sum(test))
% rms(y)