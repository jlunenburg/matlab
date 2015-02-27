function addPSD(oh);

% Custom function to update figures, i.e., to make a new figure to include 
% PSD stuff
% 
% Inputs:
%   oh: object handle

%% Get values
t = get(oh,'XData');
y = get(oh,'YData');
color = get(oh,'color');

%% Compute PSD
dt = mean(diff(t));
Fs = 1/dt;

[Pxx,F] = pwelch(y,[],[],[],Fs);

%% Plot results
figure;
plot(F,Pxx);
set(gca,'YScale,','log');

% see http://www.mathworks.com/matlabcentral/answers/21850-calculate-psd-using-fft
% see http://nl.mathworks.com/help/signal/ref/pwelch.html