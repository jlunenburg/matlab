color = [0.8,0.8,0.8];

% Obtain the tick mark locations
xtick = get(gca,'XTick'); 
ytick = get(gca,'YTick'); 
% Obtain the limits of the y axis
xlim = get(gca,'Xlim');
ylim = get(gca,'Ylim');
% Create line data in the first direction
X = repmat(xtick(2:end-1),2,1);
Y = repmat(ylim',1,size(xtick(2:end-1),2));

% Plot line data in the first direction
hold on;
plot(X,Y,'color',color)

% Create line data in the second direction
Y = repmat(ytick(2:end-1),2,1);
X = repmat(xlim',1,size(ytick(2:end-1),2));
% Plot line data in the second direction
hold on;
plot(X,Y,'color',color)