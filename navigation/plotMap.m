function result = plotMap(fighandle, map, colors)

global mapInfo;

figure(fighandle);

[xlength, ylength] = size(map);

x = [0, mapInfo.resolution*ylength]+mapInfo.origin{1}(1);
y = [mapInfo.resolution*xlength,0]+mapInfo.origin{2}(1);

colormap(colors);
im = imagesc(x, y, map);%grid;
set(gca, 'YDir', 'normal');
result = 1;

%% The following worked for the testlab
% function result = plotMap(fighandle, map, colors)
% 
% global mapInfo;
% 
% figure(fighandle);
% [xlength, ylength] = size(map);
% x = [mapInfo.origin{1}(1), mapInfo.origin{1}(1)+xlength*mapInfo.resolution];
% y = [ylength*mapInfo.resolution+mapInfo.origin{2}(1), mapInfo.origin{2}(1)];
% colormap(colors);
% im = imagesc(x, y, map);
% set(gca, 'YDir', 'normal');
% result = 1;