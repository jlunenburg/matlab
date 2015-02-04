function [xlength, ylength] = getMap(fighandle, env)
%fignum = 6;
%env = 'robotics_testlab_B';
%% Load map and metadata
mappath = strcat(getenv('HOME'),'/ros/groovy/catkin_ws/src/tue/trunk/amigo_maps/maps/');
mappath = strcat(mappath, env);
mappath = strcat(mappath, '/loc/');

yamlfile = strcat(mappath, 'yaml');
yamlstruct = ReadYaml(yamlfile);

mapfile = strcat(mappath, 'pgm');
map     = imread(mapfile);

%% Plot figure
% figure(fighandle);
[xlength, ylength] = size(map);
% x = [yamlstruct.origin{1}(1), yamlstruct.origin{1}(1)+xlength*yamlstruct.resolution];
% y = [ylength*yamlstruct.resolution+yamlstruct.origin{2}(1), yamlstruct.origin{2}(1)];
% colormap(gray);
% im = imagesc(x, y, map);
% set(gca, 'YDir', 'normal');
plotMap(fighandle, map, gray);
result = 1;