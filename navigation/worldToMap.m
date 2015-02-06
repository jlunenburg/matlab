function [mx, my] = worldToMap(wx, wy, varargin)

global mapInfo;

if nargin > 2;
    resolution = varargin{1};
else
    resolution = mapInfo.resolution;
end


%mx = floor((-wx - mapInfo.origin{2}) / mapInfo.resolution);
%my = floor((-wy - mapInfo.origin{1}) / mapInfo.resolution);

mx = floor((wy - mapInfo.origin{2}) / resolution);
if mx < 0
    sprintf('mx = %i, that cannot be good', mx)
end
my = floor((wx - mapInfo.origin{1}) / resolution);
if my < 0
    sprintf('mx = %i, that cannot be good', mx)
end
% Works, but origin off...
%mx = floor((wx - mapInfo.origin{2}) / mapInfo.resolution);
%my = floor((wy - mapInfo.origin{1}) / mapInfo.resolution);
%sprintf('x: %f -> %f\ny: %f -> %f', wx, mx, wy, my)