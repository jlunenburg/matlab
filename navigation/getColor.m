function color = getColor(input);

% Returns color between blue and red
% input should be between 0 and 1

color = [0.0, 0.0, 0.0];
seccount = 5; % Number of sections
if input >= 0.0 && input <= 0.2;
    color(1) = 1.0 - seccount*(input);
    color(3) = 1.0;
elseif input > 0.2 && input <= 0.4;
    color(2) = seccount*(input-0.2);
    color(3) = 1.0;
elseif input > 0.4 && input <= 0.6;
    color(2) = 1.0;
    color(3) = 1.0 - seccount*(input-0.4);
elseif input > 0.6 && input <= 0.8;
    color(1) = seccount*(input-0.6);
    color(2) = 1.0;
elseif input > 0.8 && input <= 1.0;
    color(1) = 1.0;
    color(2) = 1.0 - seccount*(input-0.8);
else
    fprintf('Please specify input between 0.0 <= input <= 1.0\n')
end
 
%% Limit output
 color = min([1.0, 1.0, 1.0],color);
 color = max([0.0, 0.0, 0.0],color);

