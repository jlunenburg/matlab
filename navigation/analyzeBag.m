function vel_map = analyze_bag(file, fig, vel_map);

%% Load bag
if exist(file)
    bag = ros.Bag.load(file);
else
    sprintf('File %s does not exist', file)
    return
end

%%%%%
%bag = ros.Bag.load('/home/jlunenburg/ros/data/recorded/rosbags/nav_data/20140616/20140616_132412_300276.bag');
%bag.info()
%fig = figure;
%%%%%
%% Read messages
[cmd_vel, cmd_vel_meta] = bag.readAll('/amigo/base/references');
[meas_vel, meas_vel_meta] = bag.readAll('/amigo/base/measurements');
%clear bag
    
%% Convert to matrices
% AMCL pose
% accessor = @(poseWithCoverianceStamped) poseWithCoverianceStamped.pose.pose.position;
% position = ros.msgs2mat(amcl, accessor);
% if length(position) == 0
%     return
% end
% amcl_times = cellfun(@(x) x.time.time, amcl_meta);
%%

% Command velocity
accessor = @(twist) twist.linear;
[cmd_vel_lin] = ros.msgs2mat(cmd_vel, accessor);
cmd_vel_times = cellfun(@(x) x.time.time, cmd_vel_meta);
accessor = @(twist) twist.angular;
[cmd_vel_ang] = ros.msgs2mat(cmd_vel, accessor);

% Measured velocity
accessor = @(odom) odom.twist.twist.linear;
[meas_vel_lin] = ros.msgs2mat(meas_vel, accessor);
meas_vel_times = cellfun(@(x) x.time.time, meas_vel_meta);
accessor = @(odom) odom.twist.twist.angular;
[meas_vel_ang] = ros.msgs2mat(meas_vel, accessor);

% tf
tf = ros.TFTree(bag);
position = zeros(2, length(cmd_vel_times));
n = 0;
while n >= 0;
    try
        robot_pose = tf.lookup('/map','/amigo/base_link',cmd_vel_times(n+1:end-n));
        if n > 5;
            fprintf('\nExtrapolation %i samples\n',n)
        end
        n = -1;
    catch
        n = n+1;
    end
end      

for ii = 1:n;
    position(:,ii) = robot_pose(1).translation(1:2);
end
for ii = 1:length(robot_pose);
    position(:, ii+1) = robot_pose(ii).translation(1:2);
end
for ii = 1:n;
    position(:, end-n+1) = robot_pose(end).translation(1:2);
end

%% Plot velocity over time
%vlin = (vxvyvz(1,:).^2 + vxvyvz(2,:).^2).^.5;
%figure('Name', 'Linear velocity');
%plot(times-times(1), vlin, 'LineWidth', 1);

% figure('Name','Velocity');
% subplot(3,1,1);
% plot(cmd_vel_times - cmd_vel_times(1), cmd_vel_lin(1,:),...
%     meas_vel_times - cmd_vel_times(1), meas_vel_lin(1,:));
% subplot(3,1,2);
% plot(cmd_vel_times - cmd_vel_times(1), cmd_vel_lin(2,:),...
%     meas_vel_times - cmd_vel_times(1), meas_vel_lin(2,:));
% subplot(3,1,3);
% plot(cmd_vel_times - cmd_vel_times(1), cmd_vel_ang(3,:),...
%     meas_vel_times - cmd_vel_times(1), meas_vel_ang(3,:));

%% Plot path
ax = findobj(fig, 'type', 'axes');
if length(ax) > 0
    axes(ax); hold on;
end
plot(position(1,:), position(2,:), 'k', 'LineWidth', 1);

%% Add data to velocity map
global velmapres;
for ii = 1:length(cmd_vel_times)
    [x_map, y_map] = worldToMap(position(1, ii), position(2, ii), velmapres);
    velocity = sqrt(cmd_vel_lin(1,ii) * cmd_vel_lin(1,ii) + cmd_vel_lin(2,ii) + cmd_vel_lin(2,ii));
    vel_map(x_map, y_map).count  = vel_map(x_map, y_map).count + 1;
    vel_map(x_map, y_map).cumvel = vel_map(x_map, y_map).cumvel + velocity;
end
% for ii = 1:length(meas_vel_times);
%     if meas_vel_times(ii) > amcl_times(amcl_index) && amcl_index < length(amcl_times)
%         amcl_index = amcl_index + 1;
%     end
%     [x_map, y_map] = worldToMap(position(1, amcl_index), position(2, amcl_index), velmapres);
%     velocity = sqrt(meas_vel_lin(1,ii) * meas_vel_lin(1,ii) + meas_vel_lin(2,ii) * meas_vel_lin(2,ii));
%     vel_map(x_map, y_map).count  = vel_map(x_map, y_map).count + 1;
%     vel_map(x_map, y_map).cumvel = vel_map(x_map, y_map).cumvel + velocity;
% end

clear amcl amcl_meta cmd_vel cmd_vel_meta meas_vel meas_vel_data