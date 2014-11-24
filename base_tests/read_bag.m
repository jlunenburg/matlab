%% Load bag
if exist(bagfilename)
    bag = ros.Bag.load(bagfilename);
else
    sprintf('File %s does not exist', bagfilename)
    return
end

%% Read messages
[amcl, amcl_meta] = bag.readAll('/amcl_pose');
[cmd_vel, cmd_vel_meta] = bag.readAll(strcat(robot,'/base/references'));
[meas_vel, meas_vel_meta] = bag.readAll(strcat(robot,'/base/measurements'));
[imu, imu_meta] = bag.readAll('/imu/data');

%% Convert to matrices
% AMCL pose
accessor = @(poseWithCoverianceStamped) poseWithCoverianceStamped.pose.pose.position;
position = ros.msgs2mat(amcl, accessor);
accessor = @(poseWithCoverianceStamped) poseWithCoverianceStamped.pose.pose.orientation;
orientation = ros.msgs2mat(amcl, accessor);
if length(position) == 0
    return
end
amcl_times = cellfun(@(x) x.time.time, amcl_meta);
fprintf('amcl: %i samples\n',length(amcl_times));

% Command velocity
accessor = @(twist) twist.linear;
[cmd_vel_lin] = ros.msgs2mat(cmd_vel, accessor);
accessor = @(twist) twist.angular;
[cmd_vel_ang] = ros.msgs2mat(cmd_vel, accessor);
cmd_vel_times = cellfun(@(x) x.time.time, cmd_vel_meta);
fprintf('cmd_vel: %i samples\n',length(cmd_vel_times));

% Measured velocity
accessor = @(odom) odom.twist.twist.linear;
[meas_vel_lin] = ros.msgs2mat(meas_vel, accessor);
accessor = @(odom) odom.twist.twist.angular;
[meas_vel_ang] = ros.msgs2mat(meas_vel, accessor);
meas_vel_times = cellfun(@(x) x.time.time, meas_vel_meta);
fprintf('odom: %i samples\n',length(meas_vel_times));

% Imu
accessor = @(imu) imu.orientation;
[imu_orientation] = ros.msgs2mat(imu, accessor);
accessor = @(imu) imu.angular_velocity;
[imu_ang_vel] = ros.msgs2mat(imu, accessor);
accessor = @(imu) imu.linear_acceleration;
[imu_lin_acc] = ros.msgs2mat(imu, accessor);
imu_times = cellfun(@(x) x.time.time, imu_meta);
fprintf('imu: %i samples\n',length(imu_times));
imu_rpy = zeros(3, length(imu_times));
% Get RPY
for ii = 1:length(imu_times);
    %imu_rpy(:,ii) = quat2angle(imu_orientation(:,ii)','XYZ')';
    [roll,pitch,yaw] = getRPY(imu_orientation(:,ii));
    imu_rpy(:,ii) = [roll;pitch;yaw];
end
% Unwrap
for ii = 2:length(imu_times);
    for jj = 1:3;
        if (imu_rpy(jj,ii)-imu_rpy(jj,ii-1)) > pi;
            imu_rpy(jj,ii) = imu_rpy(jj,ii) - 2*pi;
        elseif (imu_rpy(jj,ii)-imu_rpy(jj,ii-1)) < -pi;
            imu_rpy(jj,ii) = imu_rpy(jj,ii) + 2*pi;
        end
    end
end
grav_norm = zeros(size(imu_times));
% Compensate linear acceleration for roll and pitch
for ii = 1:length(imu_times);
    % Test magnitude
    grav_norm = norm(imu_lin_acc(:,ii),2);
    
    %x x-direction
    imu_lin_acc(1,ii) = imu_lin_acc(1,ii) + g*sin(imu_rpy(2,ii));
    
    % y-direction
    imu_lin_acc(2,ii) = imu_lin_acc(2,ii) - g*sin(imu_rpy(1,ii));
end

% Integrated acceleration
imu_vel = [];
% Find start and end for polyfit
% istart = 0;
% iend   = 0;
% for ii = 1:length(imu_times);
%     if istart == 0 && (imu_times(ii)-cmd_vel_times(1)) > 1.5
%         istart = ii;
%     elseif iend == 0 && (imu_times(ii) - cmd_vel_times(1)) > 17.5
%         iend = ii;
%     end
% end
for ii = 1:3;
    imu_vel = [imu_vel;cumtrapz(imu_times,imu_lin_acc(ii,:))];
    
    % Correct for static offset
    p = polyfit([0,imu_times(end)-imu_times(1)], [imu_vel(ii,1),imu_vel(ii,end)], 1);
    %p = polyfit(imu_times(istart:iend)-cmd_vel_times(1),imu_vel(ii,istart:iend), 1);
    imu_vel(ii,:) = imu_vel(ii,:) - polyval(p,imu_times-imu_times(1));
    
end

%% If no cmd_vel measurements: make something up!
if (length(cmd_vel_times) == 0)
    fprintf('No cmd vel!!!\n')
    cmd_vel_times = meas_vel_times(1);
    cmd_vel_lin = [0;0;0];
    cmd_vel_ang = [0;0;0];
end