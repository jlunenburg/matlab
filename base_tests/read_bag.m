%% Load bag
if exist(bagfilename)
    bag = ros.Bag.load(bagfilename);
else
    sprintf('File %s does not exist', bagfilename)
    return
end
clear bagfilename

%% Read messages
[amcl, amcl_meta] = bag.readAll('/amcl_pose');
[cmd_vel, cmd_vel_meta] = bag.readAll(strcat(robot,'/base/references'));
[meas_vel, meas_vel_meta] = bag.readAll(strcat(robot,'/base/measurements'));
[imu, imu_meta] = bag.readAll('/imu/data');
clear bag

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
fprintf('amcl:    %i samples\n',length(amcl_times));

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
fprintf('odom:    %i samples\n',length(meas_vel_times));

% Imu
accessor = @(imu) imu.orientation;
[imu_orientation] = ros.msgs2mat(imu, accessor);
accessor = @(imu) imu.angular_velocity;
[imu_ang_vel] = ros.msgs2mat(imu, accessor);
accessor = @(imu) imu.linear_acceleration;
[imu_lin_acc] = ros.msgs2mat(imu, accessor);
imu_times = cellfun(@(x) x.time.time, imu_meta);
fprintf('imu:     %i samples\n',length(imu_times));

clear accessor

%% If no cmd_vel measurements: make something up!
if (length(cmd_vel_times) == 0)
    fprintf('\nNo cmd vel!!!\n\n')
    cmd_vel_times = meas_vel_times;
    cmd_vel_lin = meas_vel_lin;
    cmd_vel_ang = meas_vel_ang;
end

%% Resample data
zerostamp = cmd_vel_times(1);

% cmd vel
cmd_vel_times_new = 0:sampts:(cmd_vel_times(end) - zerostamp);
cmd_vel_lin_new = [];
cmd_vel_ang_new = [];
for ii = 1:3;
    cmd_vel_lin_new = [cmd_vel_lin_new;interp1(cmd_vel_times-zerostamp, cmd_vel_lin(ii,:), cmd_vel_times_new, sampme)];
    cmd_vel_ang_new = [cmd_vel_ang_new;interp1(cmd_vel_times-zerostamp, cmd_vel_ang(ii,:), cmd_vel_times_new, sampme)];
end


% Measured velocity (odom)
meas_vel_times_new = 0:sampts:(meas_vel_times(end) - zerostamp);
meas_vel_lin_new = [];
meas_vel_ang_new = [];
for ii = 1:3;
    meas_vel_lin_new = [meas_vel_lin_new;interp1(meas_vel_times-zerostamp, meas_vel_lin(ii,:), meas_vel_times_new, sampme)];
    meas_vel_ang_new = [meas_vel_ang_new;interp1(meas_vel_times-zerostamp, meas_vel_ang(ii,:), meas_vel_times_new, sampme)];
end

% AMCL
amcl_times_new = 0:sampts:(amcl_times(end) - zerostamp);
position_new = [];
orientation_new = [];
for ii = 1:3;
    position_new = [position_new;interp1(amcl_times-zerostamp, position(ii,:), amcl_times_new, sampme)];
    orientation_new = [orientation_new;interp1(amcl_times-zerostamp, orientation(ii,:), amcl_times_new, sampme)];
end
orientation_new = [orientation_new;interp1(amcl_times-zerostamp, orientation(4,:), amcl_times_new, sampme)];

% IMU
imu_times_new = 0:sampts:(imu_times(end) - zerostamp);
imu_lin_acc_new = [];
imu_orientation_new = [];
imu_ang_vel_new = [];
for ii = 1:3;
    imu_lin_acc_new = [imu_lin_acc_new;interp1(imu_times-zerostamp, imu_lin_acc(ii,:), imu_times_new, sampme)];
    imu_orientation_new = [imu_orientation_new;interp1(imu_times-zerostamp, imu_orientation(ii,:), imu_times_new, sampme)];
    imu_ang_vel_new = [imu_ang_vel_new;interp1(imu_times-zerostamp, imu_ang_vel(ii,:), imu_times_new, sampme)];
end
imu_orientation_new = [imu_orientation_new;interp1(imu_times-zerostamp, imu_orientation(4,:), imu_times_new, sampme)];

clear ii

% Switch stuff
cmd_vel_times = cmd_vel_times_new;
cmd_vel_lin   = cmd_vel_lin_new;
cmd_vel_ang   = cmd_vel_ang_new;
clear cmd_vel_times_new cmd_vel_lin_new cmd_vel_ang_new

meas_vel_times = meas_vel_times_new;
meas_vel_lin   = meas_vel_lin_new;
meas_vel_ang   = meas_vel_ang_new;
clear meas_vel_times_new meas_vel_lin_new meas_vel_ang_new

amcl_times = amcl_times_new;
position   = position_new;
orientation= orientation_new;
clear amcl_times_new position_new orientation_new

imu_times       = imu_times_new;
imu_lin_acc     = imu_lin_acc_new;
imu_orientation = imu_orientation_new;
imu_ang_vel     = imu_ang_vel_new;
clear imu_times_new position_new imu_orientation_new imu_ang_vel_new

%% Further postprocessing
% Get velocities from AMCL
amcl_vel = [];
for ii = 2:length(amcl_times)
    dt = amcl_times(ii) - amcl_times(ii-1);
    if dt > 0.01
        vx_map = (position(1,ii) - position(1,ii-1))/dt;
        vy_map = (position(2,ii) - position(2,ii-1))/dt;
        quat  = orientation(:,ii);
        yawc  = atan2( 2*(quat(1)*quat(2) + quat(3)*quat(4)), 1-2*(quat(2)*quat(2) + quat(3)*quat(3)) );
        quat  = orientation(:,ii-1);
        yawp  = atan2( 2*(quat(1)*quat(2) + quat(3)*quat(4)), 1-2*(quat(2)*quat(2) + quat(3)*quat(3)) );
        % Correct for changes -pi/pi
        if (yawc - yawp) > pi
            yawp = yawp + 2*pi;
        elseif (yawc - yawp) < -pi
            yawp = yawp - 2*pi;
        end
        v_yaw_map = (yawc - yawp)/dt;
        if abs(v_yaw_map) > 10
            fprintf('Previous: %3f, current: %3f, dt = %3f, vel: %3f\n',yawp, yawc, dt, v_yaw_map)
        end
        yawc = (yawc+yawp)/2;
        amcl_vel = [amcl_vel, [vx_map*cos(yawc) + vy_map*sin(yawc);
            -vx_map*sin(yawc) + vy_map*cos(yawc);
            v_yaw_map]];
    else
        if numel(amcl_vel) == 0;
            amcl_vel = [0;0;0];
        else
            amcl_vel = [amcl_vel, amcl_vel(:,end)];
        end
    end
end

% Get RPY
imu_rpy = zeros(3, length(imu_times));
for ii = 1:length(imu_times);
    %imu_rpy(:,ii) = quat2angle(imu_orientation(:,ii)','XYZ')';
    %[roll,pitch,yaw] = getRPY(imu_orientation(:,ii));
    %imu_rpy(:,ii) = [roll;pitch;yaw];
    
    quat = [imu_orientation(4,ii);imu_orientation(1,ii);imu_orientation(2,ii);imu_orientation(3,ii)];
    [yaw, pitch, roll] = quat2angle(quat', 'ZYX'); % Put yaw first: reduces crosstalk
    imu_rpy(:,ii) = [roll;pitch;yaw];
end
% Unwrap
% for ii = 2:length(imu_times);
%     for jj = 1:3;
%         if (imu_rpy(jj,ii)-imu_rpy(jj,ii-1)) > pi;
%             imu_rpy(jj,ii) = imu_rpy(jj,ii) - 2*pi;
%         elseif (imu_rpy(jj,ii)-imu_rpy(jj,ii-1)) < -pi;
%             imu_rpy(jj,ii) = imu_rpy(jj,ii) + 2*pi;
%         end
%     end
% end

grav_norm = zeros(size(imu_times));
% Compensate linear acceleration for roll and pitch
imu_lin_acc_comp = zeros(size(imu_lin_acc));
gravity = [0;0;g];
for ii = 1:length(imu_times);
    % Test magnitude
    grav_norm = norm(imu_lin_acc(:,ii),2);
    
    quat = [imu_orientation(4,ii);imu_orientation(1,ii);imu_orientation(2,ii);imu_orientation(3,ii)];
    imu_lin_acc_comp(:,ii) = imu_lin_acc(:,ii) - quatrotate(quat', -gravity')';
end
figure('Name','Gravity Compensation');for ii = 1:3;subplot(3,1,ii);plot(imu_times,imu_lin_acc(ii,:),imu_times,imu_lin_acc_comp(ii,:));grid;end;
fprintf('Compensating for gravity\n')
imu_lin_acc = imu_lin_acc_comp;

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
%     fprintf('Additional drift compensation\n')
%     p = polyfit([0,imu_times(end)-imu_times(1)], [imu_vel(ii,1),imu_vel(ii,end)], 1);
%     %p = polyfit(imu_times(istart:iend)-cmd_vel_times(1),imu_vel(ii,istart:iend), 1);
%     imu_vel(ii,:) = imu_vel(ii,:) - polyval(p,imu_times-imu_times(1));
    
end

clear ii

%% Low pass filtering
for ii = 1:3;
    fprintf('Lowpass amcl_vel\n')
    amcl_vel(ii,:) = lowpass(amcl_vel(ii,:), 1/sampts, lpf);
    %imu_vel(ii,:)  = highpass(imu_vel(ii,:), 1/sampts, hpf);
end

clear ii

