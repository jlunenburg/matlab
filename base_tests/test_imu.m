% Script to plot base performance
clear all;
close all; 
clc;

%% Parameters
date   = '20141128';
g      = -9.877;     % Norm of raw acceleration vector when in rest??
sampts = 0.01;      % Resampling period
sampme = 'pchip';   % Resampling method

plotsettings;



%% Read bag file
bagfilename = strcat('/home/amigo/ros/data/recorded/rosbags/base_performance/',date,'/imu_test.bag');
if exist(bagfilename)
    bag = ros.Bag.load(bagfilename);
else
    sprintf('File %s does not exist', bagfilename)
    return
end
clear bagfilename
[imu, imu_meta] = bag.readAll('/imu/data');
clear bag

%% Read data
accessor = @(imu) imu.orientation;
[imu_orientation] = ros.msgs2mat(imu, accessor);
accessor = @(imu) imu.angular_velocity;
[imu_ang_vel] = ros.msgs2mat(imu, accessor);
accessor = @(imu) imu.linear_acceleration;
[imu_lin_acc] = ros.msgs2mat(imu, accessor);
imu_times = cellfun(@(x) x.time.time, imu_meta);
fprintf('imu:     %i samples\n',length(imu_times));
clear accessor

%% Compensate gravity and compute angles
imu_lin_acc_comp = zeros(size(imu_lin_acc));
gravity = [0;0;g];
rpy = zeros(3,length(imu_times));
for ii = 1:length(imu_times);
    % Test magnitude
    grav_norm = norm(imu_lin_acc(:,ii),2);
    
    % Using the aerospace toolbox
    quat = [imu_orientation(4,ii);imu_orientation(1,ii);imu_orientation(2,ii);imu_orientation(3,ii)];
    imu_lin_acc_comp(:,ii) = imu_lin_acc(:,ii) - quatrotate(quat', -gravity')';
    
    % Angles
    %[pitch, roll, yaw] = quat2angle(quat', 'YXZ');
    [yaw, pitch, roll] = quat2angle(quat', 'ZYX'); % Put yaw first: reduces crosstalk
    rpy(:,ii) = [roll;pitch;yaw];
end
figure('Name','Gravity Compensation');
for ii = 1:3;
    subplot(3,1,ii);
    plot(imu_times, imu_lin_acc(ii,:),...
         imu_times, imu_lin_acc_comp(ii,:));
    grid;
end;

figure('Name','Gravity norm');
plot(imu_times,grav_norm);

figure('Name','Angles');
for ii = 1:3;
    subplot(3,1,ii);
    plot(imu_times, rpy(ii,:));
    grid;
end

%% Angles
% Get RPY
% imu_rpy = zeros(3, length(imu_times));
% for ii = 1:length(imu_times);
%     %imu_rpy(:,ii) = quat2angle(imu_orientation(:,ii)','XYZ')';
%     [roll,pitch,yaw] = getRPY(imu_orientation(:,ii));
%     imu_rpy(:,ii) = [roll;pitch;yaw];
% end
% % Unwrap
% for ii = 2:length(imu_times);
%     for jj = 1:3;
%         if (imu_rpy(jj,ii)-imu_rpy(jj,ii-1)) > pi;
%             imu_rpy(jj,ii) = imu_rpy(jj,ii) - 2*pi;
%         elseif (imu_rpy(jj,ii)-imu_rpy(jj,ii-1)) < -pi;
%             imu_rpy(jj,ii) = imu_rpy(jj,ii) + 2*pi;
%         end
%     end
% end
%[pitch, roll, yaw] = quat2angle(q, 'YXZ')