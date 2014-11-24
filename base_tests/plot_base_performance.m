% Script to plot base performance
clear all;
close all; 
clc;
 
%% Parameters
robot = '/amigo'; % Put a slash before the robot name
bagfilename = '/home/amigo/ros/data/recorded/rosbags/base_performance/20141124/amigo_corridor_03.bag';
plotsettings;
plotts = 0.05; % Time where stuff is interpolated
lpf    = 6;     % Cut-off frequency of the lowpass filter
g      = 9.877;  % Gravity acceleration

%% Read bag file
read_bag

%% Filter amcl 
% if length(amcl_times) > 1;
%     pos_fil = []; or_fil = []; % Temporary variables
%     time = amcl_times(1):plotts:amcl_times(end);
%     for ii = 1:3;
%         ipv = interp1(amcl_times, position(ii,:),time); % Interpolated vector
%         [fv, N, B, A] = lowpass(ipv, 1/plotts, lpf);   % Filtered vector
%         pos_fil = [pos_fil; fv];
%     end
%     for ii = 1:4;
%         ipv = interp1(amcl_times, orientation(ii,:),time);  % Interpolated vector
%         [fv, N, B, A] = lowpass(ipv, 1/plotts, lpf);       % Filtered vector
%         or_fil = [or_fil; fv];
%     end
%     amcl_times  = time;
%     position    = pos_fil;
%     orientation = or_fil;
% end

clear time pos_fil or_fil ipv fv N B A

%% Get velocities from AMCL
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

%% Compute errors
ipv = interp1(meas_vel_times, meas_vel_lin(1,:), cmd_vel_times); % Interpolated vector
emx = cmd_vel_lin(1,:) - ipv; % Measured velocity error x
ipv = interp1(meas_vel_times, meas_vel_lin(2,:), cmd_vel_times); % Interpolated vector
emy = cmd_vel_lin(2,:) - ipv; % Measured velocity error y
ipv = interp1(meas_vel_times, meas_vel_ang(3,:), cmd_vel_times); % Interpolated vector
emp = cmd_vel_ang(3,:) - ipv; % Measured orientation velocity error

ipv = interp1(amcl_times(2:end), amcl_vel(1,:), cmd_vel_times); % Interpolated vector
eax = cmd_vel_lin(1,:) - ipv; % AMCL velocity error x
ipv = interp1(amcl_times(2:end), amcl_vel(2,:), cmd_vel_times); % Interpolated vector
eay = cmd_vel_lin(2,:) - ipv; % AMCL velocity error y
ipv = interp1(amcl_times(2:end), amcl_vel(3,:), cmd_vel_times); % Interpolated vector
eap = cmd_vel_ang(3,:) - ipv; % AMCL orientation velocity error

ipv = interp1(amcl_times(2:end), amcl_vel(1,:), meas_vel_times); % Interpolated vector
eox = ipv - meas_vel_lin(1,:);
ipv = interp1(amcl_times(2:end), amcl_vel(2,:), meas_vel_times); % Interpolated vector
eoy = ipv - meas_vel_lin(2,:);
ipv = interp1(amcl_times(2:end), amcl_vel(3,:), meas_vel_times); % Interpolated vector
eop = ipv - meas_vel_ang(3,:);

%% Plot results
velfig = figure;
set(velfig,'Name','Velocities');
%set(figvel,'defaulttextinterpreter','latex');
subplot(3,1,1);
plot(cmd_vel_times-cmd_vel_times(1), cmd_vel_lin(1,:),'color',ps.tuecyan,'LineWidth',ps.linewidth); hold on
plot(meas_vel_times-cmd_vel_times(1), meas_vel_lin(1,:),'color',ps.tuegreen,'LineWidth',ps.linewidth);
plot(amcl_times(2:end)-cmd_vel_times(1), amcl_vel(1,:),'color',ps.tuepink,'LineWidth',ps.linewidth);
yl1 = ylabel('v_x [m/s]');
subplot(3,1,2);
plot(cmd_vel_times-cmd_vel_times(1), cmd_vel_lin(2,:),'color',ps.tuecyan,'LineWidth',ps.linewidth); hold on
plot(meas_vel_times-cmd_vel_times(1), meas_vel_lin(2,:),'color',ps.tuegreen,'LineWidth',ps.linewidth);
plot(amcl_times(2:end)-cmd_vel_times(1), amcl_vel(2,:),'color',ps.tuepink,'LineWidth',ps.linewidth);
yl2 = ylabel('v_y [m/s]');
subplot(3,1,3);
plot(cmd_vel_times-cmd_vel_times(1), cmd_vel_ang(3,:),'color',ps.tuecyan,'LineWidth',ps.linewidth); hold on
plot(meas_vel_times-cmd_vel_times(1), meas_vel_ang(3,:),'color',ps.tuegreen,'LineWidth',ps.linewidth);
plot(amcl_times(2:end)-cmd_vel_times(1), amcl_vel(3,:),'color',ps.tuepink,'LineWidth',ps.linewidth);
yl3 = ylabel('v_{\theta} [rad/s]');

%% Plot errors
errorfig = figure;
set(errorfig,'Name','Errors');
subplot(3,1,1);
plot(cmd_vel_times-cmd_vel_times(1), emx, 'color', ps.tuegreen, 'LineWidth', ps.linewidth); hold on;
plot(cmd_vel_times-cmd_vel_times(1), eax, 'color', ps.tuepink, 'LineWidth', ps.linewidth); hold on;
subplot(3,1,2);
plot(cmd_vel_times-cmd_vel_times(1), emy, 'color', ps.tuegreen, 'LineWidth', ps.linewidth); hold on;
plot(cmd_vel_times-cmd_vel_times(1), eay, 'color', ps.tuepink, 'LineWidth', ps.linewidth); hold on;
subplot(3,1,3);
plot(cmd_vel_times-cmd_vel_times(1), emp, 'color', ps.tuegreen, 'LineWidth', ps.linewidth); hold on;
plot(cmd_vel_times-cmd_vel_times(1), eap, 'color', ps.tuepink, 'LineWidth', ps.linewidth); hold on;

%% Plot difference between odom and amcl
errorfig2 = figure;
set(errorfig2,'Name','Difference between odom and measurement');
subplot(3,1,1);
plot(meas_vel_times-cmd_vel_times(1), eox, 'color', ps.tuepink, 'LineWidth', ps.linewidth);
subplot(3,1,2);
plot(meas_vel_times-cmd_vel_times(1), eoy, 'color', ps.tuepink, 'LineWidth', ps.linewidth);
subplot(3,1,3);
plot(meas_vel_times-cmd_vel_times(1), eop, 'color', ps.tuepink, 'LineWidth', ps.linewidth);

%% Plot imu: orientation
orientfig = figure;
set(orientfig,'Name','IMU Orientation');
subplot(3,1,1);
plot(imu_times - cmd_vel_times(1), imu_rpy(1,:), 'color', ps.tuecyan, 'LineWidth', ps.linewidth);
subplot(3,1,2);
plot(imu_times - cmd_vel_times(1), imu_rpy(2,:), 'color', ps.tuecyan, 'LineWidth', ps.linewidth);
subplot(3,1,3);
plot(imu_times - cmd_vel_times(1), imu_rpy(3,:), 'color', ps.tuecyan, 'LineWidth', ps.linewidth);

%% IMU: accelerations
orientfig = figure;
set(orientfig,'Name','IMU Linear Acceleration');
subplot(3,1,1);
plot(imu_times - cmd_vel_times(1), imu_lin_acc(1,:), 'color', ps.tuecyan, 'LineWidth', ps.linewidth);
grid;
subplot(3,1,2);
plot(imu_times - cmd_vel_times(1), imu_lin_acc(2,:), 'color', ps.tuecyan, 'LineWidth', ps.linewidth);
grid;
subplot(3,1,3);
plot(imu_times - cmd_vel_times(1), imu_lin_acc(3,:), 'color', ps.tuecyan, 'LineWidth', ps.linewidth);
grid;

%% IMU: velocities
orientfig = figure;
set(orientfig,'Name','IMU Velocity');
subplot(3,1,1);
plot(imu_times - cmd_vel_times(1), imu_vel(1,:), 'color', ps.tuecyan, 'LineWidth', ps.linewidth);
grid;
subplot(3,1,2);
plot(imu_times - cmd_vel_times(1), imu_vel(2,:), 'color', ps.tuecyan, 'LineWidth', ps.linewidth);
grid;
subplot(3,1,3);
plot(imu_times - cmd_vel_times(1), imu_vel(3,:), 'color', ps.tuecyan, 'LineWidth', ps.linewidth);
grid;

%% IMU: angular velocities
orientfig = figure;
set(orientfig,'Name','IMU Angular Velocity');
subplot(3,1,1);
plot(imu_times - cmd_vel_times(1), imu_ang_vel(1,:), 'color', ps.tuecyan, 'LineWidth', ps.linewidth);
grid;
subplot(3,1,2);
plot(imu_times - cmd_vel_times(1), imu_ang_vel(2,:), 'color', ps.tuecyan, 'LineWidth', ps.linewidth);
grid;
subplot(3,1,3);
plot(imu_times - cmd_vel_times(1), imu_ang_vel(3,:), 'color', ps.tuecyan, 'LineWidth', ps.linewidth);
grid;

%% Command