% Script to plot base performance
clear all;
%close all; 
clc;

% ToDo: 
% lowpass filtering certain signals
% highpass imu signals
% fft plots
% check rotation stuff
% To get rid of drift:
%%% Correct acceleration: should be possible to correct this statically
%%% To do this, do a dedicated experiment: first: IMU level, then, rotated
%%% roll, wait, rotate pitch, wait, rotate yaw...
 
%% Parameters
robot = '/sergio'; % Put a slash before the robot name
date  = '20141124';
type  = 'corridor';
id    = '01';
plotsettings;

sampts = 0.01;      % Resampling period
sampme = 'pchip';  % Resampling method

lpf    = 6;      % Cut-off frequency of the lowpass filter
hpf    = 0.1;    % Pole frequency of the highpass filter
g      = -9.877;  % Gravity acceleration

%% Read bag file
bagfilename = strcat('/home/amigo/ros/data/recorded/rosbags/base_performance/',date,'/',robot,'_',type,'_',id,'.bag');
%bagfilename = strcat('/home/amigo/ros/data/recorded/rosbags/base_performance/',date,'/',robot,id,'.bag');
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

%% Compute errors
% Error between command velocity and odom
ipv = interp1(meas_vel_times, meas_vel_lin(1,:), cmd_vel_times); % Interpolated vector
emx = cmd_vel_lin(1,:) - ipv; % Measured velocity error x
ipv = interp1(meas_vel_times, meas_vel_lin(2,:), cmd_vel_times); % Interpolated vector
emy = cmd_vel_lin(2,:) - ipv; % Measured velocity error y
ipv = interp1(meas_vel_times, meas_vel_ang(3,:), cmd_vel_times); % Interpolated vector
emp = cmd_vel_ang(3,:) - ipv; % Measured orientation velocity error
em  = [emx;emy;emp]; clear emx emy emp

% Error between command velocity and amcl velocity
ipv = interp1(amcl_times(2:end), amcl_vel(1,:), cmd_vel_times); % Interpolated vector
eax = cmd_vel_lin(1,:) - ipv; % AMCL velocity error x
ipv = interp1(amcl_times(2:end), amcl_vel(2,:), cmd_vel_times); % Interpolated vector
eay = cmd_vel_lin(2,:) - ipv; % AMCL velocity error y
ipv = interp1(amcl_times(2:end), amcl_vel(3,:), cmd_vel_times); % Interpolated vector
eap = cmd_vel_ang(3,:) - ipv; % AMCL orientation velocity error
ea  = [eax;eay;eap]; clear eax eay eap

% Error between command velocity and imu velocity
ipv = interp1(imu_times, imu_vel(1,:), cmd_vel_times); % Interpolated vector
eix = cmd_vel_lin(1,:) - ipv; % imu velocity error x
ipv = interp1(imu_times, imu_vel(2,:), cmd_vel_times); % Interpolated vector
eiy = cmd_vel_lin(2,:) - ipv; % imu velocity error y
ipv = interp1(imu_times, imu_vel(3,:), cmd_vel_times); % Interpolated vector
eip = cmd_vel_ang(3,:) - ipv; % imu orientation velocity error
ei  = [eix;eiy;eip]; clear eix eiy eip
% for ii = 1:3;
%     fprintf('Highpass imu velocity error\n')
%     ei(ii,:) = highpass(ei(ii,:),1/sampts,hpf);
% end

% Error between odom velocity and amcl velocity
ipv = interp1(amcl_times(2:end), amcl_vel(1,:), meas_vel_times); % Interpolated vector
eox = ipv - meas_vel_lin(1,:);
ipv = interp1(amcl_times(2:end), amcl_vel(2,:), meas_vel_times); % Interpolated vector
eoy = ipv - meas_vel_lin(2,:);
ipv = interp1(amcl_times(2:end), amcl_vel(3,:), meas_vel_times); % Interpolated vector
eop = ipv - meas_vel_ang(3,:);
eo  = [eox;eoy;eop]; clear eox eoy eop

%% Plot results
velfig = figure;
set(velfig,'Name','Velocities');
%set(figvel,'defaulttextinterpreter','latex');
subplot(3,1,1);
plot(cmd_vel_times-cmd_vel_times(1), cmd_vel_lin(1,:),'color',ps.tuecyan,'LineWidth',ps.linewidth); hold on
plot(meas_vel_times-cmd_vel_times(1), meas_vel_lin(1,:),'color',ps.tuegreen,'LineWidth',ps.linewidth);
plot(amcl_times(2:end)-cmd_vel_times(1), amcl_vel(1,:),'color',ps.tuepink,'LineWidth',ps.linewidth);
plot(imu_times - cmd_vel_times(1), imu_vel(1,:), 'color', ps.tuedarkblue, 'LineWidth', ps.linewidth);
yl1 = ylabel('v_x [m/s]');
subplot(3,1,2);
plot(cmd_vel_times-cmd_vel_times(1), cmd_vel_lin(2,:),'color',ps.tuecyan,'LineWidth',ps.linewidth); hold on
plot(meas_vel_times-cmd_vel_times(1), meas_vel_lin(2,:),'color',ps.tuegreen,'LineWidth',ps.linewidth);
plot(amcl_times(2:end)-cmd_vel_times(1), amcl_vel(2,:),'color',ps.tuepink,'LineWidth',ps.linewidth);
plot(imu_times - cmd_vel_times(1), imu_vel(2,:), 'color', ps.tuedarkblue, 'LineWidth', ps.linewidth);
yl2 = ylabel('v_y [m/s]');
subplot(3,1,3);
plot(cmd_vel_times-cmd_vel_times(1), cmd_vel_ang(3,:),'color',ps.tuecyan,'LineWidth',ps.linewidth); hold on
plot(meas_vel_times-cmd_vel_times(1), meas_vel_ang(3,:),'color',ps.tuegreen,'LineWidth',ps.linewidth);
plot(amcl_times(2:end)-cmd_vel_times(1), amcl_vel(3,:),'color',ps.tuepink,'LineWidth',ps.linewidth);
plot(imu_times - cmd_vel_times(1), imu_vel(3,:), 'color', ps.tuedarkblue, 'LineWidth', ps.linewidth);
yl3 = ylabel('v_{\theta} [rad/s]');

%% Plot errors
errorfig = figure;
set(errorfig,'Name','Errors');
subplot(3,1,1);
plot(cmd_vel_times-cmd_vel_times(1), em(1,:), 'color', ps.tuegreen, 'LineWidth', ps.linewidth); hold on;
plot(cmd_vel_times-cmd_vel_times(1), ea(1,:), 'color', ps.tuepink, 'LineWidth', ps.linewidth); hold on;
plot(cmd_vel_times-cmd_vel_times(1), ei(1,:), 'color', ps.tuedarkblue, 'LineWidth', ps.linewidth); hold on;
subplot(3,1,2);
plot(cmd_vel_times-cmd_vel_times(1), em(2,:), 'color', ps.tuegreen, 'LineWidth', ps.linewidth); hold on;
plot(cmd_vel_times-cmd_vel_times(1), ea(2,:), 'color', ps.tuepink, 'LineWidth', ps.linewidth); hold on;
plot(cmd_vel_times-cmd_vel_times(1), ei(2,:), 'color', ps.tuedarkblue, 'LineWidth', ps.linewidth); hold on;
subplot(3,1,3);
plot(cmd_vel_times-cmd_vel_times(1), em(3,:), 'color', ps.tuegreen, 'LineWidth', ps.linewidth); hold on;
plot(cmd_vel_times-cmd_vel_times(1), ea(3,:), 'color', ps.tuepink, 'LineWidth', ps.linewidth); hold on;
plot(cmd_vel_times-cmd_vel_times(1), ei(3,:), 'color', ps.tuedarkblue, 'LineWidth', ps.linewidth); hold on;

%% Plot difference between odom and amcl
errorfig2 = figure;
set(errorfig2,'Name','Difference between odom and measurement');
subplot(3,1,1);
plot(meas_vel_times-cmd_vel_times(1), eo(1,:), 'color', ps.tuepink, 'LineWidth', ps.linewidth);
subplot(3,1,2);
plot(meas_vel_times-cmd_vel_times(1), eo(2,:), 'color', ps.tuepink, 'LineWidth', ps.linewidth);
subplot(3,1,3);
plot(meas_vel_times-cmd_vel_times(1), eo(3,:), 'color', ps.tuepink, 'LineWidth', ps.linewidth);

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

%% Power spectral densities imu vels
% Fs = 1/sampts;   
% nfft = 2^nextpow2(length(imu_times));
% figure('Name','PSD');
% for ii = 1:3;
%     Pxx = abs(fft(imu_vel(ii,:),nfft)).^2/length(imu_times)/Fs;
% 
%     % Create a single-sided spectrum
%     Hpsd = dspdata.psd(Pxx(1:length(Pxx)/2),'Fs',Fs);  
%     subplot(3,1,ii);
%     plot(Hpsd);
%     set(gca,'XScale','log');
% end