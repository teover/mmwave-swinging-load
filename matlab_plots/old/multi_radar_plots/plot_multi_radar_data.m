close all;clear;clc;


%datalog_imu = readtable("2022-03-09-18_imu2.csv");
datalog = readtable("6843_1843_datalog_241_imu3.csv");

%datalog = readtable("6843_data_log.csv");

% closestValue is used, any can be used. dummy value to have 0 to plot
% against

 plot(datalog.bestX)
 %hold on
 %plot(datalog.msg_x)
 figure

% plot(datalog.bestX)
%hold on
 %plot(datalog.rospy_get_time, datalog.msg_velocity)
%  hold on
% plot(datalog.rospy_Time, datalog.x2, 'g')
% hold on
% plot(datalog.rospy_Time, datalog.x3, 'b')
% grid
% 
%plot(datalog.rospy_Time, (datalog.x1+datalog.x2+datalog.x3)/3, 'r')
%figure
%plot(sqrt(datalog.msg_y.^2+datalog.msg_x.^2), 'g')



%datalog.rospy_Time_now[2]-datalog.rospy_Time_now[3]

set(gca,'FontSize',12)
xlabel('x-axis [m]')
ylabel('y-axis [m]')
legend('AWR1843')
%title('1843 vs 6843 sensor characteristics')


%Fs = 14;            % Sampling frequency                    
%T = 1/Fs;             % Sampling period       
%L = 1362;             % Length of signal
%t = (0:L-1)*T;        % Time vector

%F = (0:L-1)*Fs;

%fftx = fft(datalog.bestX);

%plot(F, abs(fftx))
















