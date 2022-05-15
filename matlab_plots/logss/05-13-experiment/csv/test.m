close all;clear;clc;

% Linear logs
data_lin1 = readtable("2022-05-13-15-20-31-linear-radar_scan.csv");
data_lin2 = readtable("2022-05-13-16-46-13-linear-no-grouping-radar_scan.csv");

% Rotary logs
data_rot1 = readtable("2022-05-13-16-47-41-rotary-no-grouping-radar_scan");

% Select dataset to use
dataset = data_rot1;

% Specify data range to use
%dataset = dataset(86:350, :);


% Init arrays
a = ones(height(dataset),1); % msg_x
b = zeros(height(dataset),1); % msg_y
adj_time = zeros(height(dataset),1); % msg_y

% Filter radar data 
% Check if velocity is more than 0,
% Check if xy values are within [0.4 1.5 -0.7 0.9]
for i=1:height(dataset)
    if dataset.x_velocity(i) ~= 0 && dataset.x_x(i) < 1.2 && dataset.x_x(i) > 0.7 %&& dataset.x_y(i) > -0.7 && dataset.x_y(i) < 0.9
        a(i) = dataset.x_x(i);
        b(i) = dataset.x_y(i);
    end
    %adj_time(i) = dataset.rospy_get_time(i);

end

AWR1843 = "measurement radar";
IWR6843 = "reference radar";

% Plot 3 graphs

hold on
plot(data_lin1.x_velocity, '-.')
%plot(a,'.') % dataset.msg_x
legend(AWR1843, IWR6843)
ylabel("X-postion [m]")
xlabel('Time [s]')
grid minor
set(gca,'FontSize',12)
figure

hold on
plot(data_lin1.x_x, '.')
plot(a,'.') % dataset.msg_x
legend(AWR1843, IWR6843)
ylabel("Y-postion [m]")
xlabel('Time [s]')
grid minor
set(gca,'FontSize',12)

