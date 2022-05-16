close all;clear;clc;

% Linear logs
data_lin1 = readtable("logss/25.03-experiments/6843_1843_datalog_480-lin.csv");
data_lin2 = readtable("logss/25.03-experiments/6843_1843_datalog_645-lin.csv");
data_lin3 = readtable("logss/25.03-experiments/6843_1843_datalog_676-lin.csv");
data_lin4 = readtable("logss/25.03-experiments/6843_1843_datalog_875-lin.csv");

% Rotary logs
data_rot1 = readtable("logss/25.03-experiments/6843_1843_datalog_65-rot.csv");
data_rot2 = readtable("logss/25.03-experiments/6843_1843_datalog_365-rot.csv");
data_rot3 = readtable("logss/25.03-experiments/6843_1843_datalog_411-rot.csv");
data_rot4 = readtable("logss/25.03-experiments/6843_1843_datalog_421-rot.csv");

% Logs with IMU
data_imu = readtable("logss/25.03-experiments/velocity_data.csv");


% Select dataset to use
dataset = data_rot4;

% Specify data range to use
dataset = dataset(86:350, :);

% Specify data range to use imu
data_imu = data_imu(1:650, :);


% Init arrays
a = ones(height(dataset),1); % msg_x
b = zeros(height(dataset),1); % msg_y
adj_time = zeros(height(dataset),1); % msg_y

% Filter radar data 
% Check if velocity is more than 0,
% Check if xy values are within [0.4 1.5 -0.7 0.9]
for i=1:height(dataset)
    if dataset.msg_velocity(i) ~= 0 && dataset.msg_x(i) < 1.5 && dataset.msg_x(i) > 0.4 && dataset.msg_y(i) > -0.7 && dataset.msg_y(i) < 0.9
        a(i) = dataset.msg_x(i);
        b(i) = dataset.msg_y(i);
    end
    adj_time(i) = dataset.rospy_get_time(i);

end

AWR1843 = "measurement radar";
IWR6843 = "reference radar";

% Plot 3 graphs

hold on
plot(adj_time, a) % dataset.msg_x
plot(adj_time, dataset.bestX)
legend(AWR1843, IWR6843)
ylabel("X-postion [m]")
xlabel('Time [s]')
grid minor
axis([4.3 16.4 0.75 1.25])
set(gca,'FontSize',12)
saveas(gcf, "eps-figures/fig_x_pos", "epsc")
saveas(gcf, "fig-files/fig_x_pos", "fig")
figure

hold on
plot(data_imu.time, data_imu.radar_velocity)
plot(data_imu.time, data_imu.imu_angular_velocity_x)
legend(AWR1843, "IMU reference")
ylabel("Velocity [m/s]")
xlabel('Time [s]')
axis([0 16.4 -0.8 0.8])
grid minor
set(gca,'FontSize',12)
saveas(gcf, "eps-figures/fig_velocity", "epsc")
saveas(gcf, "fig-files/fig_velocity", "fig")

figure

plot(adj_time, b) % dataset.msg_y
legend(AWR1843)
ylabel("Y-position [m]")
xlabel('Time [s]')
grid minor
axis([4.3 16.4 -0.4 0.4])
set(gca,'FontSize',12)
saveas(gcf, "eps-figures/fig_y_pos", "epsc")
saveas(gcf, "fig-files/fig_y_pos", "fig")

figure


angle = -atand((data_imu.radar_x-1.03)/0.7); % Offset is 1.03 meter in this experiment

hold on
plot(data_imu.time, angle)
plot(data_imu.time, data_imu.imu_roll*(180/pi))
legend(AWR1843, "IMU reference")
ylabel("Angle [deg]")
xlabel('Time [s]')
axis([0 12.5 -25 20])
grid minor
set(gca,'FontSize',12)
saveas(gcf, "eps-figures/fig_angle_x", "epsc")
saveas(gcf, "fig-files/fig_angle_x", "fig")

