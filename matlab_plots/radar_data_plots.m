close all;clear;clc;

% Linear logs
data_lin1 = readtable("logs/25.03-experiments/6843_1843_datalog_480-lin.csv");
data_lin2 = readtable("logs/25.03-experiments/6843_1843_datalog_645-lin.csv");
data_lin3 = readtable("logs/25.03-experiments/6843_1843_datalog_676-lin.csv");
data_lin4 = readtable("logs/25.03-experiments/6843_1843_datalog_875-lin.csv");

% Rotary logs
data_rot1 = readtable("logs/25.03-experiments/6843_1843_datalog_65-rot.csv");
data_rot2 = readtable("logs/25.03-experiments/6843_1843_datalog_365-rot.csv");
data_rot3 = readtable("logs/25.03-experiments/6843_1843_datalog_411-rot.csv");
data_rot4 = readtable("logs/25.03-experiments/6843_1843_datalog_421-rot.csv");

% Logs with IMU
data_imu = readtable("logs/25.03-experiments/velocity_data.csv");


% Select dataset to use
dataset = data_rot4;

% Specify data range to use
dataset = dataset(85:350, :);

% Specify data range to use
data_imu = data_imu(1:650, :);


% Init arrays
a = ones(height(dataset),1); % msg_x
b = zeros(height(dataset),1); % msg_y

% Adjust timestamps

%start_time_imu = data_imu.Var1(1);
%for i=1:height(data_imu)
%    data_imu.Var1(i) = data_imu.Var1(i) - start_time_imu;
%end

%start_time_dataset = 
 



for i=1:height(dataset)
    if dataset.msg_velocity(i) ~= 0 && dataset.msg_x(i) < 1.5 && dataset.msg_x(i) > 0.4 && dataset.msg_y(i) > -0.7 && dataset.msg_y(i) < 0.9
        a(i) = dataset.msg_x(i);
        b(i) = dataset.msg_y(i);
    end
end

hold on
plot(a)
plot(dataset.bestX)
ylabel("x")
xlabel('Samples')
grid minor
axis([0 266 0.75 1.25])
set(gca,'FontSize',12)
figure

hold on
plot(data_imu.time, data_imu.radar_velocity)
plot(data_imu.time, data_imu.imu_angular_velocity_x)
ylabel("Velocity [m/s")
xlabel('Time [s]')
grid minor
set(gca,'FontSize',12)
figure

plot(b)
ylabel("y")
xlabel('Samples')
grid minor
axis([0 266 -0.4 0.4])
set(gca,'FontSize',12)




%axis([0 2 -1 1])
grid
set(gca,'FontSize',12)
%ylabel('x-axis [m]')
%legend('IWR6843')
%saveas(gcf, "fig1_1843", "epsc")
