close all;clear;clc;

% Linear logs
data_lin1 = readtable("logss/05-13-experiment/csv/2022-05-13-15-20-31-linear-radar_scan.csv");              % grouping enabled
data_lin2 = readtable("logss/05-13-experiment/csv/2022-05-13-16-46-13-linear-no-grouping-radar_scan.csv");  % grouping disabled

% Rotary logs
data_rot1 = readtable("logss/05-13-experiment/csv/2022-05-13-16-47-41-rotary-no-grouping-radar_scan");      % grouping disabled

% Select dataset to use
dataset = data_rot1;

% Specify data range to use
%dataset = dataset(86:350, :);


% Init arrays
a = ones(height(dataset),1); % msg_x
b = zeros(height(dataset),1); % msg_y
c = zeros(height(dataset),1);

% Filter radar data 
% Check if velocity is more than 0,
% Check if xy values are within [0.4 1.4 -0.7 0.9 -0.5 0.5]
for i=1:height(dataset)
    if dataset.x_velocity(i) ~= 0 && ...                    % Check if velocity is not zero
       dataset.x_x(i) >  0.7 && dataset.x_x(i) < 1.4 && ...  % Check if x is within ROI
       dataset.x_y(i) > -0.7 && dataset.x_y(i) < 0.9 && ... % Check y 
       dataset.x_z(i) > -0.5 && dataset.x_z(i) < 0.5           % Check z
        a(i) = dataset.x_x(i);
        b(i) = dataset.x_y(i);
        c(i) = dataset.x_z(i);
    end

end

AWR1843 = "measurement radar";
%%

figure
hold on
plot(dataset.time, dataset.x_x, '.')
plot(dataset.time, a,'.') % dataset.msg_x
legend(AWR1843, 'region of interest')
ylabel("X-postion [m]")
xlabel('Time [s]')
grid minor
set(gca,'FontSize',12)
saveas(gcf, "eps-figures/31fig_pile-x", "epsc")
saveas(gcf, "fig-files/31fig_pile-x", "fig")

figure
hold on
plot(dataset.time, dataset.x_y, '.')
plot(dataset.time, b,'.') % dataset.msg_x
legend(AWR1843, 'region of interest')
ylabel("Y-postion [m]")
xlabel('Time [s]')
grid minor
set(gca,'FontSize',12)
saveas(gcf, "eps-figures/32fig_pile-y", "epsc")
saveas(gcf, "fig-files/32fig_pile-y", "fig")

figure
hold on
plot(dataset.time, dataset.x_z, '.')
plot(dataset.time, c,'.') % dataset.msg_x
legend(AWR1843, 'region of interest')
ylabel("Z-postion [m]")
xlabel('Time [s]')
grid minor
set(gca,'FontSize',12)
saveas(gcf, "eps-figures/33fig_pile-z", "epsc")
saveas(gcf, "fig-files/33fig_pile-z", "fig")


figure
% Both is plotted to highlight Region of Interest
plot3(dataset.x_x, dataset.x_y, dataset.x_z, '.')
hold on
plot3(a, b, c, '.')
grid on
xlabel('X [m]')
ylabel("Y [m]")
zlabel('Z [m]')
%set(gcf,'renderer','Painters') % for proper eps fig but crashes matlab
saveas(gcf,'eps-figures/34fig_pile-3d','epsc')
saveas(gcf, "fig-files/34fig_pile-3d", "fig")


