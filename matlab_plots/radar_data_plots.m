close all;clear;clc;


dataset = readtable("6843_data_log_beam.csv");



% Specify data range to use
dataset = dataset(:, :); % all


% Init arrays
a = ones(height(dataset),1); % msg_x
b = zeros(height(dataset),1); % msg_y

% Filter data 
% Check if velocity is more than 0,
% Check if xy values are within [0.4 1.5 -0.7 0.9]
%for i=1:height(dataset)
%    if dataset.msg_velocity(i) ~= 0 && dataset.msg_x(i) < 1.5 && dataset.msg_x(i) > 0.4 && dataset.msg_y(i) > -0.7 && dataset.msg_y(i) < 0.9
%        a(i) = dataset.msg_x(i);
%        b(i) = dataset.msg_y(i);
%    end
%end

% Plot 3 graphs

hold on
plot(dataset.timestampsec, dataset.bestX,'.')
ylabel("Displacement [m]")
xlabel('Time[sec]')
grid minor
set(gca,'FontSize',12)
saveas(gcf, "eps-figures/beam", "epsc")
saveas(gcf, "eps-figures/beam", "png")

