close all;clear;clc;


dataset = readtable("logs/6843_data_log_beam.csv");



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

subplot(2,1,1);
plot(dataset.timestampsec, dataset.closestValue,'.')
hold on
plot(95 ,0.377,'o', 'MarkerSize', 25, 'MarkerEdgeColor','black')
legend('IWR6843 measurement','Data zoom area')
%set(gca,'FontSize',12)

ylabel("Range to floor [m]")
xlabel('Time [sec]')
grid minor
title('(a)')


% hold on
subplot(2,1,2); 
plot(dataset.timestampsec, dataset.closestValue,'.')
ylabel("Range to floor [m]")
xlabel('Time [sec]')
grid minor
axis([90 100 0.374 0.382])
title('(b)')


%set(gca,'FontSize',12)
saveas(gcf, "eps-figures/beam", "epsc")
saveas(gcf, "eps-figures/beam", "png")

