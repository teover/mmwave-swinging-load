close all;clear;clc;

% 1843 table
timmwaveradarscan = readtable("logss/resolution_logs/2022-03-02-19-19-43-ti_mmwave-radar_scan.csv");
% 6843 table
datalog = readtable("logss/resolution_logs/6843_data_log.csv");

% closestValue is used, any can be used. dummy value to have 0 to plot
% against

scatter(timmwaveradarscan,"x_x","x_y","filled","SizeData", 20, "MarkerFaceColor",'#77AC30')
axis([0 2 -1 1])
grid
set(gca,'FontSize',12)
xlabel('x-axis [m]')
ylabel('y-axis [m]')
legend('AWR1843')
rectangle("Position",[0.6 -0.4 0.8 0.8],'LineWidth',2,'EdgeColor','r')
%title('1843 vs 6843 sensor characteristics')
saveas(gcf, "eps-figures/fig1_1843", "epsc")
saveas(gcf, "fig-files/fig1_1843", "fig")
figure


scatter(timmwaveradarscan,"x_x","x_y","filled", "MarkerFaceColor",'#77AC30')
axis([0.6 1.4 -0.4 0.4])
grid
set(gca,'FontSize',12)
xlabel('x-axis [m]')
ylabel('y-axis [m]')
legend('AWR1843')
%title('1843 vs 6843 sensor characteristics')
saveas(gcf, "eps-figures/fig2_1843_zoom", "epsc")
saveas(gcf, "fig-files/fig2_1843_zoom", "fig")
figure

scatter(datalog,"closestValue","dummy","filled","SizeData", 20)
axis([0 2 -1 1])
grid
set(gca,'FontSize',12)
xlabel('x-axis [m]')
ylabel('y-axis [m]')
rectangle("Position",[0.999 -0.001 0.002 0.002],'LineWidth',2,'EdgeColor','r')
legend('IWR6843')
%title('1843 vs 6843 sensor characteristics')
saveas(gcf, "eps-figures/fig3_6843", "epsc")
saveas(gcf, "fig-files/fig3_6843", "fig")
figure



scatter(datalog,"closestValue","dummy","filled")
hold on
xlabel('x-axis [m]')
ylabel('y-axis [m]')
legend('IWR6843')
%title('1843 vs 6843 sensor characteristics, zoom')
axis([0.999 1.001 -0.001 0.001])
grid
set(gca,'FontSize',12)
saveas(gcf, "eps-figures/fig4_6843_zoom", "epsc")
saveas(gcf, "fig-files/fig4_6843_zoom", "fig")

figure


scatter(datalog,"closestValue","dummy","filled","SizeData", 20)
hold on
scatter(timmwaveradarscan,"x_x","x_y","filled","SizeData", 20, "MarkerFaceColor",'#77AC30')
xlabel('x-axis [m]')
ylabel('y-axis [m]')
legend('IWR6843','AWR1843')
axis([0 1.5 -1.5 1.5])
grid
set(gca,'FontSize',12)
%title('6843 sensor characteristics')
saveas(gcf, "eps-figures/fig5_both", "epsc")
saveas(gcf, "fig-files/fig5_both", "fig")
