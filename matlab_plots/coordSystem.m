clc;clear;close all
figure
grid on
hold on

textOffset = 0.2;

% vars for rotation arrow
start_angle = pi/6;
end_angle = 3*pi/2;
center_x = [0.7 0 0];
center_y = [0 0.7 0];
center_z = [0 0 0.7];
norm_x = [1 0 0];
norm_y = [0 1 0];
norm_z = [0 0 1];
rad = 0.1;
len = 0.05;
tipWidth = 0.06;
tipLength = 0.05;
stemWidth = 0.02;
circLineWidth = 3;

% arrows
x = mArrow3([0 0 0],[1 0 0],'color','red','stemWidth',0.02,'facealpha',0.8);
text(1, textOffset, 0, '\bf x', 'FontSize', 12)

y = mArrow3([0 0 0],[0 1 0],'color','green','stemWidth',0.02,'facealpha',0.8);
text(0, 1+textOffset, 0, '\bf y', 'FontSize', 12)

z = mArrow3([0 0 0],[0 0 1],'color','blue','stemWidth',0.02,'facealpha',0.8);
text(0, textOffset, 1, '\bf z', 'FontSize', 12)

% rotation arrows
vecarrow(start_angle,end_angle,center_x,norm_x,rad,len,tipWidth,tipLength,stemWidth,circLineWidth);
text(1, -textOffset, textOffset, '\bf \alpha', 'FontSize', 12)

vecarrow(end_angle,start_angle,center_y,norm_y,rad,len,tipWidth,tipLength,stemWidth,circLineWidth);
text(0, 1-textOffset, textOffset, '\bf \beta', 'FontSize', 12)

vecarrow(start_angle,end_angle,center_z,norm_z,rad,len,tipWidth,tipLength,stemWidth,circLineWidth);
text(0, -textOffset, 1+textOffset, '\bf \gamma', 'FontSize', 12)

grid on
%  Camera Framing
xlim([-0.36 1.17]);
ylim([-0.24 1.29]);
zlim([-0.22 1.31]);
view([39.89 12.00]);



% Save as .eps
set(gcf,'renderer','Painters')
saveas(gcf,'eps-figures/coord-system','epsc')
saveas(gcf, "fig-files/coord-system", "fig")

