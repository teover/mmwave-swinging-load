clc;clear
figure(1)
hold on
plot3( [0  0  0; 0  0  0; 0  0  1], [0  0  0; 0  0  0; 0  0  0], [0  0  0; 0  0  0; 0  0  0], 'd-r')
text(1, 0.2, 0, '\bf x', 'FontSize', 12)
plot3( [0  0  0; 0  0  0; 0  0  0], [0  0  0; 0  1  0; 0  0  0], [0  0  0; 0  0  0; 0  0  0], 'd-g')
text(0, 1.2, 0, '\bf y', 'FontSize', 12)
plot3( [0  0  0; 0  0  0; 0  0  0], [0  0  0; 0  0  0; 0  0  0], [1  0  0; 0  0  0; 0  0  0], 'd-b')
text(0, 0.2, 1, '\bf z', 'FontSize', 12)
grid on
axis([-1 2    -1 2    -1 2])
view([42 13.2]);
% axis equal