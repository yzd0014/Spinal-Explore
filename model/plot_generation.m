clear;
% M1=csvread('plot_data_0.2.csv');
% M2=csvread('plot_data_0.6.csv');

% subplot(1,2,1);
% plot(M1(:,1),M1(:,2), '.', M2(:,1),M2(:,2), '.');
% subplot(1,2,2);
% plot(M2(:,1),M2(:,2));

M = readmatrix('plot_data.csv');
% A = readmatrix('plot_data2.csv');
plot(M(:,1),M(:,2), '.', 'Color',[0,0,0.9]);
hold on;
plot(M(:,1),M(:,3), '.', 'Color',[0,0,0.9]);
hold on;
% plot(M(:,1),M(:,4), '.', 'Color',[0,0,0.9]);
% hold on;

% plot(A(:,1),A(:,2), '.', 'Color',[1,0,0]);
% hold on;
% plot(A(:,1),A(:,3), '.', 'Color',[1,0,0]);
% hold on;
% plot(A(:,1),A(:,4), '.', 'Color',[1,0,0]);

xlabel('angle')
ylabel('torque')
% legend('right','left');
xL = xlim;
yL = ylim;
line([0 0], yL);  %x-axis
line(xL, [0 0]);  %y-axis