clear;
% M1=readmatrix('plot_data.csv');
% M2=csvread('plot_data_0.6.csv');
% 
% subplot(1,2,1);
% plot(M1(:,1),M1(:,2), '.', M2(:,1),M2(:,2), '.');
% subplot(1,2,2);
% plot(M2(:,1),M2(:,2));

% M = readmatrix('plot_data.csv');
% A = readmatrix('plot_data2.csv');
% plot(M(:,1),M(:,2), '.', 'Color',[0,0,0.9]);
% hold on;
% plot(M(:,1),M(:,3), '.', 'Color',[0,0,0.9]);
% hold on;
% plot(M(:,1),M(:,4), '.', 'Color',[0,0,0.9]);
% hold on;
% 
% plot(A(:,1),A(:,2), '.', 'Color',[1,0,0]);
% hold on;
% plot(A(:,1),A(:,3), '.', 'Color',[1,0,0]);
% hold on;
% plot(A(:,1),A(:,4), '.', 'Color',[1,0,0]);
% 
% xlabel('angle')
% ylabel('torque')
% % legend('right','left');
% xL = xlim;
% yL = ylim;
% line([0 0], yL);  %x-axis
% line(xL, [0 0]);  %y-axis

% M=readmatrix('plot_data.csv');
% subplot(2,1,1);
% plot(M(:,1),M(:,2), M(:,1),M(:,3), LineWidth=2);
% legend('right','left');
% xlabel('time (seconds)');
% ylabel('control signal frequency');
% subplot(2,1,2);
% plot(M(:,1),M(:,4),LineWidth=2);
% xlabel('time (seconds)');
% ylabel('angle position');

% M=readmatrix('plot_data.csv');
% p = plot(M(:,1),M(:,2), M(:,1),M(:,3),LineWidth=2);
% xlabel('time (seconds)');
% ylabel('actuator torque');
% xL = xlim;
% yL = ylim;
% line(xL, [0 0]);  %y-axis
% legend('right','left');

M=readmatrix('plot.csv');
p = plot(M(:,1),M(:,2),LineWidth=2);
xlabel('time (seconds)');
ylabel('filtered value');
