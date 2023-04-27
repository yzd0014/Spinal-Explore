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

% M=readmatrix('plot.csv');
% subplot(6,1,1);
% plot(M(:,1),M(:,2),LineWidth=2);
% refline(0,0)
% xlabel('t');
% ylabel('qpos');
% subplot(6,1,2);
% plot(M(:,1),M(:,3),LineWidth=2);
% refline(0,0);
% xlabel('t');
% ylabel('qvel');
% subplot(6,1,3);
% plot(M(:,1),M(:,4),LineWidth=0.2);
% xlabel('t');
% ylabel('right ctrl');
% subplot(6,1,4);
% plot(M(:,1),M(:,5),LineWidth=0.2);
% xlabel('t');
% ylabel('right length');
% subplot(6,1,5);
% plot(M(:,1),M(:,6),LineWidth=0.2);
% xlabel('t');
% ylabel('left ctrl');
% subplot(6,1,6);
% plot(M(:,1),M(:,7),LineWidth=0.2);
% xlabel('t');
% ylabel('left length');


% M=readmatrix('plot.csv');
% p = plot(M(:,1),M(:,2), M(:,1),M(:,3),M(:,1),M(:,4),M(:,1),M(:,6),LineWidth=1);
% refline(0,0)

% xL = xlim;
% yL = ylim;
% line(xL, [0 0]);  %y-axis
% legend('right','left');

% M=readmatrix('plot.csv');
% %plot(M(:,1),M(:,3),LineWidth=2);
% plot(M(:,1),M(:,2),M(:,1),M(:,3),LineWidth=2);
% legend('1','2');
% hold on;
% %plot(M(:,1),circshift(M(:,2), 1611),LineWidth=2);
% %xlim([0.98 1.04]);
% xlabel('time (seconds)');
% ylabel('filtered value');

% clear;
% x = 0:0.02:1;
% y = 0:0.02:1;
% [X, Y] = meshgrid(x,y);
% F = readmatrix('plot.csv');
% surf(X,Y,F);
% xlabel('Kl(X)');
% ylabel('Kv(Y)');

clear;
M=readmatrix('plot.csv');
plot(M(:,1),M(:,2),M(:,1),M(:,3),M(:,1),M(:,4),LineWidth=1);
%plot(M(:,1),M(:,2),M(:,1),M(:,3),M(:,1),M(:,4),M(:,1),M(:,5),LineWidth=1);
refline(0,0);
legend('velocity+length','velocity','length');
%% 
clear;
m0=readmatrix('p0.csv');
m1=readmatrix('p1.csv');

t = m0(1:8000,1);
e0 = m0(1:8000,2);
e1 = m1(1:8000,2);
plot(t,e0,t,e1,LineWidth=1);
refline(0,0);
legend('RI + PD','PD only');
%% 
clear;
m0=readmatrix('plot_0.csv');
m1=readmatrix('plot_1.csv');

t = m0(1:8000,1);
ref = m0(1:8000,2);
r1 = m0(1:8000,3);
r2 = m1(1:8000,2);
plot(t,ref,t,r1,t,r2,LineWidth=1);
refline(0,0);
legend('tracking','RI + PD','PD only');

%% 
clear;
m0=readmatrix('t0.csv');
m1=readmatrix('t1.csv');

t = m0(1:7000,1);
e0 = m0(1:7000,2);
e1 = m1(1:7000,2);
plot(t,e0,t,e1,LineWidth=1);
refline(0,0);
legend('RI + PD','PD only');
%% 
clear;
m0=readmatrix('swing0.csv');
m1=readmatrix('swing1.csv');

t = m0(1:13700,1);
e0 = m0(1:13700,2);
e1 = m1(1:13700,2);
plot(t,e0,t,e1,LineWidth=3);
refline(0,0);
legend('RI + PD','PD only');













