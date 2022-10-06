clear;
M1=csvread('plot_data_0.2.csv');
M2=csvread('plot_data_0.6.csv');

% subplot(1,2,1);
plot(M1(:,1),M1(:,2),M2(:,1),M2(:,2));
% subplot(1,2,2);
% plot(M2(:,1),M2(:,2));