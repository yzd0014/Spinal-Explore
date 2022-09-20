clear;
M=csvread('plot_data.csv');
sz=size(M);
dt=0.002;
t=zeros(sz(1),1);
for i=1:sz(1)
    t(i,1)=i*dt;
end
subplot(1,2,1);
plot(t,M(:,1));
subplot(1,2,2);
plot(t,-M(:,2));