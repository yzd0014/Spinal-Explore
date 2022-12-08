clear;
 tspan=0:0.0001:20;
y0=0;
[t,y]=ode45(@(t,y) (sin(t)-y)/-10, tspan, y0);
plot(t, y);
M=readmatrix('plot_data.csv');
plot(M(:,1),M(:,2),t,y,LineWidth=2);
%%
clear;
tspan = [0 5];
y0 = 0;
[t,y] = ode45(@(t,y) 2*t, tspan, y0);
plot(t,y,'-o')


%% 
clear;
M=readmatrix('plot_data.csv');
plot(M(:,1),M(:,2));
%% 
clear;
dt=0.0001;
t=0:dt:500;

u1=@(t) 0.5*square(t)+0.5;
u2=@(t) u1(3*t);
u=@(t) u2(t)-u1(t);
figure;
plot(t,u1(t),LineWidth=2);
figure;
plot(t,u2(t),LineWidth=2);
figure;
plot(t,u(t),LineWidth=2);

% y0 = 0;
% c=10;
% [t,y]=ode45(@(t,y) (u(t)-y)/c, t, y0);
% figure
% plot(t,y,LineWidth=2);
%% 
clear;

y=[0 0.1 0.2 0.3 0.4 0.5 0.6 0.7];
x=[0 1.1 1.3 1.6 2.0 2.7 4.3 10];

p = polyfit(x, y, 7);
x = linspace(0,10);
y = p(1)*x.^6 + p(2)*x.^5+ p(3)*x.^4 + p(4)*x.^3 + p(5)*x.^2 + p(6)*x + p(7);
plot(x,y);

%% 
clear;
a=zeros(3,1);
a(logical([1,0,1]))=9;
%% 
clear;
fc = 200;
fs = 1000;

[b,a] = butter(6,fc/(fs/2));

freqz(b,a,[],fs)

subplot(2,1,1)
ylim([-100 20])

dataIn = randn(1000,1);
dataOut = filter(b,a,dataIn);