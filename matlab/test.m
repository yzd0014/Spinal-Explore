clear;
 tspan=0:0.0001:20;
y0=0;
[t,y]=ode45(@(t,y) (sin(t)-y)/-10, tspan, y0);
plot(t, y);
M=readmatrix('plot_data.csv');
plot(M(:,1),M(:,2),t,y);
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
