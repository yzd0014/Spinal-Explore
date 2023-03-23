clear;
g_syn = 0.3;
k_syn = 2;
v_pir = 120;
v_L = -60;
v_syn = -80;
C = 1;
g_L = 0.1;
theta = 3;
g_pir = 0.3;
theta_syn = -44;

m_infinity = @(v) 1/(1+exp(-(v+65)/7.8));
S_infinity = @(v) 1/(1+exp(-(v-theta_syn)/k_syn));
dv = @(v_i,v_j,h) (-g_pir*m_infinity(v_i)^3*h*(v_i-v_pir)-g_L*(v_i-v_L)-g_syn*S_infinity(v_j)*(v_i-v_syn))/C;
h_infinity = @(v) 1/(1+exp((v+81)/11));
tao_h = @(v) h_infinity(v)*exp((v+162.3)/17.8);
dh = @(v,h) theta*(h_infinity(v)-h)/tao_h(v);

RI_dynamics = @(t,y) [dv(y(1),y(3),y(2));dh(y(1),y(2));dv(y(3),y(1),y(4));dh(y(3),y(4))];

dt = 0.0001;
tspan = 0:dt:450;
y0=[-30;0;-60;0];

[t,y] = ode45(@(t,y) RI_dynamics(t,y),tspan,y0);
subplot(2,1,1);
plot(t,y(:,1));
subplot(2,1,2);
plot(t,y(:,3));