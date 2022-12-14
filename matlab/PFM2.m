clear;
dt = 0.00001;
T = 1;
t = 0:dt:(T-dt);
y = zeros(numel(t),1);

for i=1:length(t)
    y(i) = PulseGeneration(t(i), 10*dt, dt);
end
figure;
plot(t,y);

% for i=1:length(t)
%     [t,y] = ode45(@(t,y) PulseGeneration(t,1000*dt,dt), t(1:i), 0);
% end
[t,y] = ode45(@(t,y) PulseGeneration(t, 2*dt, dt), [0, 1], 0);
figure;
plot(t,y);

function y = PulseGeneration(t, period, dt)
    y = 0;
    if mod(int32(t/dt), int32(period/dt)) == 0
        y = 1;
    end
end