dt = 0.0001;
T = 10;
t = 0:dt:(T-dt);
fs = 1/dt;

% Use 5th order poly
l = poly([1 10 4 6]);
dl = polyder(l);
alpha = 1
sig = polyval(l,t) + alpha*polyval(dl,t);

figure; hold all;
  plot(t,polyval(l,t),'r');
  plot(t,polyval(dl,t),'k');
  plot(t,sig,'c');

N = 100;
fc = logspace(-5,0,N);
filtsig = zeros(numel(t),N);
for i=1:numel(fc)
  [b,a] = butter(1,2*fc(i)/fs,'high');
  filtsig(:,i) = filter(b,a,sig);
end

figure;
  plot(filtsig);


figure; hold all;
  plot(normalize(filtsig,'range'));
  plot(normalize(polyval(dl,t),'range'),'k','LineWidth',3);
  plot(normalize(polyval(l,t),'range'),'r','LineWidth',3);


