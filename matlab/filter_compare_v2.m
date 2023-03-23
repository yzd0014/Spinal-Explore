dt = 0.0001;
T = 10;
t = 0:dt:(T-dt);
fs = 1/dt;

% calculate cost after ts
ts = 2;

% Use 5th order poly
l = poly([1 10 4 6]);
dl = polyder(l);
alpha = 0.1
sig = polyval(l,t) + alpha*polyval(dl,t);

figure; hold all;
  plot(t,polyval(l,t),'r');
  plot(t,polyval(dl,t),'k');
  plot(t,sig,'c');

N = 100;
fc = logspace(-5,1,N);
filtsig = zeros(numel(t),N);
score = zeros(N,1);
for i=1:numel(fc)
  [b,a] = butter(1,2*fc(i)/fs,'high');
  %filtsig(:,i) = filter(b,a,sig);
  filtsig(:,i) = filter(b,a,sig);
  score(i) = rms(normalize(polyval(dl,t(t>ts)),'zscore')' ...
                  - normalize(filtsig(find(t>ts),i),'zscore'));
end

figure;
  plot(filtsig);

figure;
  plot(score); hold all;
  [vq,iq] = min(score);
  plot(iq,score(iq),'or');

figure; hold all;
  plot(normalize(filtsig,'range'));
  plot(normalize(polyval(dl,t),'range'),'k','LineWidth',3);
  plot(normalize(polyval(l,t),'range'),'r','LineWidth',3);

figure; hold all;
  plot(normalize(polyval(dl,t),'zscore'));
  plot(normalize(filtsig(:,iq),'zscore'));
  ylabel('zscore');
  xlabel('time (s)');
  legend('dl','dl-est');

  hold off;

