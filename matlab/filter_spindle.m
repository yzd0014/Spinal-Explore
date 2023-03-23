clear;
M = readmatrix('plot.csv');
sz = size(M);

dt = 0.0001;
fs = 1/dt;
T = M(sz(1),1);
N = 100;

% max = fs/4;
% min = 1/T;
% fc = linspace(min,max,N);

fc = logspace(-4,0,N);

filter_out = zeros(sz(1),N);
for i=1:numel(fc)
  fc_i = fc(i);
  [b,a] = butter(2,2*fc_i/fs,'high');
  sig = M(:,2);
  filter_out(:,i) = filter(b,a,sig);
end

figure;
hold all;
plot(normalize(filter_out,'range'));
plot(normalize(M(:,3),'range'),'k','LineWidth',3);
%plot(normalize(M(:,4),'range'),'r','LineWidth',3);
hold off;
