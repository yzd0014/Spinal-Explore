clear;
% Time stuff
dt = 0.00001;
T = 0.5;
t = 0:dt:(T-dt);
fs = 1/dt;

% impulse pulse period
dt_p = 1000*dt;
% y_hat is equivalent to l* but with pusles and represents the *desired* lenght
y_hat = zeros(numel(t),1);
y_hat(mod(int32(t/dt),int32(dt_p/dt)) == 0) = 1;

figure;
plot(t,y_hat);

% create filter
fc = 10;
% fourth order low pass - you'll have to explore filter parameters
% 4th order gives better (smoother) response that first order
%[b,a] = butter(4,2*fc/fs);
[b,a] = butter(4,2*fc/fs);
% Give you an idea of how the filter *smoothes* the puslese
y_hat_filter=filter(b,a,y_hat);
figure;
  plot(t,y_hat_filter,'k');
  xlabel('Time (s)');
  ylabel('Filter Output')
  title('Ex.1 Filtered Pulses');

%% 

%
%
% Let's simulate the sensor signal now
%
%
%

% impulse pulse freq. same as y_hat
dt_p2 = 100*dt;

% y is the output of the sensor and equivalent to l (lenght) but with pusles
y = zeros(numel(t),1);
y(mod(t,dt_p2) == 0) = 1;

% if y_hat and y are identical, then l-l* = 0
% figure;
%   plot(t,filter(b,a,y-y_hat),'k')
%   xlabel('Time (s)');
%   ylabel('Filter Output')
%   title('Ex.2 Sensor Pulse = Desired Pulse');

% if y is phase shifted, still the output is close to zeros (except the
% transient portion in the beginning). This shifts y by half a period so the
% signals are completely out of phase which should produce the maximum
% variation when both signals have the same freq.
% fp=1/dt_p2;
% y = circshift(y,floor(fs/fp/2));
% 
% figure;
%   plot(t,y,'k'); hold all;
%   plot(t,-y_hat,'r');
%   legend('y','-y_{hat}')
%   xlabel('Time (s)');
%   ylabel('Amplitude')
%   title('Ex.3 Sensor Pulse = Desired Pulse, but phase shifted');
% 
% figure;
%   plot(filter(b,a,y-y_hat))
%   xlabel('Time (s)');
%   ylabel('Filter Output')
%   title('Ex.3 Sensor Pulse = Desired Pulse, but phase shifted');


% y freq. lower than y_hat -- correponsds to the muscle being
% longer than desired
dt_p2 = 999*dt;
y = zeros(numel(t),1);
y(mod(int32(t/dt),int32(dt_p2/dt)) == 0) = 1;
fp=1/dt_p2;
y = circshift(y,floor(fs/fp/2));

figure;
  plot(t,y,'k'); hold all;
  plot(t,-y_hat,'r');
  legend('y','-y_{hat}')
  xlabel('Time (s)');
  ylabel('Amplitude')
  title('Ex.4 Sensor Pulse < Desired Pulse');


figure;
  plot(filter(b,a,y-y_hat))
  xlabel('Time (s)');
  ylabel('Filter Output')
  title('Ex.4 Sensor Pulse < Desired Pulse');
%% 

% impulse pulse freq. higher than y_hat -- correponsds to the muscle being
% longer than desired
dt_p2 = 130*dt;
y = zeros(numel(t),1);
y(mod(int32(t/dt),int32(dt_p2/dt)) == 0) = 1;

figure;
  plot(t,y,'k'); hold all;
  plot(t,-y_hat,'r');
  legend('y','-y_{hat}')
  xlabel('Time (s)');
  ylabel('Amplitude')
  title('Ex.5 Sensor Pulse > Desired Pulse');


figure;
  plot(filter(b,a,y-y_hat))
  xlabel('Time (s)');
  ylabel('Filter Output')
  title('Ex.5 Sensor Pulse > Desired Pulse');


