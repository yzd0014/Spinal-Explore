clear;
dt = 0.00001;
T = 0.2;
t = 0:dt:(T-dt);
tf = 0:dt:0.07;
tf = 0:dt:0.5;
fs = 1/dt;
o_threshold = 0.0003;

fc = 50;
[b,a] = butter(2,2*fc/fs);

dt_p = 100*dt;
y = zeros(numel(tf),1);
y(mod(int32(tf/dt),int32(dt_p/dt)) == 0) = 1;

y_filter=filter(b,a,y);
figure;
plot(tf,y_filter,'k');
hold on
return

length_range = [0,0.7];
f_range = [50,1920];
p = polyfit(length_range,f_range,1);

length_threshold = 0.7:-0.01:0.01;
length_current = 0.7;
length_diff = zeros(length(length_threshold),1);
u_freq = length_diff;
for i=1:length(length_threshold)
    length_diff(i) = length_current - length_threshold(i);

    f1 = polyval(p,length_current);
    dt_p_current = 1/f1;
    y_current = zeros(numel(tf),1);
    y_current(mod(int32(tf/dt),int32(dt_p_current/dt)) == 0) = 1;
    
    f2 = polyval(p,length_threshold(i));
    dt_p_threshold = 1/f2;
    y_threshold = zeros(numel(tf),1);
    y_threshold(mod(int32(tf/dt),int32(dt_p_threshold/dt)) == 0) = 1;

    y_filter = filter(b,a,y_current-y_threshold);
    for j=1:length(tf)
        if y_filter(j) > o_threshold
            u_freq(i) = 1/tf(j);
            break
        end
    end
end
plot(length_diff, u_freq,'k');
xlabel('length difference');
ylabel('output frequency');
return

multiplier = 10:5:2000;
output_f = zeros(length(multiplier),1);
input_f = output_f;
output = zeros(numel(t),1);

for ic = 1:length(multiplier)
    dt_p = multiplier(ic)*dt;
    input_f(ic) = 1/dt_p;
    y = zeros(numel(tf),1);
    y(mod(int32(tf/dt),int32(dt_p/dt)) == 0) = 1;
    y_filter=filter(b,a,y);
    for i=1:length(tf)
        if y_filter(i) > o_threshold
            output_f(ic) = 1/tf(i);
            break
        end
    end
end
figure;
plot(input_f, output_f,'k');
xlabel('input frequency');
ylabel('output frequency');
return

j = 1;
output = zeros(numel(t),1);
for i=1:length(t)
    if (i==1) || (output(i-1)>o_threshold)
        dt_p = 100*dt;
        y = zeros(numel(tf),1);
        y(mod(int32(tf/dt),int32(dt_p/dt)) == 0) = 1;

        dt_p_rf = 110*dt;
        y_rf = zeros(numel(tf),1);
        y_rf(mod(int32(tf/dt),int32(dt_p_rf/dt)) == 0) = 1;

        y_filter=filter(b,a,y-y_rf);
        output(i)=0;
        j=1;
    end
    output(i)=y_filter(j);
    j=j+1;
end
figure;
plot(t,output,'k');
xlabel('time');
ylabel('potential');











