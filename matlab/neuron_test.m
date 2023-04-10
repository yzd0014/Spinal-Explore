clear;
K = 0.3;
dt = 0.001;
tspan = 0:dt:5;
sz = numel(tspan);
P1 = zeros(sz,1);
P2 = zeros(sz,1);
w = 0.9;

x_old = 1;
y_old = 0;
for i=1:numel(tspan)
    syms x y
    [Sx, Sy] = vpasolve([x == (x_old*(1-K)+(y*-w+w)*K)/(0.5+x_old*(1-K)+(y*w+w)*K), y == (y_old*(1-K)+(x*-w+w)*K)/(0.5+y_old*(1-K)+(x*w+w)*K)],[x,y]);
    sx = Sx(1);
end

%% 
clear;

Ka = 1;
dt = 0.01;
tspan = 0:dt:5;
sz = numel(tspan);

mode = 2;
excit_input = zeros(sz,1);
if mode == 1
    excit_input = 0.5*cos(tspan*10)+0.5;
elseif mode == 2
    excit_input = ones(sz,1)*0.5;
end

p1 = zeros(sz,1);
p1(1) = 0.5;
p2 = zeros(sz,1);
p2(1) = 0;

w_exc = 1;
w_inh = 0.9;
ws = [w_exc, w_inh];
signs = [1, -1];

filter1_top = signs(1)*ws(1)*excit_input(1)+p2(1)*ws(2)*signs(2);
filter1_bottom = excit_input(1)*ws(1)+p2(1)*ws(2);
% filter2_top = signs(1)*ws(1)*excit_input(1)+p1(1)*ws(2)*signs(2);
% % filter2_bottom = excit_input(1)*ws(1)+p1(2)*ws(2);
% filter1_top = 0;
% filter1_bottom = 0;
filter2_top = 0;
filter2_bottom = 0;

for ti=1:sz-1
    inputs_1 = [excit_input(ti), p2(ti)];
    [p1(ti+1),filter1_top,filter1_bottom] = ComputePotential(inputs_1,ws,signs,filter1_top,filter1_bottom,Ka);

    inputs_2 = [excit_input(ti), p1(ti)];
    [p2(ti+1),filter2_top,filter2_bottom] = ComputePotential(inputs_2,ws,signs,filter2_top,filter2_bottom,Ka);
end
subplot(3,1,1);
plot(tspan,p1);
subplot(3,1,2);
plot(tspan,p2);
subplot(3,1,3);
plot(tspan,excit_input);

function [output,T1,T2] = ComputePotential(potentials,ws,signs,f_top,f_bottom,K)
    n = numel(potentials);
    sum = 0;
    sum_positive = 0;
    for i=1:n
        if potentials(i) > 0
           sum = sum + potentials(i)*ws(i)*signs(i);
           sum_positive = sum_positive + potentials(i)*ws(i);
        end
    end
    T1 = f_top*(1-K)+sum*K;
    T2 = f_bottom*(1-K)+sum_positive*K;
    output = T1/(0.5+T2);
end


