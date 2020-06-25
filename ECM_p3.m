% A more advance ECM with state of charge
% This time we will add an equivalent series resistance
% And diffusion voltage
% z[k+1] = z[k] - (?t/Q)*?[k]*i[k]
% ir1[k+1] = exp(-?t/(R1*C1))*ir1[k] + (1-exp(-?t/(R1*C1))*i[k]
% v[k] = OCV(z[k]) - i[k]*R0
% ? = delta
% ? = eta

% Simulation variables
endTime = 60*50;
deltaT = 0.1;
N = 0:deltaT:endTime;

% Parameter variables
eta = ones(1,length(N));
z = zeros(1,length(N));
v = zeros(1,length(N));
ir1 = zeros(1,length(N));
R0 = 0.0082;
R1 = 0.0158;
C1 = 19000;

cc = 40;
i = zeros(1,length(N));
i(2000:10000) = cc;
i(16000:20000) = -cc;
capacity = 6.55;
nomVoltage = 3.7;
Q = capacity*nomVoltage*3600;

% Initial value
z(1) = 0.8;

for n = 1:length(N)-1
    z(n+1) = z(n) - (deltaT/Q)*eta(n)*i(n);
    a = deltaT/(R1*C1);
    ir1(n+1) = exp(-a)*ir1(n) + (1-exp(-a))*i(n);
    v(n) = OCV_from_SOC(z(n)) - R1*ir1(n) - i(n)*R0;
end

% clean up last value in v
v(length(v)) = v(length(v)-1);

subplot(2,1,1)
plot(N,z)
title('SOC')
subplot(2,1,2)
plot(N,v)
title('v(t)')
