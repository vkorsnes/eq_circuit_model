% A still simple ECM with only state of charge
% This time we will add an equivalent series resistance
% z[k+1] = z[k] - (?t/Q)*?[k]*i[k]
% v[k] = OCV(z[k]) - i[k]*R0
% ? = delta
% ? = eta

% Simulation variables
endTime = 60*8;
deltaT = 0.1;
N = 0:deltaT:endTime;

% Parameter variables
eta = ones(1,length(N));
z = zeros(1,length(N));
v = zeros(1,length(N));
R0 = 0.003;

cc = 80;
i = ones(1,length(N));
i(500:3000) = cc;

capacity = 6.55;
nomVoltage = 3.7;
Q = capacity*nomVoltage*3600;

% Initial value
z(1) = 0.8;

for n = 1:length(N)-1
    z(n+1) = z(n) - (deltaT/Q)*eta(n)*i(n);
    v(n) = OCV_from_SOC(z(n)) - i(n)*R0;
end

% clean up last value in v
v(length(v)) = v(length(v)-1);

subplot(2,1,1)
plot(N,z)
title('SOC')
subplot(2,1,2)
plot(N,v)
title('v(t)')
