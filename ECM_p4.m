% Enhanced self-correcting cell model

% Simulation variables
endTime = 60*50;
deltaT = 0.1;
N = 0:deltaT:endTime;

% Parameter variables
eta = ones(1,length(N));
gamma = 90;
% z[k]
% ir[k]
% h[k]
states = zeros(4,length(N));
v = zeros(1,length(N));
R0 = 0.0083;
R1 = 0.0158;
C1 = 10000;
F1 = exp(-deltaT/(R1*C1));
Brc = (1-F1);

cc = 40;
i = zeros(1,length(N));
i(2000:10000) = cc;
i(16000:20000) = -cc;
capacity = 6.55;
nomVoltage = 3.7;
Q = capacity*nomVoltage*3600;

M0 = 0.05;
M = 0.5;

% Initial value
states(1,1) = 0.8;

for n = 1:length(N)-1
    states(1,n+1) = states(1,n) - (deltaT/Q)*eta(n)*i(n);
    states(2,n+1) = F1*states(2,n) + Brc*i(n);
    states(3,n+1) = exp(-abs(eta(n)*i(n)*gamma*deltaT/Q)) + ...
        (exp(-abs(eta(n)*i(n)*gamma*deltaT/Q)) - 1)*sign(i(n));
    if abs(i(n)) > 0
        states(4,n+1) = sign(i(n));
    else
        states(4,n+1) = states(4,n);
    end
    v(n) = OCV_from_SOC(states(1,n)) + M0*states(4,n) + M*states(3,n) - ...
        R1*states(2,n) - R0*i(n);
end

% clean up last value in v
v(length(v)) = v(length(v)-1);

subplot(2,1,1)
plot(N,states(1,:))
title('SOC')
subplot(2,1,2)
plot(N,v)
title('v(t)')
