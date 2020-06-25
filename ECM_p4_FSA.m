% Enhanced self-correcting cell model

% Simulation variables
current_load_FSA_endu
endTime = 60*50;
deltaT = 0.015;
N = Time;

% Parameter variables
eta = ones(1,length(N));
gamma = 90;
% z[k]
% ir[k]
% h[k]
states = zeros(4,length(N));
v = zeros(1,length(N));
R0 = 0.003;
R1 = 0.004;
C1 = 8000;
F1 = exp(-deltaT/(R1*C1));
Brc = (1-F1);

i = Current/2;
capacity = 6.55;
nomVoltage = 3.7;
Q = capacity*3600;

M0 = 0.025;
M = 0.05;

% Initial value
states(1,1) = 0.93;

for n = 1:length(N)-1
    deltaT = abs(Time(n)-Time(n+1))*1.0;
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
grid on
subplot(2,1,2)
plot(N,v)
title('v(t)')
grid on

figure(2)
plot(Time,v)
hold on
plot(Time, Voltage/144 + 0.106);
grid on
legend('Sim', 'Actual')

