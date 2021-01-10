clear;clc;

disp('Loading endurance data from FSA..')
current_load_FSA_endu
disp('Finished loading endurance data from FSA.')
deltaT = 0.0043; % (Time.end-Time.start)/length(Time)
N = length(Time); % 2 NaNs at end of vector

eta = ones(1,N);
i = Current/2; % Simulating single cell
capacity = 6.55;
nomVoltage = 3.7;
Q = capacity*3600;

R1 = 300e-6;
C2 = 1e-7;
R2 = 100e-5;
C3 = 1e-8;
R3 = 10e-5;
C4 = 1e-9;
R4 = 1e-5;

F2 = exp(-deltaT/(R2*C2));
F3 = exp(-deltaT/(R3*C3));
F4 = exp(-deltaT/(R4*C4));

Arc = diag([F2,F3,F4]);
Brc = [1-F2; 1-F3; 1-F4];

gamma = 90;

% z[k]
% ir2[k]
% ir3[k]
% ir4[k]
% h[k]
states = zeros(6,N);

M0 = 0.025;
M = 0.05;

% Initial value
v = zeros(1,N);
states(1,1) = 0.93;

disp('Start simulating..')
for n = 1:N-1
    states(1,n+1) = states(1,n) - (deltaT/Q)*eta(n)*i(n);
    states(2:4,n+1) = Arc*states(2:4,n) + Brc*i(n);
    states(5,n+1) = exp(-abs(eta(n)*i(n)*gamma*deltaT/Q)) + ...
        (exp(-abs(eta(n)*i(n)*gamma*deltaT/Q)) - 1)*sign(i(n)); 
    if abs(i(n)) > 0
        states(6,n+1) = sign(i(n));
    else
        states(6,n+1) = states(6,n);
    end
    v(n) = (OCV_from_SOC(states(1,n)) + M0*states(6,n) + M*states(5,n) - ...
        R2*states(2,n) - R3*states(3,n) - R4*states(4,n) - R1*i(n));    
end
disp('Finished simulating.')

subplot(2,1,1)
plot(Time,states(1,:))
title('SOC')
grid on
subplot(2,1,2)
plot(Time,v')
title('v(t)')
grid on

figure(2)
plot(Time,v')
hold on
plot(Time, Voltage/144 + 0.106);
grid on
legend('Sim', 'Actual')

figure(3)
plot(Time,v'-Voltage/144+0.106)
grid on
