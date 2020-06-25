% A simple ECM with only state of charge
% z[k+1] = z[k] - (?t/Q)*?[k]*i[k]
% ? = delta
% ? = eta

% Simulation variables
endTime = 60*8;
deltaT = 0.1;
N = 0:deltaT:endTime;

% Parameter variables
eta = ones(1,length(N));
z = zeros(1,length(N));

cc = 80;
i = cc*ones(1,length(N));

capacity = 6.55;
nomVoltage = 3.7;
Q = capacity*nomVoltage*3600;

% Initial value
z(1) = 0.8;

for n = 1:length(N)-1
    z(n+1) = z(n) - (deltaT/Q)*eta(n)*i(n);
end

plot(N,z);