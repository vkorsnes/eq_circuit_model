% set up motor: Lmax [Nm], (RPMrated, RPMmax) [RPM], efficiency,
% inertia [kg/m2]
motor = setupMotor(116,13500,20000,.95,0.0019);
% set up wheel: radius [m], inertia [kg/m2], rollCoef
wheel = setupWheel(0.228,2,0.0111);
% set up drivetrain: inverter efficiency, fractional regen torque limit,
% gear ratio, gear inertia [kg/m2], gear efficiency, for this pack,
% motor, and wheel 
drivetrain = setupDrivetrain(0.94,0.9,14.4,0.05,0.97,motor,wheel);
% set up vehicle: # wheels, roadForce [N], Cd, frontal area [m2], weight
% [kg], payload [kg], overhead power [W] for this drivetrain
vehicle = setupVehicle(4,0,1.5,1.17,162.5,75,150,drivetrain);
rho = 1.225;

velocity_load_FSG_endu
desSpeed = INS_vx;
% pre-allocate storage for results
desAccel = zeros(size(desSpeed)); % m/s2
desAccelForce = zeros(size(desSpeed)); % N
aeroForce = zeros(size(desSpeed)); % N
rollGradeForce = zeros(size(desSpeed)); % N
demandTorque = zeros(size(desSpeed)); % N-m
maxTorque = zeros(size(desSpeed)); % N-m
limitRegen = zeros(size(desSpeed)); % N-m
limitTorque = zeros(size(desSpeed)); % N-m
motorTorque = zeros(size(desSpeed)); % N-m
demandPower = zeros(size(desSpeed)); % kW
limitPower = zeros(size(desSpeed)); % kW
batteryDemand = zeros(size(desSpeed)); % kW
current = zeros(size(desSpeed)); % A
batterySOC = zeros(size(desSpeed)); % 0..100
actualAccelForce = zeros(size(desSpeed)); % N
actualAccel = zeros(size(desSpeed)); % m/s2
motorSpeed = zeros(size(desSpeed)); % RPM
actualSpeed = zeros(size(desSpeed)); % m/s
actualSpeedKPH = zeros(size(desSpeed)); % km/h
distance = zeros(size(desSpeed)); % km

fprintf('\n\nStarting sims...\n');
prevSpeed = 0; prevMotorSpeed = 0; 
prevDistance = 0;
for k = 1:length(desSpeed) - 1
    deltaT = desSpeed(k) - desSpeed(k+1);
    desAccel(k) = (desSpeed(k) - prevSpeed)/deltaT;
    desAccelForce(k) = vehicle.equivMass * desAccel(k);
    aeroForce(k) = 0.5*rho * vehicle.Cd * vehicle.A * prevSpeed^2;

    demandTorque(k) = (desAccelForce(k) + aeroForce(k) + ...
        rollGradeForce(k) + vehicle.roadForce) * ...
        vehicle.drivetrain.wheel.radius / vehicle.drivetrain.gearRatio;
    if prevMotorSpeed < vehicle.drivetrain.motor.RPMrated
      maxTorque(k) = vehicle.drivetrain.motor.Lmax;
    else
      maxTorque(k) = vehicle.drivetrain.motor.Lmax * ...
          vehicle.drivetrain.motor.RPMrated / prevMotorSpeed;
    end
    limitRegen(k) = min(maxTorque(k),...
        vehicle.drivetrain.regenTorque  * vehicle.drivetrain.motor.Lmax);
    limitTorque(k) = min(demandTorque(k),maxTorque(k));
    if limitTorque(k) > 0
      motorTorque(k) = limitTorque(k);
    else
      motorTorque(k) = max(-limitRegen(k),limitTorque(k));
    end

    actualAccelForce(k) = limitTorque(k) * vehicle.drivetrain.gearRatio / ...
        vehicle.drivetrain.wheel.radius - aeroForce(k) - rollGradeForce(k) - ...
        vehicle.roadForce;
    actualAccel(k) = actualAccelForce(k) / vehicle.equivMass;
    motorSpeed(k) = min(vehicle.drivetrain.motor.RPMmax,...
        vehicle.drivetrain.gearRatio * (prevSpeed + actualAccel(k) * ...
        deltaT) * 60 / (2*pi*vehicle.drivetrain.wheel.radius));
    actualSpeed(k) = motorSpeed(k) * 2*pi*vehicle.drivetrain.wheel.radius / ...
        (60 * vehicle.drivetrain.gearRatio);
    actualSpeedKPH(k) = actualSpeed(k) * 3600/1000;
    deltadistance = (actualSpeed(k) + prevSpeed)/2 * ...
                    deltaT/1000;
    distance(k) = prevDistance + deltadistance;

    if limitTorque(k) > 0
      demandPower(k) = limitTorque(k);
    else
      demandPower(k) = max(limitTorque(k),-limitRegen(k));
    end
    demandPower(k) = demandPower(k) * 2*pi * ...
                          (prevMotorSpeed + motorSpeed(k))/2/60000;
    limitPower(k) = max(-vehicle.drivetrain.motor.maxPower,min(...
        vehicle.drivetrain.motor.maxPower,demandPower(k)));
    batteryDemand(k) = vehicle.overheadPwr/1000;
    if limitPower(k) > 0,
      batteryDemand(k) = batteryDemand(k) + ...
            limitPower(k)/vehicle.drivetrain.efficiency;
    else
      batteryDemand(k) = batteryDemand(k) + ...
            limitPower(k)*vehicle.drivetrain.efficiency;
    end


    prevSpeed = actualSpeed(k);
    prevMotorSpeed = motorSpeed(k);
    prevSOC = batterySOC(k);
    prevDistance = distance(k);
end

% Create a data structure to store motor-specific variables
function motor = setupMotor(Lmax,RPMrated,RPMmax,efficiency,inertia)
motor.Lmax = Lmax; % N-m
motor.RPMrated = RPMrated;
motor.RPMmax = RPMmax;
motor.efficiency = efficiency;
motor.inertia = inertia; %kg-m2
motor.maxPower = 2*pi*Lmax*RPMrated/60000; % kW
end

% Create a data structure to store wheel-specific variables
function wheel = setupWheel(radius,inertia,rollCoef)
wheel.radius = radius; % m
wheel.inertia = inertia; % km-m2
wheel.rollCoef = rollCoef;
end

% Create a data structure to store drivetrain-specific variables
function drivetrain = setupDrivetrain(inverterEfficiency,regenTorque,...
    gearRatio,gearInertia,gearEfficiency,motor,wheel)
drivetrain.inverterEfficiency = inverterEfficiency;
drivetrain.regenTorque = regenTorque; % fraction of total torque avail.
drivetrain.motor = motor;
drivetrain.wheel = wheel;
drivetrain.gearRatio = gearRatio;
drivetrain.gearInertia = gearInertia; % km-m2, measured on motor side
drivetrain.gearEfficiency = gearEfficiency;
drivetrain.efficiency = inverterEfficiency * ...
                        motor.efficiency * gearEfficiency;
end

% Create a data structure to store all vehicle specifications
function vehicle = setupVehicle(wheels,roadForce,Cd,A,weight,payload,...
                              overheadPwr,drivetrain)
vehicle.drivetrain = drivetrain;
vehicle.wheels = wheels; % number of them
vehicle.roadForce = roadForce; % N
vehicle.Cd = Cd; % drag coeff
vehicle.A = A; % frontal area, m2
vehicle.weight = weight; % kg
vehicle.payload = payload; % kg
vehicle.overheadPwr = overheadPwr; % W
vehicle.curbWeight = weight;
vehicle.maxWeight = vehicle.curbWeight + payload;
vehicle.rotWeight = ((drivetrain.motor.inertia + ...
                      drivetrain.gearInertia) * ...
                     drivetrain.gearRatio^2 + ...
                     drivetrain.wheel.inertia*wheels)/...
                     drivetrain.wheel.radius^2;
vehicle.equivMass = vehicle.maxWeight + vehicle.rotWeight;
vehicle.maxSpeed = 2 * pi * drivetrain.wheel.radius * ...
    drivetrain.motor.RPMmax * 60 / (1000*drivetrain.gearRatio); % km/h
end
