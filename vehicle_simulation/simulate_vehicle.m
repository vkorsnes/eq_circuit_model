clear;clc;
% Load constants
constants
% Load velocity cycle
velocity_load_FSG_endu

% Setup simulation
des_speed = INS_vx;
act_speed = zeros(1, length(timestamp));
des_acc = zeros(1, length(timestamp));
des_acc_f = zeros(1, length(timestamp));
dem_mot_torque = zeros(1, length(timestamp));
available_torque = zeros(1, length(timestamp));
act_acc_f = zeros(1, length(timestamp));
lim_torque = zeros(1, length(timestamp));
aero_f = zeros(1, length(timestamp));
rolling_f = rolling_friction*total_mass*g;
act_acc = zeros(1, length(timestamp));
motor_speed = zeros(1, length(timestamp));


for n = 1:length(timestamp)-1
    prev_motor_speed = 0;
    
    deltaT = abs(timestamp(n)-timestamp(n+1));
    
    des_acc(n) = (des_speed(n) - act_speed(n))/deltaT;
    des_acc_f(n) = equivalent_mass*des_acc(n);
    
    aero_f(n+1) = 0.5*air_density*front_area*drag_coef*act_speed(n)^2;
    
    dem_mot_torque(n) = (des_acc_f(n) + aero_f(n) + rolling_f)*wheel_radius/gear_ratio;
    
    if prev_motor_speed < rated_RPM
        available_torque(n) = max_torque;
    else
        available_torque(n) = max_torque*(rated_RPM/prev_motor_speed);
    end
    
    lim_torque(n) = min(available_torque(n), max_torque);
    
    act_acc_f(n) = lim_torque(n)*gear_ratio/wheel_radius - ...
        aero_f(n) - rolling_f;
    
    act_acc(n) = act_acc_f(n)/equivalent_mass;
    motor_speed(n) = min(max_RPM, gear_ratio * (prev_motor_speed + act_acc(n) * ...
        deltaT) * 60 / (2*pi*wheel_radius));
    act_speed(n) = motor_speed(n)*2*pi*wheel_radius/(60*gear_ratio);
    
end
