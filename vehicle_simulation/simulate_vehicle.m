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

aero_f = zeros(1, length(timestamp));
rolling_f = rolling_friction*total_mass*g;


for n = 1:length(timestamp)-1
    prev_motor_speed = 0;
    
    deltaT = abs(timestamp(n)-timestamp(n+1));
    
    des_acc(n) = (des_speed(n) - act_speed(n))/deltaT;
    des_acc_f(n) = equivalent_mass*des_acc(n);
    
    aero_f(n+1) = air_density*front_area*drag_coef*act_speed(n)^2;
    
    dem_mot_torque(n) = (des_acc_f(n) + aero_f(n) + rolling_f)*wheel_radius/gear_ratio;
    
    if prev_motor_speed < rated_RPM
        available_torque = max_torque;
    else
        available_torque = max_torque*(rated_RPM/prev_motor_speed);
    end
    
    
    
    
end
