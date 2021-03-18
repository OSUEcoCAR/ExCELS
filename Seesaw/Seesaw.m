clear; close all; clc

%% Inputs -- PLEASE MODIFY

servo_offset = 1760;
range = 300;

setpoint = 9; % [cm]

Kp = 0.7;
Ki = 0.2;
Kd = 0.2;

% Start Position
sensor = 6; % [cm]
servo = 1760; % theta = 0 @ 1760

%% Simulate  -- DO NOT MODIFY

servo_min = servo_offset-range;
servo_max = servo_offset+range;

% Initialize
i = 0;
dt = 0.001;
time = 0;
vel = 0;
accel = 0;
integral = 0;
previous_error = 0;
scale = 10;

while time < 20
    time = time + dt;
    i = i + 1;
    
    % Plant Model (Error) -- VERIFY AGAINST ACTUAL
    theta = (servo-servo_offset)/300*20; % 20 deg per 300 ms on servo
    accel = 9.81*100*sind(theta);
    vel = accel*dt + vel;
    sensor(i+1) = sensor(i) + vel*dt;
    error = setpoint - sensor(i);

    % PID Controller
    integral = integral + error*dt;
    derivative = (error - previous_error)/dt;
    previous_error = error;
    output = (Kp*error + Ki*integral + Kd*derivative)*scale;
    servo = output + servo_offset;
    if servo>servo_max
        servo = servo_max;
    elseif servo <servo_min
        servo = servo_min;
    end
end

plot ([1:i]/1000,sensor(1:end-1));
xlabel('Time [s]');
ylabel('Position [cm]');