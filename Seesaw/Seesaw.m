clear; close all; clc

%% Inputs -- MODIFY MY VALUES!

servo_offset = 1760;
range = 300; % 90 deg per 500 ms on servo

setpoint = 6; % [cm]

Kp = 0.9;
Ki = 0.3;
Kd = 0.2;

% Start Position
sensor = 15; % [cm]
servo = 1760; % theta = 0 @ 1760

% Live Plot? [0/1] = [OFF/ON]
live = 1;

%% Simulate  -- DO NOT MODIFY

servo_min = servo_offset-range;
servo_max = servo_offset+range;

% Initialize
i = 0;
dt = 0.025;
time = 0;
vel = 0;
accel = 0;
integral = 0;
previous_error = 0;
scale = 10;

tic
figure(1)
while time < 5
    time = time + dt;
    i = i + 1;
    
    % Plant Model (Error) -- VERIFY AGAINST ACTUAL
    theta = (servo-servo_offset)/500*90; % 90 deg per 500 ms on servo
    accel = 9.81*100*sind(theta);
    vel = accel*dt + vel;
    sensor(i+1) = sensor(i) + vel*dt;
    error = setpoint - sensor(i+1);

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
    
    if live == 1
        plot(sensor(i)*cosd(theta),sensor(i)*sind(theta),'ro',[-2:18],[-2:18].*tand(theta),'k')
        axis([-2,18,-2,2])
        pause(dt)
    end
end
toc

figure(2)
plot ([1:i]*dt,sensor(1:end-1));
xlabel('Time [s]');
ylabel('Position [cm]');