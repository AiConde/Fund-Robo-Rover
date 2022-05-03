%% TestDrive.m is the Joystick operation central for Fundamentals of Robotics Spring 2022 Team 1.
%TestDrive.m sets up everything we need for driving with the joystick.
%Use MissionDrive.m for for the sense, think, act loop.


clear;
clc;

%init_ROS();

disp("Setting up ROS objects...");

calib_params = load("camera_calibration/calib_1024x768/cameraParams.mat");
cam_intrinsics = calib_params.cameraParams.Intrinsics;


arduino = Arduino_ROS();
lidar = Lidar_ROS();
gps = GPS_ROS();
%cam = Camera_ROS(cam_intrinsics);

disp("Done!");

pause(1);

disp("Setting up joystick...");

joystick = Joystick(1);

disp("Done!")

pause(1);


disp("Clearing servos...");
arduino.write_esc_pwm(0.5);
arduino.write_steer_servo(0.5);
arduino.write_pan_servo(0.5);

pause(1);
disp("Starting drive routine! Press joystick trigger to stop.");


r = rateControl(50);

run_loop = true;

while (run_loop) 
    if (joystick.read_cluster())
        run_loop = false;
        break;
    end

    throttle_desired = joystick.read_throttle();
    steer_desired = joystick.read_steer();
    pan_desired = joystick.read_twist();

    throttle_servo_units = Utils.map(throttle_desired, -1, 1, 0.6, 0.4);
    steer_servo_units = Utils.map(steer_desired, -1, 1, 1, 0);
    pan_servo_units = Utils.map(pan_desired, -1, 1, 0, 1);

    throttle_cmd = curve_throttle(throttle_servo_units);
    steer_cmd    = curve_steering(steer_servo_units);
    pan_cmd      = curve_pan(pan_servo_units);

%     arduino.write_esc_pwm(throttle_cmd);
    arduino.write_esc_pwm(throttle_servo_units);
    arduino.write_steer_servo(steer_cmd);
    arduino.write_pan_servo(pan_servo_units);

    disp("steer:")
    disp("raw " + string(steer_servo_units) + " | curved: " + string(steer_cmd))
    disp("");disp("");

    disp("throttle:")
    disp("raw " + string(throttle_servo_units) + " | curved: " + string(throttle_cmd))
    disp("");disp("");

%     disp("throttle:")
%     disp("raw " + string(throttle_servo_units) + " | curved: " + string(throttle_cmd))
%     disp("");disp("");

    waitfor(r);
end

disp("Shutting down...");

arduino.write_esc_pwm(0.5);
arduino.write_steer_servo(0.5);
arduino.write_pan_servo(0.5);

pause(1);

clear arduino;
clear lidar;
clear gps;
clear camera;
clear joystick;

function throt_curved = curve_throttle(throt_raw)
    if throt_raw > 0.52
        throt_min = 0.52;
        throt_curved = 200*(throt_raw - throt_min)^3 + throt_min;
    elseif throt_raw < 0.46
        throt_min = 0.46;
        throt_curved = 280*(throt_raw - throt_min)^3 + throt_min;
    else
        throt_curved = 0.5;
    end
end

function steer_curved = curve_steering(steer_raw)
    steer_curved = 4*(steer_raw - 0.5)^3 + 0.5;
end

function pan_curved = curve_pan(pan_raw)
    pan_curved = pan_raw;
end

% min reverse = 0.46
% max reverse = 0.4
