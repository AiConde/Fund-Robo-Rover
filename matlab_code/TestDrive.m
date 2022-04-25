clear;
clc;

disp("Setting up ROS objects...");

arduino = Arduino_ROS();
lidar = Lidar_ROS();
gps = GPS_ROS();
cam = Camera_ROS();

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
    if (joystick.read_trigger())
        run_loop = false;
        break;
    end

    throttle_desired = joystick.read_throttle();
    steer_desired = joystick.read_steer();
    pan_desired = joystick.read_twist();

    throttle_servo_units = Utils.map(throttle_desired, -1, 1, 0, 1);
    steer_servo_units = Utils.map(steer_desired, -1, 1, 0, 1);
    pan_servo_units = Utils.map(pan_desired, -1, 1, 0, 1);

    arduino.write_esc_pwm(throttle_servo_units);
    arduino.write_steer_servo(steer_servo_units);
    arduino.write_pan_servo(pan_servo_units);

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
