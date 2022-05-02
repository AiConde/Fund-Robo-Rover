classdef Rover < handle
    % Rover class object - represents a single rover.

    properties
        %% class properties

        % LOCALIZATION
        initial_robot_pose % [x, y, theta] 2D pose in world coordinates
        localization         % will return pose 2D + velcoities
        odometry           % odometer obj
        found_dock         % boolean of having located dock
        dock_pose          % dock pose in world coordinates

        % SYSTEM STATE
        system_status % flags to detemine system status e.g. wheels_stopped
        system_time   % current time
        rate_control % Rate control object

        % SENSE
        lidar     % lidar obj
        cam       % camera obj
        arduino   % arduino obj
        gps       % gps obj
        joystick  % joystick obj

        % THINK
        command_list % List of commands to run
        command_idx % Current command index

        % ACT
        drivetrain_controller

    end % End class properties

    methods
        %% class methods

        %% Class constructor
        function obj = Rover(has_joystick)
            % set localization properties
            obj.initial_robot_pose = Pose2d(); % starts off as unknown
            obj.localization = LocalizationEngine();
            obj.odometry = Odometry();
            obj.found_dock = false;
            obj.dock_pose = Pose2d(); % starts off as unknown

            % Set system state
            obj.system_status = 0; % TODO
            obj.system_time = Utils.get_current_time();
            obj.rate_control = rateControl(50);

            % load camera calibration properties
            calib_properties = load("camera_calibration/calib_1024x768/cameraParams.mat");
            cam_intrinsics = calib_properties.cameraParams.Intrinsics;

            % set SENSE properties
            obj.lidar = Lidar_ROS();
            obj.cam = Camera_ROS(cam_intrinsics);
            obj.arduino = Arduino_ROS();
            obj.gps = GPS_ROS();
            if has_joystick
                % TODO                obj.joystick =
            end
        end

        % Waits for all ROS objects to come online
        function wait_for_ros_init(obj)
            disp("Waiting for arduino to come online...");
            while ~(...
                    obj.arduino.is_new_ir_data_available() && ...
                    obj.arduino.is_new_sonar_data_available() && ...
                    obj.arduino.is_new_imu_data_available() && ...
                    obj.arduino.is_new_mag_data_available() && ...
                    obj.arduino.is_new_tacometer_data_available())
                pause(0.1); % Spinlock
            end
            disp("Done!");
            disp("Waiting for LIDAR to come online...");
            while ~(obj.lidar.is_new_laserscan_available())
                pause(0.1);
            end
            disp("Done!");
            disp("Waiting for GPS to come online...");
            while ~(obj.gps.is_new_fix_data_available() && obj.gps.is_new_imu_data_available())
                pause(0.1);
            end
            disp("Done!");
            disp("Waiting for camera to come online...");
            while ~(obj.cam.is_new_image_available())
                pause(0.1);
            end
            disp("Done!");
        end

        % Call this as fast as possible from the external run function
        function main_loop(obj)
            obj.update_time();
            obj.update_odometry();

            
        end

        function update_time(obj)
            obj.system_time = Utils.get_current_time();
        end

        function update_odometry(obj)
            [accel_xyz, gyro_xyz] = obj.arduino.get_imu_output_calibrated();
            taco_val = obj.arduino.get_tacometer_output();
            obj.odometry.update_imu(taco_val, gyro_xyz, obj.system_time);
            obj.localization.write_odom_pose(obj.odometry.odom_pose, obj.system_time);
            obj.localization.write_odom_twist(obj.odometry.odom_twist, obj.system_time);
        end

        function write_localization(obj, pose)
            obj.localization.register_localization_observation(pose);
        end

        % Called when we're done with all commands
        function mission_end(obj)
        end

        % TODO figure out what we're doing with this 
        function mission_error(obj)
        end

        %% Class deconstructor
        function delete(obj)
            % set servos and esc to
            obj.arduino.write_esc_pwm(0.5);
            obj.arduino.write_steer_servo(0.5);
            obj.arduino.write_pan_servo(0.5);
        end

        %%
        function command_fail(obj)
            % this function gets called whenever a command fails
        end

    end % End classmethods

end % End class

