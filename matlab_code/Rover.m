classdef Rover < handle
    % Rover class object - represents a single rover.

    properties
        %% class properties

        % LOCALIZATION
        initial_robot_pose % [x, y, theta] 2D pose in world coordinates
        localization         % will return pose 2D + velcoities
        odometry           % odometer obj
        localized_starting_pose % Estimate of where we started, from initial tag localization
        found_dock         % boolean of having located dock
        dock_pose          % dock pose in world coordinates

        % SYSTEM STATE
        system_status % flags to determine system status e.g. wheels_stopped
        system_time   % current time
        rate_control % Rate control object
        mission_active % Whether or not we're current running/executing commands

        % SENSE
        lidar     % lidar obj
        cam       % camera obj
        arduino   % arduino obj
        gps       % gps obj
        joystick  % joystick obj

        % THINK
        command_list % List of commands to run
        command_idx % Current command index
        current_command % Current running command
        new_command % Are we about to run a new command for the first time?

        % ACT
        drivetrain_controller % Closed loop controller for drivetrain

    end % End class properties

    methods
        
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
            obj.mission_active = true;

            % load camera calibration properties
            calib_properties = load("camera_calibration/calib_1024x768/cameraParams.mat");
            cam_intrinsics = calib_properties.cameraParams.Intrinsics;

            % set SENSE properties
            disp("Setting up ROS objects...");
            obj.lidar = Lidar_ROS();
            obj.cam = Camera_ROS(cam_intrinsics);
            obj.arduino = Arduino_ROS();
            obj.gps = GPS_ROS();
            if has_joystick
                % TODO                obj.joystick =
            end

            % set THINK properties
            obj.command_idx = 1;
            obj.new_command = true;

            % set ACT properties
            obj.drivetrain_controller = DrivetrainController();
        end

        %% class methods

        function set_mission_command_list(obj, cmd_list)
            obj.command_list = cmd_list;
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

        % Inits the robot
        function robot_init(obj)
            % Inits all the robot command execution stuff
            obj.command_idx = 1;
            obj.current_command = obj.command_list(obj.command_idx);
            obj.new_command = true;
            %Register start position of robot once for reference:
            obj.update_time(); % Update system time
            obj.update_odometry(); % Update odometry and localization engine
            obj.localized_starting_pose = obj.odometry.odom_pose; %Set that
        end

        % Call this as fast as possible from the external run function
        function main_loop(obj)
            obj.update_time(); % Update system time
            obj.update_odometry(); % Update odometry and localization engine

            if (obj.new_command) % We need to initialize the current command
                obj.current_command.initialize();
                obj.new_command = false;
            end

            obj.current_command.execute(); % Call execute function of current command

            if (obj.current_command.is_done()) % Check if current command is done
                obj.current_command.cmd_end(); % Call current command end function

                if (obj.command_idx == size(obj.command_list))
                    obj.mission_end(); % If we've called all the commands in our mission file, call the mission end function
                else
                    % We still have more commands to run! Increment to the next one.
                    obj.command_idx = obj.command_idx + 1;
                    obj.current_command = obj.command_list(obj.command_idx); % Set obj.current_command to the next command in the list
                    obj.new_command = true; % Indicate to us that we have to run initialize() on the command next loop
                end
            end

            obj.update_drivetrain_controller(); % Update drivetrain closed loop control


            waitfor(obj.rate_control);
        end

        function update_time(obj)
            obj.system_time = Utils.get_current_time();
        end

        function update_odometry(obj)
            [~, gyro_xyz] = obj.arduino.get_imu_output_calibrated();
            taco_val = obj.arduino.get_tacometer_output();
            obj.odometry.update_imu(taco_val, gyro_xyz(3), obj.system_time);
            obj.localization.write_odom_pose(obj.odometry.odom_pose, obj.system_time);
            obj.localization.write_odom_twist(obj.odometry.odom_twist, obj.system_time);
        end

        function update_drivetrain_controller(obj)
            [~, gyro_xyz] = obj.arduino.get_imu_output_calibrated();
            taco_val = obj.arduino.get_tacometer_output();

            obj.drivetrain_controller.update(taco_val, gyro_xyz(3), obj.system_time);

            [esc_pwm, steer_pwm] = obj.drivetrain_controller.get_pwm_outputs();
            obj.arduino.write_esc_pwm(esc_pwm);
            obj.arduino.write_steer_servo(steer_pwm);
        end

        function write_localization(obj, pose)
            obj.localization.register_localization_observation(pose);
        end

        % Called when we're done with all commands
        function mission_end(obj)
            obj.mission_active = false;
            obj.drivetrain_controller.set_vel_setpoints(0,0);
        end

        % TODO figure out what we're doing with this
        function mission_error(obj)
            obj.mission_active = false;
        end

        %% Class deconstructor
        function delete(obj) %Delete is a reserved keyword and you can type "clear INSTANCENAME" to run it :B
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

