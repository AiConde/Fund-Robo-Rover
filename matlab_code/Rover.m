classdef Rover < handle
    % Rover class object - represents a single rover.
    
    properties
        %% class properties

        % LOCALIZATION
        initial_robot_pose % [x, y, theta] 2D pose in world coordinates 
        loc_engine         % will return pose 2D + velcoities
        odometry           % odometer obj
        found_dock         % boolean of having located dock
        dock_pose          % dock pose in world coordinates

        % SYSTEM STATE
        system_status % flags to detemine system status e.g. wheels_stopped
        system_time   % current time

        % SENSE
        lidar     % lidar obj
        cam       % camera obj
        arduino   % arduino obj
        gps       % gps obj
        joystick  % joystick obj

        % ACT
% TODO        drivetrain_controller % takes

    end % End class properties
    
    methods
        %% class methods

        %% Class constructor
        function obj = Rover(has_joystick)
            % set localization properties
            obj.initial_robot_pose = Pose2d(); % starts off as unknown
            obj.loc_engine = LocalizationEngine();
            obj.odometry = Odometry(start_pose);
            obj.found_dock = false;
            obj.dock_pose = Pose2d(); % starts off as unknown

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

        %% Class deconstructor
        function delete(obj)
            % set servos and esc to 
            obj.arduino.write_esc_pwm(0.5);
            obj.arduino.write_steer_servo(0.5);
            obj.arduino.write_pan_servo(0.5);
        end

    end % End classmethods

end % End class

