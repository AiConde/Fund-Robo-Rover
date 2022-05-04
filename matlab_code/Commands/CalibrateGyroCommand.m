classdef CalibrateGyroCommand < Command


    properties (Access = private)
        calib_time; % How long we want to collect calibration data
        start_time; % Time that we start running the command
        gyro_readings_xyz; % Stored gyro readings
    end

    methods
        function obj = CalibrateGyroCommand(rover_handle, calib_time)

            obj@Command(rover_handle); % Pass rover_handle to the superclass Command constructor

            if (nargin == 1) % If we don't get calib_time specifically passed in
                obj.calib_time = 5; % Default 5 seconds
            else % If we get calib_time passed in
                obj.calib_time = calib_time;
            end
        end

        function initialize(obj)
            disp("Running CalibrateGyroCommand");
            obj.start_time = obj.rover_handle.system_time; % We're starting the command running at the current system time
            obj.gyro_readings_xyz = []; % Blank buffer
        end

        function execute(obj)
            [accel_xyz, gyro_xyz] = obj.rover_handle.arduino.get_imu_output(); % Get arduino IMU readings
            obj.gyro_readings_xyz = [obj.gyro_readings_xyz ; gyro_xyz]; % Add to buffer
        end

        function done = is_done(obj)
            done = obj.rover_handle.system_time > obj.start_time + obj.calib_time; % Has at least calib_time seconds elapsed since start_time?
        end

        function cmd_end(obj)
            disp("Done with CalibrateGyroCommand");
            gyro_xyz_mean = mean(obj.gyro_readings_xyz); % Take average of readings
            disp(["Found offsets: ", gyro_xyz_mean]);
            obj.rover_handle.arduino.set_gyro_offset(gyro_xyz_mean); % Write offset to arduino class
        end

    end
end