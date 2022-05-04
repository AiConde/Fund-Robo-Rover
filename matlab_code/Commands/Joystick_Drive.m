classdef Joystick_Drive < Command
    
    properties (Access = private)
    end

    methods
        function obj = Joystick_Drive(rover_handle, calib_time)

            obj@Command(rover_handle); % Pass rover_handle to the superclass Command constructor

            if (nargin == 1) % If we don't get calib_time specifically passed in
                obj.calib_time = 5; % Default 5 seconds
            else % If we get calib_time passed in
                obj.calib_time = calib_time;
            end
        end

        function initialize(obj)
           %nothing
        end

        function execute(obj)
           throttle = rescale(obj.rover_handle.joystick.throttle, 0, 10, 'InputMin',0);
           %or is it obj.throttle_raw = obj.rover_handle.joystick.throttle; % Scaled joystick throttle input
           steer = (obj.rover_handle.joystick.steer).*2; % Scaled joystick throttle input
           obj.rover_handle.drivetrain_controller.set_vel_setpoints(joystick, throttle, steer);
        end

        function done = is_done(obj)
            done = obj.rover_handle.joystick.read_trigger(); % e-stop is trigger
        end

        function cmd_end(obj)
            obj.rover_handle.joystick.set_vel_setpoints(joystick, 0, 0);

        end

    end
end