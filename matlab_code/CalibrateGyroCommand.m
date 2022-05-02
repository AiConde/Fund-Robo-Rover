classdef CalibrateGyroCommand < Command

    properties (Constant)
        calib_time = 5;
    end

    properties (Access = private)
        start_time;
    end

    methods
        function initialize(obj, rover)
            obj.start_time = Utils.get_current_time();
        end

        function execute(obj, rover)
        end

        function done = is_done(obj, rover)
            done = (Utils.get_current_time() - obj.start_time) > obj.calib_time;
        end

        function cmd_end(obj, rover)
            rover.arduino.gyro_calib = calculated_calibration;
        end

    end

end