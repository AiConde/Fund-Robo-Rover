classdef ESCThrottle < Servo
    % ESC throttle control object. Represents the PWM control to the ESC
    % rover.

    methods (Access = private)
        %% Private classmethods
        function servo_angle = throttle_to_servo_angle(throttle)
            % TODO function mapping between throttle units and servo PWM
        end
    end
    
    properties (Access = private)
        %% Private properties
        throttle
    end
    
    methods
        %% class methods

        %% Class constructor
        function obj = ESCThrottle(arduino_object, arduino_pin)
            obj@Servo(arduino_object, arduino_pin);
        end

        %% Write desired steering angle to servo
        function write_throttle(obj, throttle)
            servo_angle = throttle_to_servo_angle(throttle);
            writePosition(obj.servo_obj, servo_angle);
            obj.throttle = throttle;
        end

        function throttle = read_throttle(obj)
            throttle = obj.throttle;
        end

    end % End classmethods

end % End class

