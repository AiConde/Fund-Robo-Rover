classdef SteeringServo < Servo
    % Steering servo class object. Represents the steering servo on the
    % rover.

    methods (Access = private)
        %% Private classmethods
        function servo_angle = steer_angle_to_servo_angle(steer_angle)
            % TODO function mapping between steer angle and servo PWM
        end
    end
    
    properties (Access = private)
        %% Private properties
        steer_angle
    end
    
    methods
        %% class methods

        %% Class constructor
        function obj = SteeringServo(arduino_object, arduino_pin)
            obj@Servo(arduino_object, arduino_pin);
        end

        %% Write desired steering angle to servo
        function write_steer_angle(obj, steer_angle)
            servo_angle = steer_angle_to_servo_angle(steer_angle);
            writePosition(obj.servo_obj, servo_angle);
            obj.steer_angle = steer_angle;
        end

        function angle = read_steer_angle(obj)
            angle = obj.steer_angle;
        end

    end % End classmethods

end % End class

