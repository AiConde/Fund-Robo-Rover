classdef PanStage < Servo
    % Pan servo control object. Represents the pan stage servo/angle.

    methods (Access = private)
        %% Private classmethods
        function servo_angle = pan_angle_to_servo_angle(pan_angle)
            % TODO function mapping between pan angle and servo PWM
        end
    end
    
    properties (Access = private)
        %% Private properties
        pan_angle
    end
    
    methods
        %% class methods

        %% Class constructor
        function obj = PanStage(arduino_object, arduino_pin)
            obj@Servo(arduino_object, arduino_pin);
        end

        %% Write desired steering angle to servo
        function write_pan_angle(obj, pan_angle)
            servo_angle = pan_angle_to_servo_angle(pan_angle);
            writePosition(obj.servo_obj, servo_angle);
            obj.pan_angle = pan_angle;
        end

        function pan_angle = read_pan_angle(obj)
            pan_angle = obj.pan_angle;
        end

    end % End classmethods

end % End class

