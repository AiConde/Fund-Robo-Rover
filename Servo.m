classdef Servo < handle
    % Servo class object. Represents a single Servo.
    % Designed to be a configurable parent class of other servo objects
    % with different mapping functions.
    
    properties
        %% class properties
        servo_obj
        servo_angle
    end % End class properties

    properties(Constant)
        %% Constant properties
        min_servo_pulse_duration = 10*10^-6
        max_servo_pulse_duration = 1925*10^-6
    end
    
    methods
        %% class methods

        %% Class constructor
        function obj = Servo(arduino_object, arduino_pin)
             obj.servo_obj = ...
                 servo(arduino_object, arduino_pin, ...
                 'MinPulseDuration', min_servo_pulse_duration, ...
                 'MaxPulseDuration', max_servo_pulse_duration);
             writePosition(obj.servo_obj, 0.5); % Initialize to midpoint
             obj.servo_angle = 0.5;
        end

        %% Passthrough write of servo angle
        function write_angle(obj, angle)
            writePosition(obj.servo_obj, angle);
            obj.servo_angle = angle;
        end

    end % End classmethods

end % End class

