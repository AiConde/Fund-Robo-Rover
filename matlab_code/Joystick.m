classdef Joystick < handle
    % Connects to and reads from the USB joystick

    properties (Access = private)
        joystick_obj % MATLAB vrjoystick object
    end

    methods
        function obj = Joystick(joystick_idx) 
            if (nargin == 0)
                joystick_obj = vrjoystick(1);
            else
                joystick_obj = vrjoystick(joystick_idx);
            end
        end

        % Returns throttle, -1 to 1
        function throttle = read_throttle()
            throttle = axis(obj.joystick_obj, 0);
        end

        % Returns steer, -1 to 1
        function steer = read_steer()
            steer = axis(obj.joystick_obj, 1);
        end

        % Returns twist, -1 to 1
        function twist = read_twist()
            twist = axis(obj.joystick_obj, 2);
        end

        % Returns trigger, boolean true/false
        function trigger = read_trigger()
            trigger = button(obj.joystick_obj, 0);
        end
    end


end

