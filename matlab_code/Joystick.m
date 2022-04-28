classdef Joystick < handle
    % Connects to and reads from the USB joystick

    properties
        joystick_obj % MATLAB vrjoystick object
    end

    methods
        function obj = Joystick(joystick_idx) 
            if (nargin == 0)
                obj.joystick_obj = vrjoystick(1);
            else
                obj.joystick_obj = vrjoystick(joystick_idx);
            end
        end

        % Returns throttle, -1 to 1
        function throttle = read_throttle(obj)
            throttle = axis(obj.joystick_obj, 2);
        end

        % Returns steer, -1 to 1
        function steer = read_steer(obj)
            steer = axis(obj.joystick_obj, 1);
        end

        % Returns twist, -1 to 1
        function twist = read_twist(obj)
            twist = axis(obj.joystick_obj, 4);
        end

        % Returns trigger, boolean true/false
        function trigger = read_trigger(obj)
            trigger = button(obj.joystick_obj, 1);
        end

        function cluster = read_cluster(obj)
            if (any(button(obj.joystick_obj,2:5)))
                cluster = true;
            else
                cluster = false;
            end
        end
    end


end

