classdef Twist2d < handle & matlab.mixin.Copyable

    properties
        dx
        dy
        dtheta
    end

    methods
        %% Default twist constructor.
        % dx is change in x direction relative to robot
        % dy is change in y direction relative to robot
        % dtheta is change in angle relative to robot, in radians
        function obj = Twist2d(dx, dy, dtheta) 
            if (nargin == 0)
                dx = 0.0;
                dy = 0.0;
                dtheta = 0.0;
            else
                obj.dx = dx;
                obj.dy = dy;
                obj.dtheta = dtheta;
            end
        end
    end

end

