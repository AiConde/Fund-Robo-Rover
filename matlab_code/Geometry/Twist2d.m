%% Represents the derivative of a position, translation, and rotation vector for a pose2d object.
% Part of a series of classes called:
% Pose2d, Transform2d, Translation2d, Rotation2d, and Twist2d. Read Pose2d.m for
% more details.
% A translation2d object will have properties that store values for x and y.

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
                obj.dx = 0.0;
                obj.dy = 0.0;
                obj.dtheta = 0.0;
            else
                obj.dx = double(dx);
                obj.dy = double(dy);
                obj.dtheta = double(dtheta);
            end
        end
    end
end

