classdef Translation2d < handle & matlab.mixin.Copyable
    %% Represents a translation in a 2d coordinate frame, represented by a simple x,y vector
    properties
        val_x
        val_y
    end

    methods (Static)
        %% Returns a Translation2d created from a distance and a Rotation2d - essentially a polar to cartesian conversion
        function t2d = from_polar(distance, r2d_angle)
            t2d = Translation2d(distance*r2d_angle.get_cos(), distance*r2d_angle.get_sin());
        end
    end

    methods
        %% Default Translation2d constructor - takes in the [x y] vector representing position
        function obj = Translation2d(x, y)
            if (nargin == 0) % If we get called with no arguments
                obj.val_x = 0.0;
                obj.val_y = 0.0;
            else
                obj.val_x = x;
                obj.val_y = y;
            end
        end

        %% Gets the distance between two Translation2d objects
        function dist = get_distance(obj, other)
            dist = norm([other.val_x - obj.val_x, other.val_y - obj.val_y]);
        end

        %% Gets the x component of the Translation2d
        function x = get_x(obj)
            x = obj.val_x;
        end

        %% Gets the y component of the Translation2d
        function y = get_y(obj)
            y = obj.val_y;
        end

        %% Rotates a Translation2d by a Rotation2d
        % For example, rotating a Translation2d of [2 0] by 90 degrees will return [0 2]
        function t2d_rotated = rotate_by(obj, rot) 
            x_new = obj.val_x * rot.get_cos() - obj.val_y * rot.get_sin();
            y_new = obj.val_x * rot.get_sin() + obj.val_y * rot.get_cos();
            t2d_rotated = Translation2d(x_new, y_new);
        end

        %% Sums two Translation2d -- essentially vector addition
        function t2d_sum = plus(obj, other)
            t2d_sum = Translation2d(obj.val_x + other.val_x, obj.val_y + other.val_y);
        end

        %% Subtract two Translation2d -- essentially vector addition
        function t2d_minus = minus(obj, other)
            t2d_minus = Translation2d(obj.val_x - other.val_x, obj.val_y - other.val_y);
        end

        %% Returns the inverse of the current translation
        function t2d_unary_minus = unary_minus(obj)
            t2d_unary_minus = Translation2d(-obj.val_x, -obj.val_y);
        end

        %% Scales the Translation2d by a scalar
        function t2d_scaled = times(obj, scalar)
            t2d_scaled = Translation2d(obj.val_x * scalar, obj.val_y * scalar);
        end

        %% Divides the Translation2d by a scalar
        function t2d_div = div(obj, scalar)
            t2d_div = Translation2d(obj.val_x / scalar, obj.val_y / scalar);
        end


    end


end
