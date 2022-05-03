classdef Rotation2d < handle & matlab.mixin.Copyable
    %% Represents a rotation in a 2d coordinate frame, represented by a rotation vector consisting of a cosine and sine component

    properties
        value_radians % The radians value of the rotation
        vec_cos % The cosine component of the rotation vector
        vec_sin % The sine component of the rotation vector
    end

    methods (Static)

        %% Create new Rotation2d from a rotation value in radians
        % e.g. cam_rotation = Rotation2d.from_radians(pan_angle_radians);
        function r2d = from_radians(value_radians)
            r2d = Rotation2d(cos(value_radians), sin(value_radians));
        end

        %% Create new Rotation2d from a rotation value in degrees
        % e.g. cam_rotation = Rotation2d.from_degrees(pan_angle_degrees);
        function r2d = from_degrees(value_radians)
            r2d = Rotation2d(cosd(value_radians), sind(value_radians));
        end
    end

    methods
        %% Default Rotation2d constructor - takes in a [x y] vector representing a rotation, and normalizes it before constructing the rotation
        function obj = Rotation2d(x, y)
            if (nargin == 0) % If we get called with no arguments
                obj.vec_cos = 1.0;
                obj.vec_sin = 0.0;
            else
                vec_mag = norm([x y]); % Vector magnitude/norm to normalize by
                if (vec_mag > 1E-6) % If vec_mag is extremely small, we can't divide by it
                    obj.vec_cos = x / vec_mag;
                    obj.vec_sin = y / vec_mag;
                else
                    obj.vec_cos = 1.0;  % Create a vector in the 0° direction if we get an infinitely small vector input
                    obj.vec_sin = 0.0;
                end
            end
            obj.value_radians = atan2(obj.vec_sin, obj.vec_cos); % Compute vector radian value
        end

        %% Return a new Rotation2d that is the result of this one rotated by another
        function r2d_plus = plus(obj, other)
            r2d_plus = obj.rotate_by(other);
        end

        %% Return a new Rotation2d that is the result of this one rotated by the inverse of the other
        function r2d_minus = minus(obj, other)
            r2d_minus = obj.rotate_by(other.unary_minus());
        end

        %% Return a new Rotation2d that is the inverse of this one
        function r2d_unary_minus = unary_minus(obj)
            r2d_unary_minus= Rotation2d.from_radians(-obj.value_radians);
        end

        %% Return a new Rotation2d that is the current rotation scaled by a scalar
        function r2d_times = times(obj, scalar) 
            r2d_times = Rotation2d.from_radians(obj.value_radians * scalar);
        end

        %% Return a new Rotation2d that is the current rotation rotated by another Rotation2d
        function r2d_rotateby = rotate_by(obj, other) 
            cos_new = obj.vec_cos * other.vec_cos - obj.vec_sin * other.vec_sin;
            sin_new = obj.vec_cos * other.vec_sin + obj.vec_sin * other.vec_cos;
            r2d_rotateby = Rotation2d(cos_new, sin_new);
        end

        %% Get the rotation of the current Rotation2d, in radians
        function rads = get_radians(obj)
            rads = obj.value_radians;
        end

        %% Get the rotation of the current Rotation2d, in degrees
        function degs = get_degrees(obj)
            degs = rad2deg(obj.value_radians);
        end

        %% Get the sin component of the current Rotation2d
        function vecsin = get_sin(obj)
            vecsin = obj.vec_sin;
        end

        %% Get the cos component of the current Rotation2d
        function vecsin = get_cos(obj)
            vecsin = obj.vec_cos;
        end

    end
end


