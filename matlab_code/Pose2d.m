classdef Pose2d < handle & matlab.mixin.Copyable
    %% Represents a pose in a 2d coordinate frame -- i.e. a rotation and a transformation

    properties
        translation
        rotation
    end

    methods (Static)
        %% Returns a Pose2d created from x,y + Rotation2d object
        function p2d = from_xyrot(x, y, r2d)
            p2d = Pose2d(Translation2d(x, y), Rotation2d(r2d.vec_cos, r2d.vec_sin));
        end

        %% Returns a Pose2d created from x,y + rotation in radians
        function p2d = from_xyrad(x, y, rad)
            p2d = Pose2d(Translation2d(x, y), Rotation2d.from_radians(rad));
        end

        %% Returns a Pose2d created from x,y + rotation in degrees
        function p2d = from_xydeg(x, y, deg)
            p2d = Pose2d(Translation2d(x, y), Rotation2d.from_degrees(deg));
        end
    end

    methods
        %% Default Pose2d constructor - creates from a Translation2d + Rotation2d object
        function obj = Pose2d(translation, rotation)
            if (nargin == 0) % If we get called with no arguments
                obj.translation = Translation2d();
                obj.rotation = Rotation2d();
            else
                obj.translation = Translation2d(translation.val_x, translation.val_y);
                obj.rotation = Rotation2d(rotation.vec_cos, rotation.vec_sin);
            end
        end

        %% Returns a new Pose2d object that is this one transformed by another
        function p2d_plus = plus(obj, other)
            p2d_plus = obj.transform_by(other);
        end

        %% Returns a new Pose2d object that is this one transformed by the inverse of the other
        function p2d_minus = minus(obj, other)
            pose_new = obj.relative_to(other);
            p2d_minus = Transform2d(pose_new.get_translation(), pose_new.get_rotation());
        end

        %% Returns the Translation2d component of this Pose2d
        function t2d = get_translation(obj)
            t2d = obj.translation;
        end

        %% Returns the Rotation2d component of this Pose2d
        function r2d = get_rotation(obj)
            r2d = obj.rotation;
        end

        %% Returns the x component of this Pose2d's translation
        function t_x = get_x(obj)
            t_x = obj.translation.get_x();
        end

        %% Returns the y component of this Pose2d's translation
        function t_y = get_y(obj)
            t_y = obj.translation.get_y();
        end

        %% Transforms this pose by a Transform2d and returns the resulting new pose
        function p2d_transform_by = transform_by(obj, other)
            translation_new = obj.translation.plus(other.get_translation().rotate_by(obj.rotation));
            rotation_new = obj.rotation.plus(other.get_rotation());
            p2d_transform_by = Pose2d(translation_new, rotation_new);
        end

        %% Gets the other pose relative to this pose
        function p2d_relative_to = relative_to(obj, other)
            transform_new = Transform2d.map_poses(other, Pose2d(obj.translation, obj.rotation));
            p2d_relative_to = Pose2d(transform_new.get_translation(), transform_new.get_rotation());
        end

        %% Get a new Pose2d from a constant curvature velocity Twist2d
        % The twist is the change in pose in the robot's coordinate frame between two poses.
        % the pose_exp function will "add" the Twist to this pose, getting a new pose represented by this pose "plus" the twist
        % While theoretically you could just add the x,y,theta components of the Twist and Pose, this treats the transform like a transform _and_ a rotation, as opposed to a transform _while rotating_, which is a representation that is both more accurate to the real world, and provides more accurate transforms.
        % pose_exp does the latter, representing the transform as a position change while under a constant angular velocity
        function p2d_exp = pose_exp(obj, twist)
            dx = twist.dx;
            dy = twist.dy;
            dtheta = twist.dtheta;

            sin_theta = sin(dtheta);
            cos_theta = cos(dtheta);

            s = 0.0;
            c = 0.0;

            if (abs(dtheta) < 1E-9) % If we have a very small or zero change in the rotation, we need to take a different approach
                s = 1.0 - ((1.0/6.0) * dtheta * dtheta);
                c = 0.5 * dtheta;
            else
                s = sin_theta / dtheta;
                c = (1 - cos_theta) / dtheta;
            end

            x_new = dx * s - dy * c;
            y_new = dx * c + dy * s;
            transform = Transform2d(Translation2d(x_new, y_new), Rotation2d(cos_theta, sin_theta));
            p2d_exp = obj.plus(transform);
        end

        %% Get a new Twist2d that maps this pose to the end pose 
        % If c == a.pose_log(b), then b == a.pose_exp(c)
        % Just like exponentials and logarithms with Normal Person Math!
        function t2d_log = pose_log(obj, end_pose)
            transform = end_pose.relative_to(Pose2d(obj.translation, obj.rotation));
            dtheta = transform.get_rotation().get_radians();
            half_dtheta = dtheta / 2.0;
            cos_minus_one = transform.get_rotation().get_cos() - 1;

            half_theta_by_tan_of_half_dtheta = 0.0;
            if (abs(cos_minus_one) < 1E-9)
                half_theta_by_tan_of_half_dtheta = 1.0 - ((1.0/12.0) * dtheta * dtheta);
            else
                half_theta_by_tan_of_half_dtheta = -(half_dtheta * transform.get_rotation().get_sin()) / cos_minus_one;
            end

            rotation_apply = Rotation2d(half_theta_by_tan_of_half_dtheta, -half_dtheta);
            vec_norm = norm([half_theta_by_tan_of_half_dtheta, half_dtheta]);
            translation_part = transform.get_translation().rotate_by(rotation_apply).times(vec_norm);

            t2d_log = Twist2d(translation_part.get_x(), translation_part.get_y(), dtheta);
        end

    end

end
