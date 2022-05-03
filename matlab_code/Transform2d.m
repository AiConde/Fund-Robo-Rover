classdef Transform2d < handle & matlab.mixin.Copyable

    properties
        translation
        rotation
    end

    methods (Static)
        %% Returns a Transform2d that maps from pose_initial to pose_last
        function tfm2d = map_poses(pose_initial, pose_last)
            translation = pose_last.translation.minus(pose_initial.translation).rotate_by(pose_initial.rotation.unary_minus());
            rotation = pose_last.rotation.minus(pose_initial.rotation);
            tfm2d = Transform2d(translation, rotation);
        end
    end

    methods
        %% Default constructor of Transform2d - takes in translation and rotation elements
        function obj = Transform2d(translation, rotation)
            if (nargin == 0)
                obj.translation = Translation2d();
                obj.rotation = Rotation2d();
            else
                obj.translation = Translation2d(translation.val_x, translation.val_y);
                obj.rotation = Rotation2d(rotation.vec_cos, rotation.vec_sin);
            end
        end

        %% Multiplies transform by a scalar
        function tfm2d_times = times(obj, scalar)
            tfm2d_times = Transform2d(obj.translation.times(scalar), obj.rotation.times(scalar));
        end

        %% Composes two transforms
        function tfm2d_plus = plus(obj, other)
            obj_copy = Transform2d(obj.translation, obj.rotation);
            tfm2d_plus = Transform2d.map_poses(Pose2d(), Pose2d().transform_by(obj_copy).transform_by(other));
        end

        %% Returns translation element
        function t2d = get_translation(obj)
            t2d =  obj.translation;
        end

        %% Returns rotation element
        function r2d = get_rotation(obj)
            r2d= obj.rotation;
        end

        %% Returns x element of the translation
        function x = get_x(obj)
            x= obj.translation.get_x();
        end

        %% Returns y element of the translation
        function y = get_y(obj)
            y= obj.translation.get_y();
        end
    end

end
