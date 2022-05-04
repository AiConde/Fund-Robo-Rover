classdef LocalizationEngine < handle

    properties (Constant)
    end

    properties
        odom_pose;
        odom_world_transform;

        robot_pose;
        robot_vel;

        last_timestamp;
        first_run;
    end

    properties (Access = private)
    end


    methods
        function obj = LocalizationEngine()
            obj.odom_pose = Pose2d();
            obj.odom_world_transform = Transform2d.map_poses(Pose2d(),Pose2d());
            obj.robot_pose = Pose2d();
            obj.robot_vel = Twist2d();
        end

        function update_odom(obj, odom_twist, system_time)
            if (obj.first_run)
                obj.last_timestamp = system_time;
                obj.first_run = false;
                return;
            end

            loop_dt = obj.last_timestamp - system_time;
            obj.last_timestamp = system_time;

            obj.odom_pose = obj.odom_pose.pose_exp(odom_twist);
            
            obj.robot_vel.dx = odom_twist.dx / loop_dt;
            obj.robot_vel.dy = 0;
            obj.robot_vel.dtheta = odom_twist.dtheta / loop_dt;

            obj.robot_pose = obj.odom_pose.transform_by(obj.odom_world_transform);
        end

        function register_localization_observation(obj, pose_new)
            % get new pose_new and update odom_world_transform such that
            % odom_pose.transform_by(odom_world_transform) == pose_new

            obj.odom_world_transform = Transform2d.map_poses(obj.odom_pose, obj.pose_new);
            obj.robot_pose = obj.odom_pose.transform_by(obj.odom_world_transform);
        end

    end

end

