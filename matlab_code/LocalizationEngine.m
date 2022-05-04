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
            %obj.odom_pose = Pose2d();
            obj.odom_world_transform = Transform2d.map_poses(Pose2d(),Pose2d());
            obj.robot_pose = Pose2d();
            obj.robot_vel = Twist2d();
            obj.first_run = true;
        end

        function update_odom(obj, odom_twist, system_time)
            if (obj.first_run)
                obj.last_timestamp = system_time;
                obj.first_run = false;
                return;
            end

            loop_dt = system_time - obj.last_timestamp;
            obj.last_timestamp = system_time;

            obj.robot_pose = obj.robot_pose.pose_exp(odom_twist);
            
            %disp("Twist dx:");
            %disp(odom_twist.dx);
            %disp("Loop dt:");
            %disp(loop_dt);

            obj.robot_vel.dx = odom_twist.dx / loop_dt;
            obj.robot_vel.dy = 0;
            obj.robot_vel.dtheta = odom_twist.dtheta / loop_dt;

            %obj.robot_pose = obj.robot_pose.transform_by(obj.odom_world_transform);

            %disp(strcat("Robot pose X: ", ...
            %    num2str(obj.robot_pose.translation.val_x), ...
            %    " Y: ", num2str(obj.robot_pose.translation.val_y), ...
            %    " Θ: ", num2str(obj.robot_pose.rotation.value_degrees)));
        end

        function register_localization_observation(obj, pose_new)
            % get new pose_new and update odom_world_transform such that
            % odom_pose.transform_by(odom_world_transform) == pose_new

            %obj.odom_world_transform = Transform2d.map_poses(obj.odom_pose, pose_new);
            obj.robot_pose = copy(pose_new);
        end

    end

end

