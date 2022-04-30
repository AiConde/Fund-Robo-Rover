classdef Odometry < handle

    properties (Constant)
        WHEEL_DIAMETER_METERS = 0.01518617811430287;
        ENCODER_CPR = 2;
        WHEELBASE_METERS = 0.422275;
        TRACKWIDTH_METERS = 0.295275;
    end

    properties
        odom_pose % Pose2d
        odom_twist % Twist2d
        odom_vel % Twist2d
    end

    properties (Access = private)
        last_timestamp % tic/toc timestamp of last run
        last_tacometer_value % last tacometer value, used for computing distance traveled

        first_run % Is this the first time running? ie should we zero the timestamps
    end

    methods

        function obj = Odometry(start_pose)
            if (nargin == 0)
                obj.odom_pose = Pose2d.from_xydeg(0, 0, 0); % If a start pose wasn't specified, set to 0,0 with a heading of 0°
            else
                obj.odom_pose = copy(start_pose); % Else start from the specified start pose
            end

            obj.odom_twist = Twist2d();
            obj.odom_vel = Twist2d();

            obj.first_run = true;
        end

        %% Main update loop, only using wheel encoder + steering angle (degrees)
        function update(obj, tacometer_value, steer_angle_deg)
            if (obj.first_run)
                obj.last_timestamp = tic;
                obj.last_tacometer_value = tacometer_value;
                obj.first_run = false;
                return;
            end
            %loop_dt = toc(obj.last_timestamp);
            loop_dt = 0.01;
            obj.last_timestamp = tic;

            tacometer_distance = tacometer_value - obj.last_tacometer_value;
            obj.last_tacometer_value = tacometer_value;

            fwd_distance = (tacometer_distance / Odometry.ENCODER_CPR) * pi * Odometry.WHEEL_DIAMETER_METERS;
            steering_angle = steer_angle_deg;
            dtheta = fwd_distance * tand(steer_angle_deg) / Odometry.WHEELBASE_METERS;

            v = fwd_distance / loop_dt;
            w = dtheta / loop_dt;

            obj.odom_vel.dx = v;
            obj.odom_vel.dy = 0;
            obj.odom_vel.dtheta = w;

            obj.odom_twist.dx = fwd_distance;
            obj.odom_twist.dy = 0;
            obj.odom_twist.dtheta = dtheta;

            %disp(["dtheta: ", dtheta])
            %disp(["dist: ", fwd_distance])

            obj.odom_pose = obj.odom_pose.pose_exp(obj.odom_twist);
        end

        %% Main update loop, using wheel encodes + angular velocity from gyro (rad/s) 
        function update_imu(obj, tacometer_value, gyro_angularvel_rads)
            if (obj.first_run)
                obj.last_timestamp = tic;
                obj.last_tacometer_value = tacometer_value;
                return;
            end

            loop_dt = toc(obj.last_timestamp);
            obj.last_timestamp = tic;

            tacometer_distance = tacometer_value - obj.last_tacometer_value;
            obj.last_tacometer_value = tacometer_value;

            fwd_distance = (tacometer_distance / Odometry.ENCODER_CPR) * pi * Odometry.WHEEL_DIAMETER_METERS;
            w = gyro_angularvel_rads;
            v = fwd_distance / loop_dt;

            dtheta = w * loop_dt;

            obj.odom_vel.dx = v;
            obj.odom_vel.dy = 0;
            obj.odom_vel.dtheta = w;

            obj.odom_twist.dx = fwd_distance;
            obj.odom_twist.dy = 0;
            obj.odom_twist.dtheta = dtheta;

            obj.odom_pose = obj.odom_pose.pose_exp(obj.odom_twist);
        end

        %% Overrides internal pose and blanks the vel and twists
        function set_pose(obj, pose)
            obj.odom_pose = copy(pose);
            obj.odom_vel = Twist2d();
            obj.odom_twist = Twist2d();
        end
    end
end
