classdef WaypointNavCommand < Command


    properties (Access = private)
        waypoints;
        end_distance_tolerance;
        max_nav_time;
        start_time;

        pp_controller;
    end

    methods
        % waypoint_list is a list of global Pose2d waypoints
        % end_dist_tolerance is the absolute distance tolerance (in meters) to the last waypoint, to determine if we're done with our path or not.
        % timeout is the max time to run the nav command, and to exit with an error if we don't finish correctly
        % linear_velocity is how fast we want to go, in m/s

        function obj = WaypointNavCommand(rover_handle, waypoint_list, end_dist_tolerance, timeout, linear_velocity)
            obj@Command(rover_handle); % Pass rover_handle to the superclass Command constructor
            obj.end_distance_tolerance = end_dist_tolerance;
            obj.max_nav_time = timeout;
            obj.waypoints = waypoint_list;

            waypoints_converted = zeros(size(waypoint_list,1), 2);
            for i=1:size(waypoint_list,1)
                waypoints_converted(:,i) = [waypoint_list(i).translation.val_x waypoint_list(i).translation.val_y];
            end

            obj.pp_controller = controllerPurePursuit( ...
                'DesiredLinearVelocity',linear_velocity, ...
                'LookaheadDistance', 1.0, ...
                'MaxAngularVelocity', 1.0, ...
                'Waypoints', waypoints_converted ...
                );
        end

        function initialize(obj)
            obj.start_time = obj.rover_handle.system_time;
        end

        function execute(obj)
            robot_pose = obj.rover_handle.localization.current_pose;
            robot_pose_xyt = [robot_pose.translation.val_x robot_pose.translation.val_y robot_pose.rotation.value_radians];
            [vel, ang_vel] = obj.pp_controller(robot_pose_xyt);
            obj.rover_handle.drivetrain_controller.set_vel_setpoints(vel,ang_vel);
        end

        function done = is_done(obj)
            is_timeout = obj.rover_handle.system_time > obj.start_time + obj.max_nav_time;

            current_translation = obj.rover_handle.localization.current_pose.translation;
            final_waypoint_translation = obj.waypoints(end).translation;
            translation_delta = current_translation.get_distance(final_waypoint_translation);

            is_path_complete = abs(translation_delta) < abs(obj.end_distance_tolerance);

            done = is_timeout || is_path_complete;
        end

        function cmd_end(obj)
            obj.rover_handle.drivetrain_controller.set_vel_setpoints(0,0);
        end

    end
end