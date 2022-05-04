classdef DriveDockCommand  < Command


    properties (Access = private)
        waypoints;  % list of local wayponts
        end_distance_tolerance;  %    
        final_position;
        dock_distance;
        start_time;  % time cutoff(the time in which the dock command should start)
      
        pp_controller;
    end

    methods
        % waypoint_list is a list of global Pose2d waypoints
        % end_dist_tolerance is the absolute distance tolerance (in meters) to the last waypoint, to determine if we're done with our path or not.
        % linear_velocity is how fast we want to go, in m/s
        % final_position is the final global position we want the rover to
        % end up in
        % dock_apriltag_distance is the distance between the rover and the
        % dock

        function obj = DriveDockCommand(rover_handle, DrivetrainController,waypoint_list, end_dist_tolerance,linear_velocity,final_position,dock_distance,start_postion)
            obj@Command(rover_handle); % Pass rover_handle to the superclass Command constructor
            obj.end_distance_tolerance = end_dist_tolerance;
            obj.waypoints = waypoint_list;
            obj.final_position = final_position;
            obj.DrivetrainController = DrivetrainController();
            obj.dock_distance = dock_distance;
            obj.start_position = start_postion;

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
            % check the time, if over 13 minutes then check the robot pose.
            % If within a certain amount time, check and store robot intial
            % position
            % store dock pose
            obj.dock_pose = obj.rover_handle.dock_pose.p2d.translation;
           
            


        end

        function execute(obj)
            % In this function the robot will make a trajector of waypoints in order to move within 4.5 meters of the end dock: make this a function 
            robot_pose = obj.rover_handle.localization.current_pose;
            robot_pose_xyt = [robot_pose.translation.val_x robot_pose.translation.val_y robot_pose.rotation.value_radians];
            [vel, ang_vel] = obj.pp_controller(robot_pose_xyt);
            obj.rover_handle.drivetrain_controller.set_vel_setpoints(vel,ang_vel);
                
            end
        end

        function done = is_done(obj)
            % if rover is within 4 meeters of the dock, then the command is
            % done
            

            current_translation = obj.rover_handle.localization.current_pose.translation;
            final_position = obj.waypoints(end).translation;
            translation_delta = current_translation.get_distance(final_position);

            is_path_complete = abs(translation_delta) < abs(obj.end_distance_tolerance);
             
            

            done = is_done || is_path_complete;
        end

        function cmd_end(obj)
            obj.rover_handle.drivetrain_controller.set_vel_setpoints(0,0);
        end

            end
end