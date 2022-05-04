classdef SimpleAprilTagLocalize < Command
%SimpleAprilTagLocalize takes img from cam and localizes from biggest tag

    properties (Access = private)
        tags_detected        % bool for whether tags could be detected
        localized_pose       % pose of rover from tag localization
    end

    properties (Access = public)
        img_undistort  % the camera image we detect tags from
    end

    methods
        function obj = SimpleAprilTagLocalize(rover_handle)

            obj@Command(rover_handle); % Pass rover_handle to the superclass Command constructor
        end

        function initialize(obj)
            disp("Running SimpleAprilTagLocalize");
        end

        function execute(obj)
            pan_rot = Rotation2d.from_degrees(0); % this is hard-coded for now

            % get image from camera
            img = rgb2gray(cam.get_image_raw());
            [obj.img_undistort, ~] = cam.undistort_image(img);

            % run april tag detection
            [num_tags, tag_ids, tag_img_corners, rigid3ds] = ...
                AprilTags.detect_tags_in_image( ...
                obj.img_undistort, obj.rover_handle.cam.cam_intrinsics);

            % if no tag detected, disp so and return
            if ~num_tags
                obj.tags_detected = false;
                disp("no tags detected")
                return
            else
                obj.tags_detected = true;
            end

            % find largest tag
            [rigid3d, tag_id] = get_largest_tag( ...
                num_tags, tag_ids, tag_img_corners, rigid3ds);

            % we cannot localize from these tagids because they dont have
            % defined poses
            bad_tag_ids = [18 17 0];
            if ismember(tag_id, bad_tag_ids)
                obj.tags_detected = false;
                disp(["only bad tags detected: tagid ", tag_id])
                return
            end
            
            % get rover pose from that tag's rigid3d
            robot_pose = AprilTags.get_robot_pose_from_localization_tag( ...
                tag_id, rigid3d, pan_rot);
            obj.localized_pose = robot_pose;

            % update rover object with localized robot pose
            obj.rover_handle.odometry.set_pose(robot_pose);
            obj.rover_handle.write_localization(robot_pose);
        end

        function done = is_done(obj)
            done = true; % this command will always be done after the first run of execute()
        end

        function cmd_end(obj)
            disp("Done with SimpleAprilTagLocalize");
            disp(["localized pose at: ", obj.localized_pose]);
        end

    end
end

function [largest_tag_rigid3d, largest_tag_id] = get_largest_tag(num_tags, tag_ids, tag_img_corners, rigid3ds)
    % takes a list of tag image corners and returns the index of the
    % largest size tag in that list
    tag_areas = zeros(1, num_tags);
    for i = 1:num_tags
        tag_areas(i) = polyarea(tag_img_corners(:,1,i), tag_img_corners(:,2,i));
    end
    
    % find index of largest tag
    [~, idx_largest] = max(tag_areas);

    % get detected rigid3d from that index
    largest_tag_rigid3d = rigid3ds(idx_largest);
    largest_tag_id = tag_ids(idx_largest);
end