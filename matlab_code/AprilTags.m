classdef AprilTags

    properties (Constant)
        tag_size_meters = 0.164; % TODO what is it irl?
        tag_family = "tag36h11"; % TODO what is it irl?
        worldPoints = [0 0 0; AprilTags.tag_size_meters/2 0 0; 0 AprilTags.tag_size_meters/2 0; 0 0 AprilTags.tag_size_meters/2];

        localization_tag_ids = [1 2 3 4 5 6 7 8 9 10 11 13 14 15 16];
        localization_tag_poses = [
            Pose2d.from_xydeg(-6.860, 8.502, -85)
            Pose2d.from_xydeg(-4.188, 23.775, -109.6)
            Pose2d.from_xydeg(0.847, 34.282, -120)
            Pose2d.from_xydeg(12.418, 39.531, -135)
            Pose2d.from_xydeg(10.230, 46.645, -135)
            Pose2d.from_xydeg(17.400, 44.631, -137)
            Pose2d.from_xydeg(22.329, 56.340, -150)
            Pose2d.from_xydeg(40.329, 63.738, -167.4)
            Pose2d.from_xydeg(59.013, 62.425, 150)
            Pose2d.from_xydeg(70.776, 48.080, 105)
            Pose2d.from_xydeg(71.186, 36.752, 77)
            Pose2d.from_xydeg(50.138, 10.889, 45)
            Pose2d.from_xydeg(39.286, 2.134, 33)
            Pose2d.from_xydeg(26.494, -3.701, 17)
            Pose2d.from_xydeg(14.098, -5.531, -9)
            ];
        dock_tag_id = 0;
        gate_tag_id = 17;
    end

    methods (Static)
        function [num_tags, tag_ids, tag_img_corner_pts, tag_poses] = detect_tags_in_image(img_undistorted, camera_intrinsics)
            [id, loc, pose] = readAprilTag(img_undistorted, AprilTags.tag_family, camera_intrinsics, AprilTags.tag_size_meters);
            num_tags = length(id);
            tag_ids = id;
            tag_img_corner_pts = loc;
            tag_poses = pose;
        end

        function img_tags = draw_tags_on_image(img_undistorted, camera_intrinsics, num_tags, tag_ids, tag_img_corner_pts, tag_poses)
            I = img_undistorted;
            for i = 1:length(tag_poses)
%                 disp(tag_poses(i).T);
                % Get image coordinates for axes.
                imagePoints = worldToImage(camera_intrinsics,tag_poses(i).Rotation, ...
                    tag_poses(i).Translation,AprilTags.worldPoints);

                % Draw colored axes.
                I = insertShape(I,"Line",[imagePoints(1,:) imagePoints(2,:); ...
                    imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
                    "Color",["red","green","blue"],"LineWidth",7);

                I = insertText(I,tag_img_corner_pts(1,:,i),tag_ids(i),"BoxOpacity",1,"FontSize",25);
            end
            img_tags = I;
        end

        function is_dock_tag = is_dock_tag(tagid)
            is_dock_tag = tagid == AprilTags.dock_tag_id;
        end

        function is_gate_tag = is_gate_tag(tagid)
            is_gate_tag = tagid == AprilTags.gate_tag_id;
        end

        function is_localization_tag = is_localization_tag(tagid)
            is_localization_tag = any(AprilTags.localization_tag_ids == tagid);
        end

        function pose = get_pose_of_localization_tag(tagid)
            if (~AprilTags.is_localization_tag(tagid))
                error("Given tag ID is not a localization tag!");
            end
            pose = AprilTags.localization_tag_poses((AprilTags.localization_tag_ids == tagid));
        end

        function robot_pose = get_robot_pose_from_localization_tag(tagid, detected_pose, tag_image_pts)
            % Args:
            %   tagid: int, tag ID
            %   tag_pose: MATLAB rigid3d object, detected pose of april tag
            %   tag_image_pts: 2D tag corner points in image
            % Returns:
            %   robot_pose: Pose2d of robot in world frame

            % the transpose of T should be the cam T in tag coords
            t_inv = detected_pose.T^-1;

            % get camera pose in tag frame
            cam_pose_tag_frame = Pose2d( ...
                Translation2d(-t_inv(4, 1), -t_inv(4, 3)),...
                Rotation2d(t_inv(1,3), t_inv(3,3)).unary_minus);

            % translate cam pose by tag pose to get cam pose in world frame
            tag_pose = AprilTags.get_pose_of_localization_tag(tagid);

            cam_pose_world_frame = cam_pose_tag_frame.plus(tag_pose);
            robot_pose = cam_pose_world_frame;
        end

    end

end

