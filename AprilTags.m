classdef AprilTags

    properties (Constant)
        tag_size_meters = 0.05; % TODO what is it irl?
        tag_family = "tag36h11"; % TODO what is it irl?
    end

    methods (Static)
        function [num_tags, tag_ids, tag_img_corner_pts, tag_poses] = detect_tags_in_image(img_undistorted, camera_intrinsics)
            [id, loc, pose] = readAprilTag(img_undistorted, AprilTags.tag_family, camera_intrinsics, AprilTags.tag_size_meters);
            num_tags = length(id);
            tag_ids = id;
            tag_img_corner_pts = loc;
            tag_poses = pose;
        end

        function img_tags = draw_tags_on_image(img_undistorted, num_tags, tag_ids, tag_img_corner_pts, tag_poses)
        end
    end
    
    end

