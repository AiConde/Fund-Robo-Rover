classdef AprilTags

    properties (Constant)
        tag_size_meters = 0.019; % TODO what is it irl?
        tag_family = "tag36h11"; % TODO what is it irl?
        worldPoints = [0 0 0; AprilTags.tag_size_meters/2 0 0; 0 AprilTags.tag_size_meters/2 0; 0 0 AprilTags.tag_size_meters/2];
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
                disp(tag_poses(i).T);
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
    end

end

