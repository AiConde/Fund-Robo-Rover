classdef FindLocalizeAprilTagCommand < Command

    properties (Constant)
        ANGLE_MIN_DEG = -150;
        ANGLE_MAX_DEG = 150;
        NUM_PTS = 15;
        SERVO_RESET_DWELL_SECONDS = 2;
        SERVO_DWELL_SECONDS = 1
        NUM_IMAGE_AVGS = 10; % Capture 10 images of final tag
        NUM_IMAGE_OUTLIER_REJECTION = 0.33; % Reject 1/3 of outliers
        WAIT_BETWEEN_IMAGES_SECONDS = 0.1;
    end

    properties (Access = private)
        angle_list;
        best_candidate_id;
        best_candidate_cam_size;
        best_candidate_world_pose;
        best_candidate_cam_angle;
        tag_found;

        angle_idx;

        start_first_move_time;
        start_move_time;
        last_image_acq_time;

        is_servo_reset;
        is_servo_moving;

        is_done_scanning;
        is_done_final_scan;

        img_samples_left;

        final_poll_num_ok_frames;
        final_poll_tag_poses;
    end

    methods
        function obj = FindLocalizeAprilTagCommand(rover_handle)
            obj@Command(rover_handle);
            obj.angle_list = linspace( ...
                FindLocalizeAprilTagCommand.ANGLE_MIN_DEG, ...
                FindLocalizeAprilTagCommand.ANGLE_MAX_DEG, ...
                FindLocalizeAprilTagCommand.NUM_PTS ...
                );

            obj.tag_found = false;
            obj.best_candidate_cam_size = 0;
            obj.angle_idx = 1;

            obj.is_servo_reset = false;
            obj.is_servo_moving = false;

            obj.is_done_scanning = false;
            obj.is_done_final_scan = false;

            obj.img_samples_left = 0;

            obj.final_poll_num_ok_frames = 0;
            obj.final_poll_tag_poses = [];

        end

        function initialize(obj)
            servo_pwm = Arduino_ROS.pan_angle_to_servo(obj.angle_list(obj.angle_idx));
            obj.rover_handle.arduino.write_pan_servo(servo_pwm);
            obj.start_first_move_time = obj.rover_handle.system_time;
            obj.start_move_time = obj.rover_handle.system_time;
            obj.last_image_acq_time = obj.rover_handle.system_time;
        end

        function execute(obj) % Called 50hz in a loop


            if ~(obj.is_done_scanning || obj.is_done_final_scan) % The main tag search & survey routine

                if (~obj.is_servo_reset) % If we're waiting to move to the first position
                    if (obj.rover_handle.system_time > obj.start_first_move_time + FindLocalizeAprilTagCommand.SERVO_RESET_DWELL_SECONDS)
                        obj.is_servo_reset = true;
                    end

                elseif (obj.is_servo_moving) % If we're waiting to move to subsequent positions
                    if (obj.rover_handle.system_time > obj.start_move_time + FindLocalizeAprilTagCommand.SERVO_DWELL_SECONDS)
                        obj.is_servo_moving = false;
                    end

                else % We're settled at our sample position, time to do our capture loop

                    % Grab image, find any tags
                    img_cap = obj.rover_handle.camera.get_image_raw();
                    [img_undistort, new_center] = obj.rover_handle.camera.undistort_image(img_cap);
                    [num_tags, tag_ids, tag_img_corners, tag_poses] = AprilTags.detect_tags_in_image(img_undistort, cam_intrinsics);

                    % Loop through each tag
                    for tag_idx=1:num_tags
                        % Check if it's a known localization tag
                        if (AprilTags.is_localization_tag(tag_ids(tag_idx)))
                            % Grab the image points
                            tag_cam_pts = tag_img_corners(:,:,tag_idx);
                            % Calc the image area
                            cam_pts_area = polyarea(tag_cam_pts(:,1),tag_cam_pts(:,2));
                            % If this is the biggest tag we've found....
                            if (cam_pts_area > obj.best_candidate_cam_size)
                                obj.tag_found = true;
                                obj.best_candidate_cam_size = cam_pts_area; % Grab area for future comparison
                                obj.best_candidate_id = tag_ids(tag_idx); % Store ID
                                obj.best_candidate_world_pose = tag_poses(tag_idx); % Store world pose

                                % Calculate world angle relative to robot so we can come back and take a final pic once we're done
                                tag_mean_center_point = mean(tag_cam_pts);
                                f_est_px = (0.5 * 1024) / tand(0.5 * Camera_ROS.CAMERA_HFOV_DEGREES);
                                angle_rad = atan(tag_mean_center_point/f_est_px);
                                angle_deg = rad2deg(angle_rad);
                                angle_world = obj.angle_list(obj.angle_idx) + angle_deg;
                                obj.best_candidate_cam_angle = angle_world;
                            end
                        end
                    end

                    % If we've finished scanning, we need to turn the servo to the angle of the best found tag for the final localization
                    if (obj.angle_idx == size(obj.angle_list,2))
                        % If we haven't found a tag, we're fucked. Just give up and say we were successful - we'll deal with this in cmd_end
                        obj.is_done_scanning = true;
                        if (~obj.tag_found)
                            obj.is_done_final_scan = true;
                            servo_pwm = 0.5;
                        else
                            servo_pwm = Arduino_ROS.pan_angle_to_servo(obj.best_candidate_cam_angle);
                            obj.img_samples_left = FindLocalizeAprilTagCommand.NUM_IMAGE_AVGS;
                        end
                    else % Else, we move to the next angle
                        obj.angle_idx = obj.angle_idx + 1;
                        servo_pwm = Arduino_ROS.pan_angle_to_servo(obj.angle_list(obj.angle_idx));
                    end

                    obj.rover_handle.arduino.write_pan_servo(servo_pwm);
                    obj.start_move_time = obj.rover_handle.system_time;
                    obj.is_servo_moving = true;
                end

            else % if obj.is_done_scanning || obj.is_done_final_scan

                if (~obj.is_done_final_scan) % && obj.is_done_scanning
                    % Now, we've finished the initial scan, but haven't finished the precise zero-in
                    % We need to wait until we move to the "best tag" position
                    if (obj.is_servo_moving)
                        if (obj.rover_handle.system_time > obj.SERVO_RESET_DWELL_SECONDS + obj.start_move_time)
                            obj.is_servo_moving = false;
                        end

                    else % Our servo is settled at the "best tag" location.
                        % Now we're going to find the tag-of-interest pose in multiple images and average/outlier reject/etc

                        % If we've waited long enough since taking the last image
                        if (obj.rover_handle.system_time > obj.WAIT_BETWEEN_IMAGES_SECONDS + obj.last_image_acq_time)
                            img_cap = obj.rover_handle.camera.get_image_raw();
                            
                            [img_undistort, new_center] = obj.rover_handle.camera.undistort_image(img_cap);
                            [num_tags, tag_ids, tag_img_corners, tag_poses] = AprilTags.detect_tags_in_image(img_undistort, cam_intrinsics);
                            
                            % If we found the tag we were looking for, write it to the buffer
                            if (any(obj.best_candidate_id == tag_ids))
                                tag_pose = tag_poses(obj.best_candidate_id == tag_ids);
                                obj.final_poll_num_ok_frames = obj.final_poll_num_ok_frames + 1;
                                obj.final_poll_tag_poses = [obj.final_poll_tag_poses ; tag_pose];
                            end
                            
                            obj.last_image_acq_time = obj.rover_handle.system_time;
                            obj.img_samples_left = obj.img_samples_left - 1;

                            % Check if we have any more scans left to do. If not, we're done!
                            obj.is_done_final_scan = obj.img_samples_left == 0;
                        end

                    end
                else % if (obj.is_done_final_scan)
                    % If is_done_final_scan is true, then we should do nothing, just wait - all the final localization
                    % math will be done in the cmd_end() routine;
                end
            end
        end

        function done = is_done(obj)
            done = obj.is_done_final_scan;
        end

        function cmd_end(obj)
            if (~obj.tag_found)
                % Lol we're fucked
                disp("WARNING: NO TAG FOUND IN FINDLOCALIZEAPRILTAG");
            else
                num_outliers_detect = floor(FindLocalizeAprilTagCommand.NUM_IMAGE_OUTLIER_REJECTION * obj.final_poll_num_ok_frames);
                
                % Run Ben's localization code for each april tag
                AprilTags
                

            end
        end

    end
end

