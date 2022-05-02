
calib_params = load("camera_calibration/calib_1024x768/cameraParams.mat");
cam_intrinsics = calib_params.cameraParams.Intrinsics;

cam = Camera(1, cam_intrinsics);
cam.config_resolution('1024x768');

r = rateControl(5);

while (1) 
    img = cam.cap_img();
    %cam.flush_buffer();
    [img_undistort, new_center] = cam.undistort_image(img);
    %imshow(img_undistort);

    [num_tags, tag_ids, tag_img_corners, tag_poses] = AprilTags.detect_tags_in_image(img_undistort, cam_intrinsics);

    img_tags_draw = AprilTags.draw_tags_on_image(img_undistort, cam_intrinsics, num_tags, tag_ids, tag_img_corners, tag_poses);

    imshow(img_tags_draw);


    %waitfor(r);
end

