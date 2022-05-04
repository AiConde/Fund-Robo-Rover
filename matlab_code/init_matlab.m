function init_matlab()
    %INIT_MATLAB initialize ROS and take care of matlab path
    
    % if you add folders in /matlab_code, add them to the addpath command
    addpath('Commands/', ...
        'Geometry/', ...
        'ROS classes/', ...
        'camera_calibration/calib_1024x768/', ...
        'testing/tag_images/');
    disp("path added!")

    % initialize ROS so we can read from and write to topics
    nuc_ip = 'http://192.168.16.71:11311/';

    try
        init_ROS(nuc_ip);
    catch
        disp("init_ROS failed.")
        disp("Make sure the NUC is on and all ROS nodes are launched")
        disp("make sure that the URI " + nuc_ip + " is correct.")
    end
end

