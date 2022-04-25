%% testing MATLAB / ROS / URG Lidar sense integration

% Setup ROS connection to the NUC
init_ROS

% Create lidar object
lidar = Lidar_ROS();

pause(1)

% collect and plot lidar scans
while (1)
    pause(0.1);
    %rosPlot(lidar.get_scandata_raw);
    [angles, ranges] = lidar.get_scandata();
    polarplot(angles, ranges, 'r.')
end