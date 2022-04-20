clear;
lidar = Lidar_ROS();

pause(1)

while (1)
    pause(0.1);
    %rosPlot(lidar.get_scandata_raw);
    [angles, ranges] = lidar.get_scandata();
    polarplot(angles, ranges, 'r.')
end