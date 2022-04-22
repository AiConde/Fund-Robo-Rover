clear
rosshutdown % Clear any existing ROS connection

% Initialize ROS connection
ROS_URI = 'http://192.168.34.152:11311/';
rosinit(ROS_URI)

lidar = Lidar_ROS();

pause(1)

while (1)
    pause(0.1);
    %rosPlot(lidar.get_scandata_raw);
    [angles, ranges] = lidar.get_scandata();
    polarplot(angles, ranges, 'r.')
end