commands = ["roscore",
            "roslaunch urg_node urg_lidar.launch",
            "roslaunch usb_camera usb_cam.launch",
%             "roslaunch ublox m8u_rover.launch",
            ];

% for i = 1:length(commands)
%     system("ls");
% end

system("roscore");