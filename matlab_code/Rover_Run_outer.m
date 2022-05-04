clc;
clear;
init_matlab(); % start ROS server and set matlab path

rover = Rover(false); %Initialize rover without joystick


% 1:5  6:10  11:14  15:20  20:24  21:24

oval_nav_pts = [
Pose2d.from_xydeg(   -3.3895 , 12.5338, 0);
Pose2d.from_xydeg(   -1.8964 , 19.2005, 0);
Pose2d.from_xydeg(    0.5921 , 26.1978, 0);
Pose2d.from_xydeg(    3.9101 , 32.6993, 0);
Pose2d.from_xydeg(    8.0576 , 38.8150, 0); % stop and localize
Pose2d.from_xydeg(   12.9793 , 44.2697, 0);
Pose2d.from_xydeg(   18.3987 , 49.2284, 0);
Pose2d.from_xydeg(   24.5370 , 53.2505, 0);
Pose2d.from_xydeg(   31.0071 , 56.7216, 0);
Pose2d.from_xydeg(   37.9748 , 59.3112, 0); % stop and localize
Pose2d.from_xydeg(   45.0532 , 60.9641, 0);
Pose2d.from_xydeg(   53.3482 , 60.7989, 0);
Pose2d.from_xydeg(   60.7031 , 57.2177, 0);
Pose2d.from_xydeg(   65.6802 , 52.5896, 0); % stop and localize
Pose2d.from_xydeg(   67.7263 , 47.0799, 0);
Pose2d.from_xydeg(   68.6112 , 40.0826, 0);
Pose2d.from_xydeg(   67.2288 , 33.1404, 0);
Pose2d.from_xydeg(   56.0030 , 12.2034, 0);
Pose2d.from_xydeg(   49.4776 ,  5.6468, 0);
Pose2d.from_xydeg(   43.6711 ,  1.1839, 0); % stop and localize
Pose2d.from_xydeg(   37.4222 , -2.7832, 0);
Pose2d.from_xydeg(   30.6755 , -5.5381, 0);
Pose2d.from_xydeg(   23.6524 , -7.6318, 0);
Pose2d.from_xydeg(   16.8504 , -8.6786, 0); % stop and localize
Pose2d.from_xydeg(    4.5184 , -4.6566, 0);
Pose2d.from_xydeg(   -0.0715 ,  0.0267, 0);
Pose2d.from_xydeg(   -7.189516129032258 ,  -1.7265016685205783, 0);
];

% 1:5  6:10  11:14  15:20  20:24
nav_pts_segmented = {
    oval_nav_pts(1:5);
    oval_nav_pts(6:10);
    oval_nav_pts(11:14);
    oval_nav_pts(15:20);
    oval_nav_pts(20:24);
    oval_nav_pts(25:end);
};

figure; hold on; axis equal;


waypoints = zeros(length(oval_nav_pts), 2);
for i=1:length(oval_nav_pts)
%     % plot pose i
%     Utils.quiver_pose(oval_nav_pts(i));
    navptx = oval_nav_pts(i).translation.val_x;
    navpty = oval_nav_pts(i).translation.val_y;
    waypoints(i,:) = [navptx, navpty];
end
plot(waypoints(:,1), waypoints(:,2), "-o");


% define list of rover commands to run
rover.set_mission_command_list({
    CalibrateGyroCommand(rover);
    
    HoldHorses(rover,2);
    SimpleAprilTagLocalize(rover);
    WaypointNavCommand(rover, nav_pts_segmented{1}, 0.5, 240, 2.2);

    HoldHorses(rover,2);
    SimpleAprilTagLocalize(rover);
    WaypointNavCommand(rover, nav_pts_segmented{2}, 0.5, 240, 2.2);

    HoldHorses(rover,2);
    SimpleAprilTagLocalize(rover);
    WaypointNavCommand(rover, nav_pts_segmented{3}, 0.5, 240, 2.2);

    HoldHorses(rover,2);
    SimpleAprilTagLocalize(rover);
    WaypointNavCommand(rover, nav_pts_segmented{4}, 0.5, 240, 2.2);

    HoldHorses(rover,2);
    SimpleAprilTagLocalize(rover);
    WaypointNavCommand(rover, nav_pts_segmented{5}, 0.5, 240, 2.2);

    WaypointNavCommand(rover, nav_pts_segmented{6}, 0.5, 240, 2.2);
})


start_pose_outer = Pose2d.from_xydeg(-5.249930478309232,1.582591768631813,24.6);


start_pose = copy(start_pose_inner);
rover.odometry.set_pose(copy(start_pose));
rover.write_localization(copy(start_pose));
rover.robot_init();
rover.odometry.set_pose(copy(start_pose));
rover.write_localization(copy(start_pose));

rover_timestamps = [];
rover_poses = [];
rover_vel_twists = [];



while (rover.mission_active)
    rover.main_loop();
    rover_timestamps = [rover_timestamps ; rover.system_time];
    rover_poses = [rover_poses ; copy(rover.localization.robot_pose)];
    rover_vel_twists = [rover_vel_twists ; copy(rover.odometry.odom_vel)];
end

disp("Mission done!");

rover_timestamps = rover_timestamps - rover_timestamps(1);
rover_pose_xytheta = [];
rover_vel_vw = [];
for idx=1:size(rover_timestamps)
    current_pose = rover_poses(idx);
    current_vel = rover_vel_twists(idx);
    rover_pose_xytheta = [rover_pose_xytheta ; current_pose.translation.val_x current_pose.translation.val_y current_pose.rotation.value_radians];
    rover_vel_vw = [rover_vel_vw ; current_vel.dx current_vel.dtheta];
end

waypoint_xypts = [];

for idx=1:size(nav_pts,1)
    waypoint_xypts = [waypoint_xypts; [nav_pts(idx).translation.val_x nav_pts(idx).translation.val_y]];
end

windowSize = 10; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;


figure;
plot(rover_pose_xytheta(:,1),rover_pose_xytheta(:,2));
hold on;
axis equal;
xlabel("x (m)");
ylabel("y (m)");
title("Rover odometry log");
scatter(waypoint_xypts(:,1), waypoint_xypts(:,2));
hold off;

figure;
plot(rover_timestamps,rover_pose_xytheta(:,3));
hold on;
axis equal;
xlabel("t (s)");
ylabel("Θ (rad)");
title("Rover angle log");
hold off;


figure;
subplot(2,1,1);
plot(rover_timestamps, filter(b,a,rover_vel_vw(:,1)));
xlabel("t (s)");
ylabel("v (m/s)");
title("Rover velocity vs time");
subplot(2,1,2);
plot(rover_timestamps, rover_vel_vw(:,2));
xlabel("t (s)");
ylabel("w (rad/s)");
title("Rover angular velocity vs time");


%if ~rover.mission_active
%    rover.delete(); %WOO Safety
%end
