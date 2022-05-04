clc;
clear;
init_matlab(); % start ROS server and set matlab path

rover = Rover(false); %Initialize rover without joystick


% define list of rover commands to run
rover.set_mission_command_list({
    CalibrateGyroCommand(rover),...
    WaypointNavCommand(rover, [
        Pose2d.fromxydeg(0,0,0);
        Pose2d.fromxydeg(2,0,0);
    ]), ...
    HoldHorses(rover, 5),...
    SimpleAprilTagLocalize(rover)
})


%{
    0.0637    0.0701
   -2.7011    5.8789
   -3.4825   12.5261
%}

nav_pts = [
    Pose2d.from_xydeg(0,0,0);
    Pose2d.from_xydeg(-2.7,5.87,0);
    Pose2d.from_xydeg(-3.48,12.52,0);
    Pose2d.from_xydeg(-1.91,19.17,0);
    ];

%nav_pts = [
%    Pose2d.from_xydeg(0,0,135);
%    Pose2d.from_xydeg(-5, 5, 135);
%    Pose2d.from_xydeg(-10, 10, 135);
%    ];


nav_pts = [
      Pose2d.from_xydeg(4.4789,-4.7844, 0);
   Pose2d.from_xydeg(-2.9029,5.5121, 0);
   Pose2d.from_xydeg(-2.0909,18.9710, 0);
    Pose2d.from_xydeg(3.5930,32.4298, 0);
   Pose2d.from_xydeg(12.9678,44.1236, 0);
   Pose2d.from_xydeg(24.6308,53.1697, 0);
   Pose2d.from_xydeg(37.8439,59.2740, 0);
   Pose2d.from_xydeg(53.2716,60.8186, 0);
   Pose2d.from_xydeg(65.2299,52.6551, 0);
   Pose2d.from_xydeg(68.7732,40.2260, 0);
   Pose2d.from_xydeg(56.0263,12.2204, 0);
   Pose2d.from_xydeg(43.7844,1.2249, 0);
   Pose2d.from_xydeg(30.5775,-5.6848, 0);
   Pose2d.from_xydeg(16.8279,-8.8693, 0);
    Pose2d.from_xydeg(4.4652,-4.9038, 0);
    ];

rover.set_mission_command_list({
    CalibrateGyroCommand(rover);
    %HoldHorses(rover, 1);
    %CalibrateGyroCommand(rover);

    WaypointNavCommand(rover, nav_pts, 0.5, 240, 2.2);
    HoldHorses(rover, 3);
    });

start_pose = Pose2d.from_xydeg(0,0,126);
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
