clc;
clear;
init_matlab(); % start ROS server and set matlab path

rover = Rover(false); %Initialize rover without joystick

oval_nav_pts = [
  %  Pose2d.from_xydeg(    4.1866 , -4.4913, 0);
%Pose2d.from_xydeg(    0.0391 , -0.0284, 0);
%Pose2d.from_xydeg(   -2.7259 ,  5.8119, 0);
Pose2d.from_xydeg(   -3.3895 , 12.5338, 0);
Pose2d.from_xydeg(   -1.8964 , 19.2005, 0);
Pose2d.from_xydeg(    0.5921 , 26.1978, 0);
Pose2d.from_xydeg(    3.9101 , 32.6993, 0);
Pose2d.from_xydeg(    8.0576 , 38.8150, 0);
Pose2d.from_xydeg(   12.9793 , 44.2697, 0);
Pose2d.from_xydeg(   18.3987 , 49.2284, 0);
Pose2d.from_xydeg(   24.5370 , 53.2505, 0);
Pose2d.from_xydeg(   31.0071 , 56.7216, 0);
Pose2d.from_xydeg(   37.9748 , 59.3112, 0);
Pose2d.from_xydeg(   45.0532 , 60.9641, 0);
Pose2d.from_xydeg(   53.3482 , 60.7989, 0);
Pose2d.from_xydeg(   60.7031 , 57.2177, 0);
Pose2d.from_xydeg(   65.6802 , 52.5896, 0);
Pose2d.from_xydeg(   67.7263 , 47.0799, 0);
Pose2d.from_xydeg(   68.6112 , 40.0826, 0);
Pose2d.from_xydeg(   67.2288 , 33.1404, 0);
Pose2d.from_xydeg(   56.0030 , 12.2034, 0);
Pose2d.from_xydeg(   49.4776 ,  5.6468, 0);
Pose2d.from_xydeg(   43.6711 ,  1.1839, 0);
Pose2d.from_xydeg(   37.4222 , -2.7832, 0);
Pose2d.from_xydeg(   30.6755 , -5.5381, 0);
Pose2d.from_xydeg(   23.6524 , -7.6318, 0);
Pose2d.from_xydeg(   16.8504 , -8.6786, 0);
Pose2d.from_xydeg(    4.5184 , -4.6566, 0);
Pose2d.from_xydeg(   -0.0715 ,  0.0267, 0);
Pose2d.from_xydeg(   -7.189516129032258 ,  -1.7265016685205783, 0);
];

figure; hold on; axis equal;

inner_nav_pts = [
        %Pose2d.from_xydeg(-10.796370967741934,8.540114015572858,95);
        %Pose2d.from_xydeg(-12.796370967741934,10.540114015572858,95);
        Pose2d.from_xydeg(-11.34,13.00,0);
Pose2d.from_xydeg(-8.71 ,16.99,0);
Pose2d.from_xydeg(-7.36 ,20.89,0);
Pose2d.from_xydeg(-5.90 ,24.72,0);
Pose2d.from_xydeg(-5.04 ,28.54,0);
Pose2d.from_xydeg(-2.92 ,32.08,0);
Pose2d.from_xydeg(-1.33 ,35.92,0);
Pose2d.from_xydeg(0.94  ,39.20,0);
Pose2d.from_xydeg(3.07  ,42.70,0);
Pose2d.from_xydeg(5.93  ,45.62,0);
Pose2d.from_xydeg(9.07  ,48.24,0);
Pose2d.from_xydeg(11.73 ,51.17,0);
Pose2d.from_xydeg(14.71 ,53.85,0);
Pose2d.from_xydeg(18.23 ,55.95,0);
Pose2d.from_xydeg(21.48 ,58.40,0);
Pose2d.from_xydeg(25.19-.3 ,60.09+.3,0);
Pose2d.from_xydeg(28.76-.3 ,61.88+.3,0);
Pose2d.from_xydeg(32.38-.3 ,63.76+.3,0);
%Pose2d.from_xydeg(32.149819243604 ,64.45661846496107,0); % Gate
Pose2d.from_xydeg(36.14-.3 ,65.07+.3,0);
Pose2d.from_xydeg(40.19-0.3 ,66.16+0.3,0);
Pose2d.from_xydeg(43.94-0.3 ,66.98+0.3,0);
%Pose2d.from_xydeg(48.25 ,67.45,0);
    ];

waypoints = zeros(length(oval_nav_pts), 2);
for i=1:length(oval_nav_pts)
%     % plot pose i
%     Utils.quiver_pose(oval_nav_pts(i));
    navptx = oval_nav_pts(i).translation.val_x;
    navpty = oval_nav_pts(i).translation.val_y;
    waypoints(i,:) = [navptx, navpty];
end
plot(waypoints(:,1), waypoints(:,2), "-o");

% inner_nav_pts = [
%         %Pose2d.from_xydeg(-10.796370967741934,8.540114015572858,95);
%         %Pose2d.from_xydeg(-12.796370967741934,10.540114015572858,95);
%         Pose2d.from_xydeg(-11.34,13.00,0);
% Pose2d.from_xydeg(-8.71 ,16.99,0);
% Pose2d.from_xydeg(-7.36 ,20.89,0);
% Pose2d.from_xydeg(-5.90 ,24.72,0);
% Pose2d.from_xydeg(-5.04 ,28.54,0);
% Pose2d.from_xydeg(-2.92 ,32.08,0);
% Pose2d.from_xydeg(-1.33 ,35.92,0);
% Pose2d.from_xydeg(0.94  ,39.20,0);
% Pose2d.from_xydeg(3.07  ,42.70,0);
% Pose2d.from_xydeg(5.93  ,45.62,0);
% Pose2d.from_xydeg(9.07  ,48.24,0);
% Pose2d.from_xydeg(11.73 ,51.17,0);
% Pose2d.from_xydeg(14.71 ,53.85,0);
% Pose2d.from_xydeg(18.23 ,55.95,0);
% Pose2d.from_xydeg(21.48 ,58.40,0);
% Pose2d.from_xydeg(25.19 ,60.09,0);
% Pose2d.from_xydeg(28.76 ,61.88,0);
% Pose2d.from_xydeg(32.38 ,63.76,0);
% %Pose2d.from_xydeg(32.149819243604 ,64.45661846496107,0); % Gate
% Pose2d.from_xydeg(36.14 ,65.07,0);
% Pose2d.from_xydeg(40.19 ,66.16,0);
% Pose2d.from_xydeg(43.94 ,66.98,0);
% %Pose2d.from_xydeg(48.25 ,67.45,0);
%     ];
% 
nav_pts_firsthalf = copy(oval_nav_pts(1:13));
nav_pts_secondhalf = copy(oval_nav_pts(14:end));
% 
% nav_pts = copy(inner_nav_pts);
% 
% define list of rover commands to run
rover.set_mission_command_list({
    CalibrateGyroCommand(rover);
    %SimpleAprilTagLocalize(rover);
    HoldHorses(rover,1);
    %WaypointNavCommand(rover, nav_pts_firsthalf, 0.5, 240, 2.2);
    %WaypointNavCommand(rover, nav_pts_secondhalf, 0.5, 240, 2.2);
    WaypointNavCommand(rover, nav_pts, 1, 240, 2.2);
    HoldHorses(rover, 5);
    
})


%{
    0.0637    0.0701
   -2.7011    5.8789
   -3.4825   12.5261
%}

%{
nav_pts = [
    Pose2d.from_xydeg(0,0,0);
    Pose2d.from_xydeg(-2.7,5.87,0);
    Pose2d.from_xydeg(-3.48,12.52,0);
    Pose2d.from_xydeg(-1.91,19.17,0);
    ];

%}

%nav_pts = [
%    Pose2d.from_xydeg(0,0,135);
%    Pose2d.from_xydeg(-5, 5, 135);
%    Pose2d.from_xydeg(-10, 10, 135);
%    ];

%{
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

%}

%{
rover.set_mission_command_list({
    CalibrateGyroCommand(rover);
    %HoldHorses(rover, 1);
    %CalibrateGyroCommand(rover);

    WaypointNavCommand(rover, nav_pts, 0.5, 240, 2.2);
    HoldHorses(rover, 3);
    });
%}

start_pose_inner = Pose2d.from_xydeg(-10.796370967741934,8.540114015572858,94.3);
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
