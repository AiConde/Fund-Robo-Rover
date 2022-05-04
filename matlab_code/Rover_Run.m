clc;
clear;

rover = Rover(false); %Initialize rover without joystick

rover.write_localization(Pose2d.from_xydeg(0,0,0));

rover.set_mission_command_list([ ...
    CalibrateGyroCommand(rover); ...
    %Odometry is updated in the rover class main loop don't @ me for
    %overcommenting :^)
    %RegisterStartPose(rover); is taken care of in Rover.init method
    WaypointNavCommand(rover, [
            Pose2d.fromxydeg(0,0,0);
            Pose2d.fromxydeg(5,0,0);
        ], 0.5, 10, 3); ...
]);

rover.robot_init();

while (rover.mission_active) 
    rover.main_loop();
end

if ~rover.mission_active
    rover.delete(); %WOO Safety
end