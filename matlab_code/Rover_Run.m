clc;
clear;
init_matlab(); % start ROS server and set matlab path

rover = Rover(false); %Initialize rover without joystick

rover.write_localization(Pose2d.from_xydeg(0,0,0));

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

rover.robot_init();

while (rover.mission_active) 
    rover.main_loop();
end

% rover.mission_end()

disp("Finished with Rover_Run")