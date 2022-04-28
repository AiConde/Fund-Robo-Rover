function init_ROS(varargin)
    %INIT_ROS Initialize global ROS node to connect to the NUC's ROS server
    %   Clears existing workspace variables.
    %   Init ROS global node with URI at [ros_uri]
    
    rosshutdown % Clear any existing ROS connection
    
    % Initialize ROS connection

    Defaults = {'http://192.168.16.71:11311/'};
%     Defaults = {'http://localhost:11311/'};
    Defaults(1:nargin) = varargin;

    ros_uri = Defaults{1};
    
    rosinit(ros_uri)

    disp("ROS initialization complete! :-)")
end

