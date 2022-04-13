classdef Lidar_ROS < handle
    % Lidar class object. Represents a Hokuyo lidar we access over ROS

    properties
        %% class properties
        last_scan_angles % Last set of scanned angles we got
    end % End class properties

    properties (Dependent)
        %% getter properties
    end % End getter properties

    methods
        %% getter methods
    end % End getter methods

    methods (Static)
        %% static methods
        function angles_filtered = filter_angles(angles)
            % TODO
        end

        function cartesian_points = polar_to_cartesian(angles) % should this take ranges, too?
            % TODO
        end
    end % End static methods

    properties  (Access = private)
        %% private class properties
        lidar_sub
        range_min
        range_max
        angle_min
        angle_max
        angle_increment
    end % End clas

    properties(Constant)
        %% Constant properties
    end

    methods
        %% class methods

        %% Class constructor
        function obj = Lidar(com_port)
            lidar_sub = rossubscriber("/scan", @obj.Callback_Laser, "DataFormat", "struct");
        end

        %% Class destructor
        function delete(obj)
            clear obj.lidar_sub;
        end

        function Callback_Laser(obj, sub, scandata)
            disp("Laser callback")
            obj.range_min = scandata.RangeMin;
            obj.range_max = scandata.RangeMax;
            obj.angle_min = scandata.AngleMin;
            obj.angle_max = scandata.AngleMax;
            obj.angle_increment = scandata.AngleIncrement;
            obj.last_scan_angles = scandata.Ranges;

        end

        function angles = lidar_scan(obj)
            obj.scandata = receive(obj.lidar_sub,1);
            obj.range_min = scandata.RangeMin;
            obj.range_max = scandata.RangeMax;
            obj.angle_min = scandata.AngleMin;
            obj.angle_max = scandata.AngleMax;
            obj.angle_increment = scandata.AngleIncrement;
            obj.last_scan_angles = scandata.Ranges;
        end


    end % End classmethods



end % End class

