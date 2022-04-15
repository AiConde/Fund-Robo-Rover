classdef Lidar_ROS < handle
    % Lidar class object. Represents a Hokuyo lidar we access over ROS

    properties
        %% class properties
    end % End class properties

    properties (Dependent)
        %% getter properties
    end % End getter properties

    methods
        %% getter methods
    end % End getter methods

    methods (Static)
        %% static methods
    end % End static methods

    properties  (Access = private)
        %% private class properties
        lidar_sub
        range_min
        range_max
        angle_min
        angle_max
        angle_increment
        ranges

        laserscan_raw
    end % End clas

    properties(Constant)
        %% Constant properties
    end

    methods
        %% class methods

        %% Class constructor
        function obj = Lidar_ROS(com_port)
            obj.lidar_sub = rossubscriber("/scan", @obj.Callback_Laser, "DataFormat", "struct");
        end

        %% Class destructor
        function delete(obj)
            clear obj.lidar_sub;
        end

        function Callback_Laser(obj, sub, scandata)
            obj.range_min = scandata.RangeMin;
            obj.range_max = scandata.RangeMax;
            obj.angle_min = scandata.AngleMin;
            obj.angle_max = scandata.AngleMax;
            obj.angle_increment = scandata.AngleIncrement;
            obj.ranges = scandata.Ranges;
            obj.laserscan_raw = scandata;

        end

        %{
        function angles = lidar_scan(obj)
            obj.ranges = receive(obj.lidar_sub,1);
            obj.range_min = scandata.RangeMin;
            obj.range_max = scandata.RangeMax;
            obj.angle_min = scandata.AngleMin;
            obj.angle_max = scandata.AngleMax;
            obj.angle_increment = scandata.AngleIncrement;
            obj.ranges = scandata.Ranges;
        end
        %}

        function [angles, ranges] = get_scandata(obj)
            angles = rosReadScanAngles(obj.laserscan_raw);
            ranges = obj.laserscan_raw.Ranges;
        end

        function laserScan = get_scandata_raw(obj)
            laserScan = obj.laserscan_raw;
        end

        function [ranges_min, ranges_max, angle_min, angle_max, angle_increment] = get_scanparams(obj)
            ranges_min = obj.range_min;
            ranges_max = obj.range_max;
            angle_min = obj.angle_min;
            angle_max = obj.angle_max;
            angle_increment = obj.angle_increment;
        end


       function angles_filtered = filter_angles(obj, angles)
            % TODO
        end

        function cartesian_points = polar_to_cartesian(obj, angles) % should this take ranges, too?
            % TODO
        end



    end % End classmethods



end % End class

