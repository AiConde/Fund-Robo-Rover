classdef Lidar_ROS < handle
    % Lidar class object. Represents a Hokuyo lidar we access over ROS

    properties  (Access = private)
        %% private class properties
        lidar_sub

        laserscan_raw

        new_laser_data
    end % End clas

    methods
        %% class methods

        %% Class constructor
        function obj = Lidar_ROS(com_port)
            obj.lidar_sub = rossubscriber("/scan", @obj.Callback_Laser, "DataFormat", "struct");
            obj.new_laser_data = false;
        end

        %% Class destructor
        function delete(obj)
            clear obj.lidar_sub;
        end

        function Callback_Laser(obj, sub, scandata)
            obj.laserscan_raw = scandata;
            obj.new_laser_data = true;
        end

        function [angles, ranges] = get_scandata(obj)
            if (obj.new_laser_data)
                obj.new_laser_data = false;
            end
            angles = rosReadScanAngles(obj.laserscan_raw);
            ranges = obj.laserscan_raw.Ranges;
        end

        function laserScan = get_scandata_raw(obj)
            if (obj.new_laser_data)
                obj.new_laser_data = false;
            end
            laserScan = obj.laserscan_raw;
        end
        
        function is_new_data = is_new_laserscan_available(obj) 
            is_new_data = obj.new_laser_data;
        end

        function [ranges_min, ranges_max, angle_min, angle_max, angle_increment] = get_scanparams(obj)
            ranges_min = obj.laserscan_raw.RangeMin;
            ranges_max = obj.laserscan_raw.RangeMax;
            angle_min = obj.laserscan_raw.AngleMin;
            angle_max = obj.laserscan_raw.AngleMax;
            angle_increment = obj.laserscan_raw.AngleIncrement;
        end


       function angles_filtered = filter_angles(obj, angles)
            % TODO
        end

        function cartesian_points = polar_to_cartesian(obj, angles) % should this take ranges, too?
            % TODO
        end



    end % End classmethods



end % End class

