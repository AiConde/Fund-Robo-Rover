classdef GPS_ROS < handle
    % GPS class object. Represents the UBlox GPS accessed over a ROS link.
    
    % /ublox_gps/fix sensor_msgs/NavSatFix
    % /ublox_gps/fix_velocity geometry_msgs/TwistWithCovarianceStamped
    % /ublox_gps/imu_meas sensor_msgs/Imu
    % /ublox_gps/hnrpvt ublox_msgs/HnrPVT
    % /ublox_gps/navpvt ublox_msgs/NavPVT
    %
    
    properties
        %% class properties
    end % End class properties

    properties (Dependent)
        %% getter properties
    end % End getter properties

    properties  (Access = private)
        %% private properties
        fix_sub
        fix_vel_sub
        imu_sub
        hnr_posvel_pub
        nav_posvel_pub

        imu_output_msg
        fix_output_msg

        new_imu_data
        new_fix_data
    end % End getter properties

    methods
        %% getter methods
    end % End getter methods

    methods (Static)
        %% static methods
    end % End static methods

    
    methods
        %% class methods

        %% Class constructor
        function obj = GPS_ROS()
            obj.fix_sub = rossubscriber("/ublox_gps/fix", @obj.Callback_Fix, "DataFormat", "struct");
            obj.imu_sub = rossubscriber("/ublox_gps/imu_meas", @obj.Callback_Imu, "DataFormat", "struct");
            obj.new_imu_data = false;
            obj.new_fix_data = false;
        end

        function delete(obj)
            clear obj.fix_sub;
            clear obj.imu_sub;
        end

        function Callback_Imu(obj, sub, imudata)
            obj.imu_output_msg = imudata;
        end
        function Callback_Fix(obj, sub, fixdata)
            obj.fix_output_msg = fixdata;
        end

        function [lat, long, alt, lat_covariance, long_covariance, alt_covariance] = get_fix_data(obj)
            if (obj.new_fix_data)
                obj.new_fix_data = false;
            end
            lat = obj.fix_output_msg.Latitude;
            long = obj.fix_output_msg.Longitude;
            alt = obj.fix_output_msg.Altitude;
            lat_covariance= obj.fix_output_msg.PositionCovariance(1);
            long_covariance= obj.fix_output_msg.PositionCovariance(5);
            alt_covariance= obj.fix_output_msg.PositionCovariance(9);
        end

        function [accel_xyz, gyro_xyz] = get_imu_output(obj) 
            if (obj.new_imu_data)
                obj.new_imu_data = false;
            end
            accel_x = obj.imu_output_msg.LinearAcceleration.X;
            accel_y = obj.imu_output_msg.LinearAcceleration.Y;
            accel_z = obj.imu_output_msg.LinearAcceleration.Z;
            gyro_x = obj.imu_output_msg.AngularVelocity.X;
            gyro_y = obj.imu_output_msg.AngularVelocity.Y;
            gyro_z = obj.imu_output_msg.AngularVelocity.Z;
            accel_xyz = [accel_x accel_y accel_z];
            gyro_xyz = [gyro_x gyro_y gyro_z]; 
        end

        function is_new_data = is_new_fix_data_available(obj) 
            is_new_data = obj.new_fix_data;
        end
        function is_new_data = is_new_imu_data_available(obj) 
            is_new_data = obj.new_imu_data;
        end


    end % End classmethods

end % End class

