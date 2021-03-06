classdef Arduino_ROS < handle
    % Arduino class object. Represents the Arduino connected over ROS

    properties  (Access = private)
        %% private class properties
       
        
        %% Cached messages
        ir_voltages_msg 
        sonar_voltages_msg
        tacometer_count_msg
        imu_output_msg
        mag_output_msg
        
        %% boolean flags to check if we've recieved new data
        new_ir_data
        new_sonar_data
        new_tacometer_data
        new_imu_data
        new_mag_data
        
        %% Subscriber objects
        imu_sub
        ir_array_sub
        sonar_array_sub
        tacometer_sub
        magnetometer_sub

        %% Publisher objects
        esc_pwm_pub
        steer_servo_pub
        pan_servo_pub

        tacometer_offset

        gyro_offset_xyz

    end % End clas

    properties
        last_esc_pwm
        last_steer_angle
        last_pan_angle
        last_steer_pwm
        last_pan_pwm
    end
    
    methods (Static)
        %% Takes in target steering angle in degrees, outputs the corresponding servo value
        function servo_pwm = steer_angle_to_servo(steer_angle)
            steer_angle_min = -30;
            steer_angle_max = 30;
            steer_angle_clamped = Utils.clamp(steer_angle, steer_angle_min, steer_angle_max);
            servo_pwm = Utils.map(steer_angle_clamped, steer_angle_min, steer_angle_max, 0.0, 1.0);
        end
        %% Takes in target steering angle in degrees, outputs the corresponding servo value
        function servo_pwm = servo_to_steer_angle(servo_pwm)
            steer_angle_min = -30;
            steer_angle_max = 30;
            servo_pwm = Utils.map(servo_pwm, 0, 1, steer_angle_min, steer_angle_max);
        end

        %% Takes in target pan angle in degrees, outputs the corresponding servo value
        function servo_pwm = pan_angle_to_servo(pan_angle)
            pan_angle_min = -150;
            pan_angle_max = 150;
            pan_angle_clamped = Utils.clamp(pan_angle, pan_angle_min, pan_angle_max);
            servo_pwm = Utils.map(pan_angle_clamped, pan_angle_min, pan_angle_max, 0.0, 1.0);
        end
    end

    methods
        %% class methods

        %% Class constructor
        function obj = Arduino_ROS()
            obj.imu_sub = rossubscriber("/arduino_data/imu", @obj.Callback_Imu, "DataFormat", "struct");
            obj.ir_array_sub = rossubscriber("/arduino_data/ir_array", @obj.Callback_IR, "DataFormat", "struct");
            obj.sonar_array_sub = rossubscriber("/arduino_data/sonar_array", @obj.Callback_Sonar, "DataFormat", "struct");
            obj.tacometer_sub = rossubscriber("/arduino_data/tacometer", @obj.Callback_Tacometer, "DataFormat", "struct");
            obj.magnetometer_sub = rossubscriber("/arduino_data/magnetometer", @obj.Callback_Magnetometer, "DataFormat", "struct");

            obj.esc_pwm_pub = rospublisher("/arduino_cmd/throttle", "std_msgs/Float32");
            obj.steer_servo_pub = rospublisher("/arduino_cmd/steer", "std_msgs/Float32");
            obj.pan_servo_pub = rospublisher("/arduino_cmd/pan", "std_msgs/Float32");

            obj.write_esc_pwm(0.5);
            obj.write_steer_servo(0.5);
            obj.write_pan_servo(0.5);

            obj.new_ir_data = false;
            obj.new_sonar_data = false;
            obj.new_tacometer_data = false;
            obj.new_imu_data = false;

            obj.tacometer_offset = 0;

            obj.gyro_offset_xyz = [0 0 0];
        end

        %% Class destructor
        function delete(obj)
            clear obj.imu_sub;
            clear obj.ir_array_sub;
            clear obj.sonar_array_sub;
            clear obj.tacometer_sub;

            clear obj.esc_pwm_pub;
            clear obj.steer_servo_pub;
            clear obj.pan_servo_pub;
        end

        %% callback methods

        function Callback_Imu(obj, sub, imudata)
            obj.new_imu_data = true;
            obj.imu_output_msg = imudata;
        end
        function Callback_Magnetometer(obj, sub, magdata)
            obj.new_mag_data = true;
            obj.mag_output_msg = magdata;
        end
        function Callback_IR(obj, sub, irdata)
            obj.new_ir_data = true;
            obj.ir_voltages_msg = irdata;
        end
        function Callback_Sonar(obj, sub, sonardata)
            obj.new_sonar_data = true;
            obj.sonar_voltages_msg = sonardata;
        end
        function Callback_Tacometer(obj, sub, tacometerdata)
            obj.new_tacometer_data = true;
            obj.tacometer_count_msg = tacometerdata;
        end

        %% Act methods

        function write_esc_pwm(obj, esc_pwm_value)  % pwm vlue 0-1?
            obj.last_esc_pwm = esc_pwm_value;
            pub_msg = rosmessage(obj.esc_pwm_pub);
            pub_msg.Data = esc_pwm_value;
            send(obj.esc_pwm_pub, pub_msg);
        end

        function write_steer_servo(obj, steer_servo_value) 
            obj.last_steer_pwm = steer_servo_value;
            pub_msg = rosmessage(obj.steer_servo_pub);
            pub_msg.Data = steer_servo_value;
            send(obj.steer_servo_pub, pub_msg);
        end

        function write_pan_servo(obj, pan_servo_value) 
            obj.last_pan_pwm = pan_servo_value;
            pub_msg = rosmessage(obj.pan_servo_pub);
            pub_msg.Data = pan_servo_value;
            send(obj.pan_servo_pub, pub_msg);
        end
        
        function write_steer_servo_angle(obj, steer_servo_angle) 
            obj.last_steer_angle = steer_servo_angle;
            pwm = Arduino_ROS.steer_angle_to_servo(steer_servo_angle);
            obj.write_steer_servo(pwm);
        end

        function write_pan_servo_angle(obj, pan_servo_angle) 
            obj.last_pan_angle = pan_servo_angle;
            pwm = Arduino_ROS.pan_angle_to_servo(pan_servo_angle);
            obj.write_pan_servo(pwm);
        end

        %% Sense methods (IR, Sonar, IMU, Magnetometer, Tachometer)

        function ir_array = get_ir_voltages(obj) 
            if (obj.new_ir_data)
                obj.new_ir_data = false;
            end
            ir_array = obj.ir_voltages_msg.Data;
        end

        function sonar_array = get_sonar_voltages(obj) 
            if (obj.new_sonar_data)
                obj.new_sonar_data = false;
            end
            sonar_array = obj.sonar_voltages_msg.Data;
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

        function mag_xyz = get_mag_output(obj) 
            if (obj.new_mag_data)
                obj.new_mag_data = false;
            end
            mag_x = obj.mag_output_msg.MagneticField_.X;
            mag_y = obj.mag_output_msg.MagneticField_.Y;
            mag_z = obj.mag_output_msg.MagneticField_.Z;
            mag_xyz = [mag_x, mag_y, mag_z];
        end

        function tacometer_count = get_tacometer_output(obj)
            if (obj.new_tacometer_data)
                obj.new_tacometer_data = false;
            end
            tacometer_count = obj.tacometer_count_msg.Data - obj.tacometer_offset;
        end

        %% calibration methods

        %{
        function [accel_offset, gyro_offset] = calibrate_imu(obj)
            num_readings = 100;

            % empty lists for collecting calibration readings
            accel_cal = zeros(num_readings, 3);
            gyro_cal  = zeros(num_readings, 3);
            mag_cal   = zeros(num_readings, 3);
            
            % collect a bunch of imu readings
            for i = 1:num_readings
                waitfor(obj.new_imu_data);
                [accel_reading, gyro_reading] = obj.get_imu_output();
                mag_reading = obj.get_mag_output();

                accel_cal(i, :) = accel_reading;
                mag_cal(i, :)   = mag_reading;
                gyro_cal(i, :)  = gyro_reading;
            end

            % get average accel offset
            obj.accel_offset = mean(accel_cal) - [0, 0, 9.80665];
            
            % get average gyro offset
            obj.gyro_offset = mean(gyro_cal);
        end
        %}

        function set_gyro_offset(obj, gyro_cal)
            obj.gyro_offset_xyz = gyro_cal;
        end

        function offset_xyz = get_gyro_offset(obj)
            offset_xyz = obj.gyro_offset_xyz;
        end

        function [accel_xyz, gyro_xyz] = get_imu_output_calibrated(obj) 
            [accel_xyz_raw, gyro_xyz_raw] = obj.get_imu_output();
            accel_xyz = accel_xyz_raw;
            gyro_xyz = gyro_xyz_raw - obj.gyro_offset_xyz;
        end


        %% new data methods

        function is_new_data = is_new_ir_data_available(obj) 
            is_new_data = obj.new_ir_data;
        end
        function is_new_data = is_new_sonar_data_available(obj) 
            is_new_data = obj.new_sonar_data;
        end
        function is_new_data = is_new_imu_data_available(obj) 
            is_new_data = obj.new_imu_data;
        end
        function is_new_data = is_new_mag_data_available(obj) 
            is_new_data = obj.new_mag_data;
        end
        function is_new_data = is_new_tacometer_data_available(obj) 
            is_new_data = obj.new_tacometer_data;
        end

        function zero_tacometer() 
            obj.tacometer_offset = obj.tacometer_count_msg.Data;
        end


    end % End classmethods



end % End class

