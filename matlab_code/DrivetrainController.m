classdef DrivetrainController < handle
    % Closed-loop controller for drivetrain. Not closed loop right now,
    % maybe will be in the future.

    properties (Constant)
        lin_vel_max = 10;
        lin_vel_min = 0;
    end

    properties (Access = private)
        lin_vel_setpoint;
        ang_vel_setpoint;
        last_esc_pwm;
        last_steer_pwm;
    end

    methods
        function obj = DrivetrainController()
            obj.lin_vel_setpoint = 0;
            obj.ang_vel_setpoint = 0;
            obj.last_esc_pwm = 0;
            obj.last_steer_pwm = 0;
        end

        function set_vel_setpoints(obj, lin_vel, ang_vel)
            obj.lin_vel_setpoint = Utils.clamp(lin_vel, obj.lin_vel_min, obj.lin_vel_max);
            obj.ang_vel_setpoint = ang_vel;
        end

        % Update closed loop controller
        function update(obj, tacometer, gyro_angvel, system_time)
        end

        function [esc_pwm, steer_pwm] = get_pwm_outputs(obj)
            esc_pwm = obj.lin_vel_to_esc_pwm_openloop();
            steer_pwm = obj.ang_vel_to_steer_pwm_openloop();
            obj.last_esc_pwm = esc_pwm;
            obj.last_steer_pwm = steer_pwm;
        end
    end

    methods (Access = private)
        function esc_pwm = lin_vel_to_esc_pwm_openloop(obj)
            esc_pwm = Utils.map(obj.lin_vel_setpoint, 0, 10, 0.5, 0.6);
        end
        function steer_pwm = ang_vel_to_steer_pwm_openloop(obj)
            % Angular velocity is lin_vel * tand(steer_angle_deg) / Odometry.WHEELBASE_METERS
            % steer angle degrees is 180/pi * ArcTan(ang_vel * wheelbase / velocity)
            if (abs(obj.lin_vel_setpoint) > 1e-6)
                steer_angle = 180/pi * atan(obj.ang_vel_setpoint * Odometry.WHEELBASE_METERS / obj.lin_vel_setpoint);
                steer_pwm = Arduino_ROS.steer_angle_to_servo(steer_angle);
            else
                steer_pwm = obj.last_steer_pwm;
            end
        end
    end
end

