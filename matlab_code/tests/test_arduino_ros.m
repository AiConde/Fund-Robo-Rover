arduino = Arduino_ROS();
%cam.config_resolution('1024x768');

% pause(1);
% 
% r = rateControl(5);
% 
% while (1) 
%     ir_voltages = arduino.get_ir_voltages()
%     sonar_voltages = arduino.get_sonar_voltages()
%     [accel_xyz, gyro_xyz] = arduino.get_imu_output();
%     tacometer_count = arduino.get_tacometer_output();
%     pause(0.1);
% end



imu_sub = rossubscriber("/arduino_data/imu", @callback_imu, "DataFormat", "struct")

[msg2,status,statustext] = receive(imu_sub,1)

function imu_output_msg = callback_imu(sub, imudata)
    imu_output_msg = imudata;
end