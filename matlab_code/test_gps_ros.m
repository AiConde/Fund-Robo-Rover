clear;


gps = GPS_ROS();

pause(1)

%pp = quiver3(0,0,0,0,0,0);

while (1)

    [lat, long, alt, lat_covariance, long_covariance, alt_covariance] = ...
        gps.get_fix_data();

    [accel_xyz, gyro_xyz] = gps.get_imu_output();

    vec = normalize(accel_xyz);

    
    geoplot(lat, long, '-*', "MarkerSize", 20);
    hold on;
    geobasemap streets;
    text(0, 1, "GPS Location", "FontSize", 26);
    hold off;

    %pp = quiver3(0, 0, 0, vec(1), vec(2), vec(3));
    %pause(0.1);
end


