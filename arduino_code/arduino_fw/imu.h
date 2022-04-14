#include "pindef.h"
#include "typedefs.h"

namespace imu {

    typedef struct {
        float accel_x; // Accelerometer, in m/s^2
        float accel_y;
        float accel_z;
        float gyro_x; // Gyroscope, in rad/s
        float gyro_y;
        float gyro_z;
        float mag_x; // Magnetometer, in T
        float mag_y;
        float mag_z;
        uint32_t micros_since_read;
    } imu_reading_t;


    void init();
    void read_loop_1khz();
    void get_readings_filtered(imu_reading_t *readings_out);
    void get_readings_raw(imu_reading_t *readings_out);
    void deinit();
}
