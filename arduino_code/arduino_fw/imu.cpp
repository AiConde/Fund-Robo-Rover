#include "digital_filter.h"
#include "imu.h"
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include "led.h"

namespace imu {

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(&Wire1, 0);

static imu_reading_t last_imu_reading;
static uint32_t last_read_timestamp_micros;

void init() {
  /*
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  */
  if (!lsm.begin()) {
    while (1) {
    Serial.println("Failed to init LSM9DS1");
    led::toggle();
    delay(500);
    led::toggle();
    delay(500);
    }
  }
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

  last_read_timestamp_micros = get_time_micros_safe();
}



void read_loop_1khz() {
  last_read_timestamp_micros = get_time_micros_safe();
  
  lsm.read();

  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 
  
  last_imu_reading.accel_x = a.acceleration.x;
  last_imu_reading.accel_y = a.acceleration.y;
  last_imu_reading.accel_z = a.acceleration.z;

  last_imu_reading.gyro_x = g.gyro.x;
  last_imu_reading.gyro_y = g.gyro.y;
  last_imu_reading.gyro_z = g.gyro.z;

  last_imu_reading.mag_x = m.magnetic.x;
  last_imu_reading.mag_y = m.magnetic.y;
  last_imu_reading.mag_z = m.magnetic.z;

  last_imu_reading.micros_since_read = 0;
}

void get_readings_filtered(imu_reading_t *readings_out) {
}

void get_readings_raw(imu_reading_t *readings_out) {
  *readings_out = last_imu_reading;
  readings_out->micros_since_read = get_period_since_micros_safe(last_read_timestamp_micros);
}


void deinit() {
}
}
