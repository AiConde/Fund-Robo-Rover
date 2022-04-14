#include "imu.h"
#include "typedefs.h"
#include "pindef.h"
#include "led.h"
#include "serial_comms.h"
#include "servos.h"
#include "sharpir.h"
#include "sonar.h"
#include "tacometer.h"

/*
  typedef struct {
    float esc_pwm_setpoint;
    float pan_servo_setpoint;
    float steer_servo_setpoint;
    bool sys_reset;
    bool transfer_success;
  } cmd_frame_t;

     typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    uint32_t us_since_lsm9_read;

    float sharpir1_voltage, sharpir2_voltage, sharpir3_voltage, sharpir4_voltage, sharpir5_voltage, sharpir6_voltage;
    uint32_t us_since_sharpir_read;

    float sonar1_voltage, sonar2_voltage;
    uint32_t us_since_sonar_read;

    uint32_t tacometer_count;
    uint32_t us_since_tacometer_read;
  } status_frame_t;
*/
serial_comms::cmd_frame_t cmd_frame;
serial_comms::status_frame_t status_frame;

void setup() {
  analogReadResolution(12);

  led::setup();
  led::set(false);

  servos::init();
  sharpir::init();
  sonar::init();
  tacometer::init();
  imu::init();
  serial_comms::init();

  led::set(true);


}


void loop_100hz() {
  sharpir::read_loop_100hz();
  sonar::read_loop_100hz();
  imu::read_loop_1khz();

  if (serial_comms::rx_available()) {
    /*
    for (uint8_t i=0; i<10; i++) {
      led::toggle(); delay(100); led::toggle(); delay(100);  
    }
    */
    led::toggle();
    serial_comms::txrx_data(&cmd_frame, &status_frame);
    status_frame.accel_x = cmd_frame.esc_pwm_setpoint;
    status_frame.accel_y = cmd_frame.pan_servo_setpoint;
    status_frame.accel_z = cmd_frame.steer_servo_setpoint;
  }
}
void loop_1khz() {

}

uint32_t loop_start_us;
const uint32_t loop_period_us = 10000; // 10 ms = 100Hz

void loop() {
  loop_start_us = get_time_micros_safe();
  loop_100hz();
  while (!period_elapsed_micros_safe(loop_start_us, loop_period_us)) {
    delayMicroseconds(1);
  }
}
