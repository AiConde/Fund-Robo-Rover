#include "typedefs.h"
#include "pindef.h"
#include <SerialTransfer.h>

namespace serial_comms {

typedef struct {
  float esc_pwm_setpoint = 0.5f;
  float pan_servo_setpoint = 0.5f;
  float steer_servo_setpoint = 0.5f;
  bool sys_reset = false;
  bool transfer_success = false;
} cmd_frame_t;

typedef struct {
  float accel_x = 0.f, accel_y = 0.f, accel_z = 0.f;
  float gyro_x = 0.f, gyro_y = 0.f, gyro_z = 0.f;
  float mag_x = 0.f, mag_y = 0.f, mag_z = 0.f;
  uint32_t us_since_lsm9_read = 0;

  float sharpir1_voltage = 0.f, sharpir2_voltage = 0.f, sharpir3_voltage = 0.f, sharpir4_voltage = 0.f, sharpir5_voltage = 0.f, sharpir6_voltage = 0.f;
  uint32_t us_since_sharpir_read = 0;

  float sonar1_voltage = 0.f, sonar2_voltage = 0.f;
  uint32_t us_since_sonar_read = 0;

  uint32_t tacometer_count = 0;
  uint32_t us_since_tacometer_read = 0;
} status_frame_t;

void init();

bool rx_available();
void txrx_data(cmd_frame_t *cmd_rx, status_frame_t *status_tx);
}
