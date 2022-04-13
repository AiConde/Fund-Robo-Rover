#include "serial_comms.h"

namespace serial_comms {
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

static SerialTransfer serial_transfer;

void init() {
  Serial.begin(115200);
  serial_transfer.begin(Serial);
}

bool rx_available() {
  return serial_transfer.available();
}

void txrx_data(cmd_frame_t *cmd_rx, status_frame_t *status_tx) {
  uint16_t rec_size = 0;

  cmd_frame_t cmd_rx_local;
  status_frame_t status_frame_local = *status_tx;

  rec_size = serial_transfer.rxObj(cmd_rx_local, rec_size);
  
  uint16_t send_size = 0;
  send_size = serial_transfer.txObj(status_frame_local, send_size);
  serial_transfer.sendData(send_size);

  *cmd_rx = cmd_rx_local;  
}

}
