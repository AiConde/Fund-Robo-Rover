#include "imu.h"
#include "typedefs.h"
#include "pindef.h"
#include "led.h"
#include "serial_comms.h"
#include "servos.h"
#include "sharpir.h"
#include "sonar.h"
#include "tacometer.h"

#include <ros.h>
#include <ros/time.h>

#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

void esc_servo_callback(const std_msgs::Float32 &cmd_msg) {
  led::toggle();
  servos::servos_write(cmd_msg.data, servos::ESC_PWM);
}
void pan_servo_callback(const std_msgs::Float32 &cmd_msg) {
  servos::servos_write(cmd_msg.data, servos::SERVO_PAN);
}
void steer_servo_callback(const std_msgs::Float32 &cmd_msg) {
  servos::servos_write(cmd_msg.data, servos::SERVO_STEER);
}

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Float32> esc_servo_sub("/arduino_cmd/throttle", esc_servo_callback);
ros::Subscriber<std_msgs::Float32> steer_servo_sub("/arduino_cmd/steer", steer_servo_callback);
ros::Subscriber<std_msgs::Float32> pan_servo_sub("/arduino_cmd/pan", pan_servo_callback);

sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
std_msgs::Float32MultiArray ir_msg;
std_msgs::Float32MultiArray sonar_msg;
std_msgs::UInt32 tacometer_count;


#define IR_MSG_BUFFER_SIZE 6
#define SONAR_MSG_BUFFER_SIZE 2
float ir_msg_buffer[IR_MSG_BUFFER_SIZE];
float sonar_msg_buffer[SONAR_MSG_BUFFER_SIZE];

ros::Publisher pub_imu("/arduino_data/imu", &imu_msg);
ros::Publisher pub_mag("/arduino_data/magnetometer", &mag_msg);
ros::Publisher pub_ir("/arduino_data/ir_array", &ir_msg);
ros::Publisher pub_sonar("/arduino_data/sonar_array", &sonar_msg);
ros::Publisher pub_tacometer("/arduino_data/tacometer", &tacometer_count);


//ir_msg.data_length = 6;

uint32_t loop_start_us_100Hz;
const uint32_t loop_period_us_100Hz = 10000; // 10 ms = 100Hz

uint32_t loop_start_us_10Hz;
const uint32_t loop_period_us_10Hz = 100000; // 100 ms = 10Hz

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

  nh.initNode();

  for (uint8_t i = 0; i < IR_MSG_BUFFER_SIZE; i++)
    ir_msg_buffer[i] = 0.0f;
  for (uint8_t i = 0; i < SONAR_MSG_BUFFER_SIZE; i++)
    sonar_msg_buffer[i] = 0.0f;

  ir_msg.data = ir_msg_buffer;
  sonar_msg.data = sonar_msg_buffer;
  ir_msg.data_length = IR_MSG_BUFFER_SIZE;
  sonar_msg.data_length = SONAR_MSG_BUFFER_SIZE;

  tacometer_count.data = 0;

  for (uint8_t i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = 0.f;
    imu_msg.angular_velocity_covariance[i] = 0.f;
    imu_msg.linear_acceleration_covariance[i] = 0.f;
  }
  imu_msg.orientation_covariance[0] = -1.f;
  imu_msg.angular_velocity_covariance[0] = -1.f;
  imu_msg.linear_acceleration_covariance[0] = -1.f;

  imu_msg.orientation.x = -1.f;
  imu_msg.orientation.y = -1.f;
  imu_msg.orientation.z = -1.f;
  imu_msg.orientation.w = -1.f;

  imu_msg.angular_velocity.x = -1.f;
  imu_msg.angular_velocity.y = -1.f;
  imu_msg.angular_velocity.z = -1.f;

  imu_msg.linear_acceleration.x = -1.f;
  imu_msg.linear_acceleration.y = -1.f;
  imu_msg.linear_acceleration.z = -1.f;

  for (uint8_t i = 0; i < 9; i++) {
    mag_msg.magnetic_field_covariance[i] = 0.f;
  }
  mag_msg.magnetic_field.x = -1.f;
  mag_msg.magnetic_field.y = -1.f;
  mag_msg.magnetic_field.z = -1.f;

  loop_start_us_10Hz = get_time_micros_safe();
  loop_start_us_100Hz = loop_start_us_10Hz;

  nh.advertise(pub_imu);
  nh.advertise(pub_mag);
  nh.advertise(pub_sonar);
  nh.advertise(pub_ir);
  nh.advertise(pub_tacometer);

  nh.subscribe(esc_servo_sub);
  nh.subscribe(steer_servo_sub);
  nh.subscribe(pan_servo_sub);


}

void loop_100hz() {
  sharpir::read_loop_100hz();
  sonar::read_loop_100hz();
  imu::read_loop_1khz();



  imu::imu_reading_t imu_reading_buf;


  imu::get_readings_raw(&imu_reading_buf);


  // Convert deg/s to rad/s
  imu_msg.angular_velocity.x = imu_reading_buf.gyro_x * 0.017453f;
  imu_msg.angular_velocity.y = imu_reading_buf.gyro_y * 0.017453f;
  imu_msg.angular_velocity.z = imu_reading_buf.gyro_z * 0.017453f;

  // Convert gees to m/s
  imu_msg.linear_acceleration.x = imu_reading_buf.accel_x*9.8066f;
  imu_msg.linear_acceleration.y = imu_reading_buf.accel_y*9.8066f;
  imu_msg.linear_acceleration.z = imu_reading_buf.accel_z*9.8066f;

  // Convert gauss to tesla
  mag_msg.magnetic_field.x = imu_reading_buf.mag_x / 10000.f;
  mag_msg.magnetic_field.y = imu_reading_buf.mag_y / 10000.f;
  mag_msg.magnetic_field.z = imu_reading_buf.mag_z / 10000.f;

  tacometer::tacometer_readings_t taco_readings;
  tacometer::get_count(&taco_readings);
  tacometer_count.data = taco_readings.tacometer_count;


  pub_imu.publish(&imu_msg);
  pub_mag.publish(&mag_msg);

  pub_tacometer.publish(&tacometer_count);


}

void loop_10hz() {
  sharpir::sharpir_reading_t sharpir_reading_buf;
  sonar::sonar_reading_t sonar_reading_buf;
  sharpir::get_filtered_readings(&sharpir_reading_buf);
  sonar::get_filtered_readings(&sonar_reading_buf);
  ir_msg_buffer[0] = sharpir_reading_buf.sharpir1_voltage;
  ir_msg_buffer[1] = sharpir_reading_buf.sharpir2_voltage;
  ir_msg_buffer[2] = sharpir_reading_buf.sharpir3_voltage;
  ir_msg_buffer[3] = sharpir_reading_buf.sharpir4_voltage;
  ir_msg_buffer[4] = sharpir_reading_buf.sharpir5_voltage;
  ir_msg_buffer[5] = sharpir_reading_buf.sharpir6_voltage;

  sonar_msg_buffer[0] = sonar_reading_buf.sonar1_voltage;
  sonar_msg_buffer[1] = sonar_reading_buf.sonar2_voltage;
  pub_sonar.publish(&sonar_msg);
  pub_ir.publish(&ir_msg);
}

void loop_1khz() {

}

void loop() {
  if (period_elapsed_micros_safe(loop_start_us_100Hz, loop_period_us_100Hz)) {
    loop_start_us_100Hz = get_time_micros_safe();
    loop_100hz();
  }

  if (period_elapsed_micros_safe(loop_start_us_10Hz, loop_period_us_10Hz)) {
    loop_start_us_10Hz = get_time_micros_safe();
    loop_10hz();
  }

  nh.spinOnce();
}
