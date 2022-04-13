#include "sonar.h"
#include "digital_filter.h"
/*
  typedef struct {
  float SONAR1_V;
  float SONAR2_V;
  } sonar_reading_t;
*/

namespace sonar {

// 100Hz filter update, 20Hz low pass cutoff frequency
static digital_filter filt_sonar1(FILT_A_Fc20Hz_Ts100Hz, FILT_B_Fc20Hz_Ts100Hz, FILT_C_Fc20Hz_Ts100Hz, false);
static digital_filter filt_sonar2(FILT_A_Fc20Hz_Ts100Hz, FILT_B_Fc20Hz_Ts100Hz, FILT_C_Fc20Hz_Ts100Hz, false);
static uint32_t last_read_timestamp_micros;

void init() {
  analogReadResolution(12);
  last_read_timestamp_micros = get_time_micros_safe();
}
void read_loop_100hz() {
  uint16_t sonar1_reading_u16 = analogRead(SONAR1_PIN);
  uint16_t sonar2_reading_u16 = analogRead(SONAR2_PIN);
  float sonar1_reading_float = ((float) sonar1_reading_u16) * ADC_AREF_VOLTAGE / 4095;
  float sonar2_reading_float = ((float) sonar2_reading_u16) * ADC_AREF_VOLTAGE / 4095;
  filt_sonar1.update(sonar1_reading_float);
  filt_sonar2.update(sonar2_reading_float);
  last_read_timestamp_micros = get_time_micros_safe();
}
void get_filtered_readings(sonar_reading_t *readings_out) {
  readings_out->micros_since_read = get_period_since_micros_safe(last_read_timestamp_micros);
  readings_out->sonar1_voltage = filt_sonar1.get_output();
  readings_out->sonar2_voltage = filt_sonar2.get_output();
}
void deinit() {
  filt_sonar1.reset();
  filt_sonar2.reset();
}
}
