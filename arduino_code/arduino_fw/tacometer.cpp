#include "tacometer.h"

//#define ENCODER_OPTIMIZE_INTERRUPTS
//#include <Encoder.h>
extern "C" {
  static volatile uint32_t tacometer_count;
  static volatile uint32_t last_read_timestamp_micros;

  void tacometer_isr_fn() {
    tacometer_count++;
    last_read_timestamp_micros = get_time_micros_safe();
  }
}

namespace tacometer {



//Encoder tacometer_encoder(TACOMETER_INPUT_PIN, TACOMETER_DUMMY_PIN);


void init() {
  tacometer_count = 0;
  last_read_timestamp_micros = get_time_micros_safe();

  pinMode(TACOMETER_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACOMETER_INPUT_PIN), tacometer_isr_fn, CHANGE);
}

void get_count(tacometer_readings_t *readings_out) {
  readings_out->tacometer_count = tacometer_count;
  readings_out->micros_since_read = get_period_since_micros_safe(last_read_timestamp_micros);
}

void deinit() {
  tacometer_count = 0;
  detachInterrupt(digitalPinToInterrupt(TACOMETER_INPUT_PIN));
}
}
