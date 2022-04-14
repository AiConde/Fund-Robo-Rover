#pragma once

#include "pindef.h"
#include "typedefs.h"


namespace sonar {

typedef struct {
  float sonar1_voltage;
  float sonar2_voltage;
  uint32_t micros_since_read;
} sonar_reading_t;

void init();
void read_loop_100hz();
void get_filtered_readings(sonar_reading_t *readings_out);
void deinit();
}
