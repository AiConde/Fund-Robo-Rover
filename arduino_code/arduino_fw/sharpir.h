#pragma once

#include "pindef.h"
#include "typedefs.h"

namespace sharpir {

    typedef struct {
        float sharpir1_voltage;
        float sharpir2_voltage;
        float sharpir3_voltage;
        float sharpir4_voltage;
        float sharpir5_voltage;
        float sharpir6_voltage;
        uint32_t micros_since_read;
    } sharpir_reading_t;

    void init();
    void read_loop_100hz();
    void get_filtered_readings(sharpir_reading_t *readings_out);
    void deinit();
}
