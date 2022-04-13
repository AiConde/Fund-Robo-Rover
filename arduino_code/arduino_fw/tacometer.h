#include "pindef.h"
#include "typedefs.h"

namespace tacometer {

    typedef struct {
        uint32_t tacometer_count;
        uint32_t micros_since_read;
    } tacometer_readings_t;

    void init();
    //void read_tacometer_1khz();
    void get_count(tacometer_readings_t *readings_out);
    void deinit();
}
