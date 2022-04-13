#pragma once

#include "pindef.h"
#include "typedefs.h"

namespace servos {
    typedef enum {
        ESC_PWM, SERVO_PAN, SERVO_STEER
    } servos_t;

    void init();
    void servos_write(float value, servos_t servo);
    void deinit();
}
