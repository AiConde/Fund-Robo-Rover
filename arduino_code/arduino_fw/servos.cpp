#include "servos.h"
#include <Servo.h>

namespace servos {

    Servo esc_pwm_servo;
    Servo pan_servo;
    Servo steer_servo;

    void init() {
        esc_pwm_servo.attach(ESC_PWM_PIN);
        pan_servo.attach(PAN_SERVO_PIN);
        steer_servo.attach(STEER_SERVO_PIN);
    }

    void servos_write(float value, servos_t servo) {
        uint32_t pwm_us_min;
        uint32_t pwm_us_max;
        switch(servo) {
            case ESC_PWM:
                pwm_us_min = ESC_PWM_MIN_PULSE;
                pwm_us_max = ESC_PWM_MAX_PULSE;
                break;
            case SERVO_PAN:
                pwm_us_min = PAN_SERVO_MIN_PULSE;
                pwm_us_max = PAN_SERVO_MAX_PULSE;
                break;
            case SERVO_STEER:
                pwm_us_min = STEER_SERVO_MIN_PULSE;
                pwm_us_max = STEER_SERVO_MAX_PULSE;
                break;
            default:
                pwm_us_min = 1000;
                pwm_us_max = 2000;

        }

        float value_clamped = math_clamp<float>(value, 0.0f, 1.0f);

        float out_us_f32 = math_map<float>(value, 0.0f, 1.0f, (float) pwm_us_min, (float) pwm_us_max);
        uint32_t out_us_u32 = (uint32_t) roundf(out_us_f32);
        uint32_t servo_pwm_us = math_clamp<uint32_t>(out_us_f32, pwm_us_min, pwm_us_max);

        switch(servo) {
            case ESC_PWM:
                esc_pwm_servo.writeMicroseconds(servo_pwm_us);
                break;
            case SERVO_PAN:
                pan_servo.writeMicroseconds(servo_pwm_us);
                break;
            case SERVO_STEER:
                steer_servo.writeMicroseconds(servo_pwm_us);
                break;
            default: break;
        }
    }

    void deinit() {
        esc_pwm_servo.detach();
        pan_servo.detach();
        steer_servo.detach();
    }
}
