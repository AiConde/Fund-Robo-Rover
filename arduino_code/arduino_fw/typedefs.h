#pragma once

#include <Arduino.h>

#define ERROR_HANDLER(msg) ({Serial.println(msg); while(1) {digitalWrite(LED_PIN, HIGH); delay(500); digitalWrite(LED_PIN, LOW); delay(500);}})

template <typename T>
static T math_max(T a, T b) {
  if (a > b)
    return a;
  else
    return b;
}

template <typename T>
static T math_min(T a, T b) {
  if (a < b)
    return a;
  else
    return b;
}

template <typename T>
static T math_clamp(T x, T low, T high) {
  return math_min<T>(high, math_max<T>(low, x));
}

// Overflow-proof implementation of current time in microseconds
static inline uint32_t get_time_micros_safe() {
  return micros();
}

// Overflow-proof implementation of time duration checking. Returns true if at least t_period microseconds has elapsed since t_start, returns false otherwise.
static inline bool period_elapsed_micros_safe(uint32_t t_start, uint32_t t_period) {
  return ((uint32_t) (micros() - t_start) > t_period);
}

// Overflow-proof implementation of time duration calculation. Returns number of microseconds elapsed since t_start.
static inline uint32_t get_period_since_micros_safe(uint32_t t_start) {
  return ((uint32_t) (micros() - t_start));
}

#define ADC_AREF_VOLTAGE ((float) 3.3)

template <typename T>
static T math_map(T x, T in_min, T in_max, T out_min, T out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
