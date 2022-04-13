#pragma once

#include "pindef.h"

namespace led {

static inline void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

static inline void set(bool state) {
  digitalWrite(LED_PIN, state ? HIGH : LOW);
}

static inline void toggle() {
  digitalWrite(LED_PIN, digitalRead(LED_PIN) == HIGH ? LOW : HIGH);
}

}
