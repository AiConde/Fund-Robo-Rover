#include "digital_filter.h"

void digital_filter::reset() {
  _first_run = true;
  _internal_state = 0.0;
}

digital_filter::digital_filter(float A, float B, float C, bool preheat) {
  _preheat = preheat;
  _first_run = true;
  _filt_A = A;
  _filt_B = B;
  _filt_C = C;
}
void digital_filter::update(float new_reading) {
  if (_first_run) {
    _first_run = false;
    _internal_state = 0.0;
    if (_preheat) {
      _internal_state = new_reading/_filt_C;
      return;
    }
  }
  
  _internal_state = _filt_A * _internal_state + _filt_B * new_reading;
}

float digital_filter::get_output() {
  return _internal_state * _filt_C;
}
