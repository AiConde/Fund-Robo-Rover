#pragma once

#include "typedefs.h"

#define FILT_A_Fc50Hz_Ts1kHz ((float) 0.7386)
#define FILT_B_Fc50Hz_Ts1kHz ((float) 0.02614)
#define FILT_C_Fc50Hz_Ts1kHz ((float) 10.0)

#define FILT_A_Fc20Hz_Ts100Hz ((float) 0.2982)
#define FILT_B_Fc20Hz_Ts100Hz ((float) 0.07018)
#define FILT_C_Fc20Hz_Ts100Hz ((float) 10.0)


class digital_filter {
  private:
    float _internal_state;
    float _filt_A;
    float _filt_B;
    float _filt_C;
    bool _first_run;
    bool _preheat;
  public:
    void reset();
    digital_filter(float A, float B, float C, bool preheat);
    void update(float new_reading);
    float get_output();
};
