#include "sharpir.h"
#include "digital_filter.h"

namespace sharpir {

    // 100Hz filter update, 20Hz low pass cutoff frequency
    static digital_filter filt_sharpir1(FILT_A_Fc20Hz_Ts100Hz, FILT_B_Fc20Hz_Ts100Hz, FILT_C_Fc20Hz_Ts100Hz, false);
    static digital_filter filt_sharpir2(FILT_A_Fc20Hz_Ts100Hz, FILT_B_Fc20Hz_Ts100Hz, FILT_C_Fc20Hz_Ts100Hz, false);
    static digital_filter filt_sharpir3(FILT_A_Fc20Hz_Ts100Hz, FILT_B_Fc20Hz_Ts100Hz, FILT_C_Fc20Hz_Ts100Hz, false);
    static digital_filter filt_sharpir4(FILT_A_Fc20Hz_Ts100Hz, FILT_B_Fc20Hz_Ts100Hz, FILT_C_Fc20Hz_Ts100Hz, false);
    static digital_filter filt_sharpir5(FILT_A_Fc20Hz_Ts100Hz, FILT_B_Fc20Hz_Ts100Hz, FILT_C_Fc20Hz_Ts100Hz, false);
    static digital_filter filt_sharpir6(FILT_A_Fc20Hz_Ts100Hz, FILT_B_Fc20Hz_Ts100Hz, FILT_C_Fc20Hz_Ts100Hz, false);
    static uint32_t last_read_timestamp_micros;

    void init() {
        analogReadResolution(12);
        last_read_timestamp_micros = get_time_micros_safe();
    }

    void read_loop_100hz() {
        uint16_t sharpir1_reading_u16 = analogRead(SHARPIR1_PIN);
        uint16_t sharpir2_reading_u16 = analogRead(SHARPIR2_PIN);
        uint16_t sharpir3_reading_u16 = analogRead(SHARPIR3_PIN);
        uint16_t sharpir4_reading_u16 = analogRead(SHARPIR4_PIN);
        uint16_t sharpir5_reading_u16 = analogRead(SHARPIR5_PIN);
        uint16_t sharpir6_reading_u16 = analogRead(SHARPIR6_PIN);

        float sharpir1_reading_float = ((float) sharpir1_reading_u16) * ADC_AREF_VOLTAGE / 4095;
        float sharpir2_reading_float = ((float) sharpir2_reading_u16) * ADC_AREF_VOLTAGE / 4095;
        float sharpir3_reading_float = ((float) sharpir3_reading_u16) * ADC_AREF_VOLTAGE / 4095;
        float sharpir4_reading_float = ((float) sharpir4_reading_u16) * ADC_AREF_VOLTAGE / 4095;
        float sharpir5_reading_float = ((float) sharpir5_reading_u16) * ADC_AREF_VOLTAGE / 4095;
        float sharpir6_reading_float = ((float) sharpir6_reading_u16) * ADC_AREF_VOLTAGE / 4095;

        filt_sharpir1.update(sharpir1_reading_float);
        filt_sharpir2.update(sharpir2_reading_float);
        filt_sharpir3.update(sharpir3_reading_float);
        filt_sharpir4.update(sharpir4_reading_float);
        filt_sharpir5.update(sharpir5_reading_float);
        filt_sharpir6.update(sharpir6_reading_float);

        last_read_timestamp_micros = get_time_micros_safe();
    }

    void get_filtered_readings(sharpir_reading_t *readings_out) {
        readings_out->micros_since_read = get_period_since_micros_safe(last_read_timestamp_micros);
        readings_out->sharpir1_voltage = filt_sharpir1.get_output();
        readings_out->sharpir2_voltage = filt_sharpir2.get_output();
        readings_out->sharpir3_voltage = filt_sharpir3.get_output();
        readings_out->sharpir4_voltage = filt_sharpir4.get_output();
        readings_out->sharpir5_voltage = filt_sharpir5.get_output();
        readings_out->sharpir6_voltage = filt_sharpir6.get_output();
    }

    void deinit() {
        filt_sharpir1.reset();
        filt_sharpir2.reset();
        filt_sharpir3.reset();
        filt_sharpir4.reset();
        filt_sharpir5.reset();
        filt_sharpir6.reset();
    }
}
