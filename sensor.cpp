
#include "mbed.h"
#include "sensor.h"

#include <cmath>
#include <cstdint>

#define SENSOR_WHITE_THRESHOLD 0.5

SensorArray::SensorArray(PinName a0, PinName a1, PinName a2, PinName a3, PinName a4, PinName a5, PinName en): a0(a0), a1(a1), a2(a2), a3(a3), a4(a4), a5(a5), en(en){
    this->en.write(1);
}

void SensorArray::read_raw(float *a0, float *a1, float *a2, float *a3, float *a4, float *a5){
    *a0 = this->a0.read();
    *a1 = this->a1.read();
    *a2 = this->a2.read();
    *a3 = this->a3.read();
    *a4 = this->a4.read();
    *a5 = this->a5.read();
}

/// return NaN if no white line is detected
float SensorArray::read_distance_from_centre(){
    // read sensor values
    uint16_t s0 = 65535 - a0.read_u16();
    uint16_t s1 = 65535 - a1.read_u16();
    uint16_t s2 = 65535 - a2.read_u16();
    uint16_t s3 = 65535 - a3.read_u16();
    uint16_t s4 = 65535 - a4.read_u16();
    uint16_t s5 = 65535  - a5.read_u16();

    uint32_t mean = (s0 + s1 + s2 + s3 + s4 + s5) / 6;

    uint32_t sum_square = (mean - s0) * (mean - s0) + (mean - s1) * (mean - s1) + (mean - s2) * (mean - s2) + (mean - s3) * (mean - s3) + (mean - s4) * (mean - s4) + (mean - s5) * (mean - s5);

    double sd = sqrt((sum_square / 5));

    // standard deviation does not exceed threshold
    if (sd < SENSOR_WHITE_THRESHOLD){
        // return nan
        return std::nanf("0");
    }

    double numerator = -85*s0 -51*s1 -17*s2 +17*s3 +51*s4 +85*s5;

    double denominator = s0 + s1 + s2 + s3 + s4 + s5;

    // divide by 2 to get mm
    double position_mm = (numerator / denominator) / 2.0;

    return position_mm;
}