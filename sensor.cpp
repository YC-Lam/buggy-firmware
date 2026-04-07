
#include "mbed.h"
#include "sensor.h"

#include <cmath>
#include <cstdint>

#define SENSOR_SD_THRESHOLD 0.025

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

    float s0, s1, s2, s3, s4, s5;

    // read adc and invert reading
    s0 = 1.0f - a0.read();
    s1 = 1.0f - a1.read();
    s2 = 1.0f - a2.read();
    s3 = 1.0f - a3.read();
    s4 = 1.0f - a4.read();
    s5 = 1.0f - a5.read();

    // calculate mean
    float mean = (s0 + s1 + s2 + s3 + s4 + s5) / 6.0f;
    // calculate sum of square of difference
    float sum_square = (mean - s0) * (mean - s0) + (mean - s1) * (mean - s1) + (mean - s2) * (mean - s2) + (mean - s3) * (mean - s3) + (mean - s4) * (mean - s4) + (mean - s5) * (mean - s5);
    // calculate standar deviation
    float sd = std::sqrtf(sum_square / 5.0f);

    // check if there's a white line
    if (sd <= SENSOR_SD_THRESHOLD){
        // return nan
        return std::sqrtf(-1.0f);
    }

    // calculate norminal total distance
    float numerator = -85.0*s0  - 51.0*s1  - 17.0*s2  + 17.0*s3  + 51.0*s4 + 85.0*s5;
    // sum of readings
    float denominator = s0 + s1 + s2 + s3 + s4 + s5;

    // divide by 2 to get mm
    float position_mm = (numerator / denominator) / 2.0;

    return position_mm;
}

float SensorArray::read_sd(){
    float s0, s1, s2, s3, s4, s5;

    s0 = 1.0f - a0.read();
    s1 = 1.0f - a1.read();
    s2 = 1.0f - a2.read();
    s3 = 1.0f - a3.read();
    s4 = 1.0f - a4.read();
    s5 = 1.0f - a5.read();

    float mean = (s0 + s1 + s2 + s3 + s4 + s5) / 6.0f;
    float sum_square = (mean - s0) * (mean - s0) + (mean - s1) * (mean - s1) + (mean - s2) * (mean - s2) + (mean - s3) * (mean - s3) + (mean - s4) * (mean - s4) + (mean - s5) * (mean - s5);

    float sd = std::sqrtf(sum_square / 5.0f);

    return sd;
}