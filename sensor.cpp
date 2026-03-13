
#include "mbed.h"
#include "sensor.h"

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