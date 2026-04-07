
#include "mbed.h"

class SensorArray{
    private:
        AnalogIn a0;
        AnalogIn a1;
        AnalogIn a2;
        AnalogIn a3;
        AnalogIn a4;
        AnalogIn a5;
        DigitalOut en;
    public:
        SensorArray(PinName a0, PinName a1, PinName a2, PinName a3, PinName a4, PinName a5, PinName en);
        void read_raw(float *a0, float *a1,float *a2, float *a3,float *a4,float *a5);
        float read_distance_from_centre();
        float read_sd();
};