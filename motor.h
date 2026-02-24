#include "mbed.h"
#include "QEI.h"

class Motor{
    private:
        int last_query_time, last_query_pulses;
        QEI encoder;
        bool invert_pulses;
        PwmOut pwm;
        DigitalOut bipolar, dir;
    public:
        Motor(PinName pwm, PinName bipolar, PinName dir, PinName chA, PinName chB, bool invert_pulses);
        void setPWM(float value);
        void setBipolar(int value);
        void setDir(int direction);
        int getRPM(int time);
        int getPulses();
};