#include "mbed.h"
#include "QEI.h"

class MotorControl{
    private:
        QEI encoder;
        PwmOut pwm_pin;
        DigitalOut bipolar_pin, dir_pin;

        bool inverted, forward, bipolar;
        float power;
        int last_rpm_query_pulses;
        
    public:
        MotorControl(PinName pwm, PinName bipolar, PinName dir, PinName chA, PinName chB, bool inverted);
        
        void setPWM(float duty);
        void setPower(float value);
        void setBipolarMode(bool enable);
        void setForward();
        void setBackward();
        
        bool isBipolarMode();
        int getPulses();
        float getRPM(int interval);
};