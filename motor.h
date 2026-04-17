#include "mbed.h"
#include "QEI.h"

class MotorControl{
    private:
        QEI encoder;
        PwmOut pwm_pin;
        // digital pins
        DigitalOut bipolar_pin, dir_pin;

        // flags
        bool inverted, forward, bipolar;
        // power output
        float power;

        int last_pulses;           // last total encoder count
        int pulse_accumulator;     // sum of pulses over window
        int sample_count;          // counts 1 ms samples

        float rpm;                 // raw RPM from window
        float rpm_filtered;        // smoothed RPM for PID

    public:
        MotorControl(PinName pwm, PinName bipolar, PinName dir, PinName chA, PinName chB, bool inverted);
        
        void setPWM(float duty);
        void setPower(float value);
        void setBipolarMode(bool enable);
        void setForward();
        void setBackward();
        
        bool isBipolarMode();
        int getPulses();
        float getRPM(int freq);
};
