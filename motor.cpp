#include "mbed.h"
#include "motor.h"
#include "QEI.h"

#define PWM_PERIOD_US 50

Motor::Motor(PinName pwm, PinName bipolar, PinName dir, PinName chA, PinName chB, bool invert_pulses): 
    pwm(pwm), bipolar(bipolar), dir(dir), encoder(chA, chB, NC, 256, QEI::X4_ENCODING), invert_pulses(invert_pulses){
        this->pwm.period_us(PWM_PERIOD_US);
        last_query_time = 0;
        last_query_pulses = 0;
}

void Motor::setPWM(float value){
    pwm.write(value);
}

void Motor::setDir(int direction){
    dir.write(direction);
}

void Motor::setBipolar(int value){
    bipolar.write(value);
}

int Motor::getPulses(){
    if (invert_pulses){
        return -encoder.getPulses();
    }
    return encoder.getPulses();
}

/// time: current time in us
int Motor::getRPM(int time){
    int interval = time - last_query_time;
    last_query_time = time;

    int pulses = encoder.getPulses();
    int pulses_diff = pulses - last_query_pulses;
    last_query_pulses = pulses;

    return ((pulses_diff *60 *1000 * 1000) / interval) / 1024;
}