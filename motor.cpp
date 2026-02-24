#include "mbed.h"
#include "motor.h"
#include "QEI.h"

#define PWM_PERIOD_US 50
#define PULSE_PER_REVOLUTION 1024

MotorControl::MotorControl(PinName pwm_pin, PinName bipolar_pin, PinName dir_pin, PinName chA, PinName chB, bool inverted): 
    pwm_pin(pwm_pin), bipolar_pin(bipolar_pin), dir_pin(dir_pin), encoder(chA, chB, NC, 256, QEI::X4_ENCODING), inverted(inverted){

        last_rpm_query_pulses = 0;
        power = 0.0;
        
        this->pwm_pin.period_us(PWM_PERIOD_US);
        this->setBipolarMode(0);
        this->setForward();
        this->setPower(0.0);
}

void MotorControl::setPWM(float duty){
    pwm_pin.write(duty);

    // calculate the power equivalant of pwm
    if (bipolar){
        if (duty > 0.5){
            // todo
        } else{
            // todo
        }
    } else{
        power = 1.0f - duty;
    }
}

/// automatically handles bipolar and inverted modes
void MotorControl::setPower(float value){
    // clamp value
    if (value > 1.0f){
        value = 1.0f;
    }
    if (value < 0.0f){
        value = 0.0f;
    }

    // record the power set
    power = value;

    if (bipolar){
        // half the value
        value = 0.5f * value;

        if ((forward && inverted) || (forward == 0)){
            pwm_pin.write(0.5 - value);
        } else{
            pwm_pin.write(0.5 + value);
        }
    } else{
        pwm_pin.write(1.0 - value);
    }
}

/// enable or disable bibolar mode, automatically converts pwm
void MotorControl::setBipolarMode(bool enable){
    if (enable){
        bipolar = true;
        bipolar_pin.write(1);
    } else{
        bipolar = false;
        bipolar_pin.write(0);
    }

    // adjust pwm
    this->setPower(power);
}

void MotorControl::setForward(){
    // set dir pin anyways
    if (inverted){
        dir_pin.write(0);
    } else{
        dir_pin.write(1);
    }

    forward = true;
    // adjust pwm
    this->setPower(power);
}

void MotorControl::setBackward(){
    // set dir pin anyways
    if (inverted){
        dir_pin.write(1);
    } else{
        dir_pin.write(0);
    }

    forward = false;
    // adjust pwm
    this->setPower(power);
}

bool MotorControl::isBipolarMode(){
    return bipolar;
}

int MotorControl::getPulses(){
    if (inverted){
        return -encoder.getPulses();
    }
    return encoder.getPulses();
}

/// interval: interval in us
float MotorControl::getRPM(int interval){

    int pulses = this->getPulses();
    int pulses_diff = pulses - last_rpm_query_pulses;
    
    last_rpm_query_pulses = pulses;

    float a = (float)pulses_diff * 60000000.0f;

    return  a / (float)(interval* PULSE_PER_REVOLUTION);
}