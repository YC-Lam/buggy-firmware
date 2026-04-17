// source code file for PID control

#include <stdlib.h>
#include <math.h>
#include "pid.h"

PidControl::PidControl(float Kp, float Ki, float Kd, float freq){
    PidControl::Kp = Kp;
    PidControl::Ki = Ki;
    PidControl::Kd = Kd;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    integrator = 0.0;
    previous_error = 0.0;
    integrator_limit = INFINITY;
    frequency = freq;
}

PidControl::~PidControl(){}

void PidControl::reset(){
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    integrator = 0.0;
    previous_error = 0.0;
    integrator_limit = INFINITY;
}

void PidControl::set_kp(float kp){
    Kp = kp;
}

void PidControl::set_ki(float ki){
    Ki = ki;
}

void PidControl::set_kd(float kd){
    Kd = kd;
}

/// @brief 
/// @param error the new error
/// @return output
float PidControl::update(float error){
    float output;
    integrator += error;

    if (integrator > integrator_limit) {
        integrator = integrator_limit;
    } else if (integrator < -integrator_limit) {
        integrator = -integrator_limit;
    }

    output  = Kp * error;
    output += Ki * integrator / frequency;
    output += Kd * (error - previous_error) * frequency;

    if (output > 1.0f || output < 0.0f) {
        integrator *= 0.9f; // anti-windup
    }

    previous_error = error;
    return output;
}