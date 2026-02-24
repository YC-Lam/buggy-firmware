// source code file for PID control

#include <stdlib.h>
#include <math.h>
#include "pid.h"

PidControl::PidControl(float Kp, float Ki, float Kd){
    PidControl::Kp = Kp;
    PidControl::Ki = Ki;
    PidControl::Kd = Kd;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    integrator = 0.0;
    previous_error = 0.0;
    integrator_limit = INFINITY;
    frequency = 1.0;
}

PidControl::~PidControl(){}

void PidControl::reset(){
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    integrator = 0.0;
    previous_error = 0.0;
    integrator_limit = INFINITY;
    frequency = 1.0;
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

    previous_error = error;
    return output;
}