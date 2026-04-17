#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdlib.h>
#include <math.h>

#include "mbed.h"
#include "C12832.h"
#include "QEI.h"
#include "ds2781.h"
#include "motor.h"
#include "pid.h"
#include "pidautotuner.h"
#include "sensor.h"

#define BLE_BUFFER_SIZE 10

// control loop frequency
#define CTL_LOOP_FREQUENCY  200
#define CTL_LOOP_PERIOD_US (1000000/CTL_LOOP_FREQUENCY)

#define SENSOR_SAMPLE_COUNT 20

// maximum differential factor, defines half the maximum difference between two motor
#define MAX_DIFF_FACTOR 140
// turn factor constant, defines how much a turn reduces target speed
#define TURN_FACTOR_CONSTANT 0.8f
// number of counts before stopping buggy in gap, 1 count per ms
#define GAP_MAX_COUNT 20
// smoothing factor of motor ouput
#define MOTOR_SMOOTH_FACTOR 0.3f
#define MOTOR_BASE_POWER 0.5f

/// target rpm in run state
#define RUN_TARGET_RPM 250
#define GAP_TARGET_RPM 200
#define UTURN_TARGET_RPM 120

#define STEER_PID_KP 3.5f
#define STEER_PID_KI 0.0f
#define STEER_PID_KD 0.0f

#define LEFT_PID_KP 0.06f
#define LEFT_PID_KI 0.002f
#define LEFT_PID_KD 0.0f

#define RIGHT_PID_KP 0.0599f
#define RIGHT_PID_KI 0.002f
#define RIGHT_PID_KD 0.0f

// global timer
Timer global_timer;

/// enable pin for both motors
DigitalOut motor_en(PB_13);
/// left motor, aka motor A
MotorControl left_motor(PC_8, PC_5, PB_2, PC_10, PC_12, false);
/// right motot, aka motor B
MotorControl right_motor(PA_15, PB_12, PB_14, PA_13, PA_14, true);

/// put in calibrated gain
PidControl left_motor_pid(LEFT_PID_KP, LEFT_PID_KI, LEFT_PID_KD, CTL_LOOP_FREQUENCY);
PidControl right_motor_pid(RIGHT_PID_KP, RIGHT_PID_KI, RIGHT_PID_KD, CTL_LOOP_FREQUENCY);
PidControl steering_pid(STEER_PID_KP, STEER_PID_KI, STEER_PID_KD, CTL_LOOP_FREQUENCY);

// pid autotuner
PIDAutotuner pid_auto_tuner;

SensorArray sensor_array(A5, A4, A3, A2, A1, A0, PC_3);

/// LCD display on the mbed shield
C12832 lcd(D11,D13, D12,D7,D10);

/// serial connection to the bluetooth module
Serial hm10(PA_11, PA_12);
/// buffer for reading from serial
char hm10_buffer[BLE_BUFFER_SIZE];
int hm10_buffer_cursor = 0;
bool hm10_command_ready = false;

///  OneWire pin for the DS2781
DigitalInOut   one_wire_pin(PD_2);

/// state machine states
enum {
    STATE_IDLE, 
    STATE_TEST_MOTOR, 
    STATE_SQUARE, 
    STATE_UTURN, 
    STATE_RUN, 
    STATE_GAP,
    STATE_MOTOR_TUNE_LEFT,
    STATE_MOTOR_TUNE_RIGHT,
    STATE_STEER_TUNE
} state;

// sensor array multi sampling
int sensor_sample_count = 0;
float sensor_samples[SENSOR_SAMPLE_COUNT] = {};

// previous differential factor
float prev_diff_factor = 0.0;
float prev_left_power = 0.0;
float prev_right_power= 0.0;
// counter for gap state
int gap_counter;

float lcd_left_rpm = 0.0;
float lcd_right_rpm = 0.0;
float lcd_sensor_position = 0.0;
float lcd_steer_factor = 0.0;
float lcd_left_target = 0.0;
float lcd_right_target = 0.0;

float lcd_tune_kp = 0.0;
float lcd_tune_ki = 0.0;
float lcd_tune_kd = 0.0;

void enter_idle_state(){
    state = STATE_IDLE;
    steering_pid.reset();
    left_motor_pid.reset();
    right_motor_pid.reset();

    motor_en.write(0);

    //hm10.printf("enter idle state\n");
}

void enter_run_state(){
    state = STATE_RUN;
    steering_pid.reset();
    left_motor_pid.reset();
    right_motor_pid.reset();
    
    left_motor.setBipolarMode(false);
    right_motor.setBipolarMode(false);

    left_motor.setForward();
    right_motor.setForward();

    prev_left_power = 0.0;
    prev_right_power = 0.0;

    motor_en.write(1);

    //hm10.printf("enter run state\n");
}

void enter_test_state(){
    state = STATE_TEST_MOTOR;
    steering_pid.reset();
    left_motor_pid.reset();
    right_motor_pid.reset();
    
    left_motor.setBipolarMode(false);
    right_motor.setBipolarMode(false);

    left_motor.setForward();
    right_motor.setForward();

    prev_left_power = 0.0;
    prev_right_power = 0.0;

    motor_en.write(1);

    //hm10.printf("enter test state\n");
}

void enter_uturn_state(){
    state = STATE_UTURN;
    steering_pid.reset();
    left_motor_pid.reset();
    right_motor_pid.reset();
    
    left_motor.setBipolarMode(false);
    right_motor.setBipolarMode(false);

    left_motor.setForward();
    right_motor.setBackward();

    prev_left_power = 0.0;
    prev_right_power = 0.0;

    motor_en.write(1);

    //hm10.printf("enter turn state\n");
}

void enter_motor_tune_right_state(){
    state = STATE_MOTOR_TUNE_RIGHT;
    pid_auto_tuner = PIDAutotuner();
    pid_auto_tuner.setTargetInputValue(RUN_TARGET_RPM);
    pid_auto_tuner.setLoopInterval(CTL_LOOP_PERIOD_US);
    pid_auto_tuner.setOutputRange(-0.5, 0.5);
    pid_auto_tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);
    pid_auto_tuner.setTuningCycles(10);
    pid_auto_tuner.startTuningLoop(global_timer.read_us());

    steering_pid.reset();
    left_motor_pid.reset();
    right_motor_pid.reset();
    
    left_motor.setBipolarMode(false);
    right_motor.setBipolarMode(false);

    left_motor.setForward();
    right_motor.setForward();

    prev_left_power = 0.0;
    prev_right_power = 0.0;

    motor_en.write(1);

    //hm10.printf("enter right tune state\n");
}

void enter_motor_tune_left_state(){
    state = STATE_MOTOR_TUNE_LEFT;
    pid_auto_tuner = PIDAutotuner();
    pid_auto_tuner.setTargetInputValue(RUN_TARGET_RPM);
    pid_auto_tuner.setLoopInterval(CTL_LOOP_PERIOD_US);
    pid_auto_tuner.setOutputRange(-0.5, 0.5);
    pid_auto_tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);
    pid_auto_tuner.setTuningCycles(10);
    pid_auto_tuner.startTuningLoop(global_timer.read_us());

    steering_pid.reset();
    left_motor_pid.reset();
    right_motor_pid.reset();
    
    left_motor.setBipolarMode(false);
    right_motor.setBipolarMode(false);

    left_motor.setForward();
    right_motor.setForward();

    prev_left_power = 0.0;
    prev_right_power = 0.0;

    motor_en.write(1);

    //hm10.printf("enter left tune state\n");
}

void enter_steer_tune_state(){
    state = STATE_STEER_TUNE;
    pid_auto_tuner = PIDAutotuner();
    pid_auto_tuner.setTargetInputValue(0.0);
    pid_auto_tuner.setLoopInterval(CTL_LOOP_PERIOD_US);
    pid_auto_tuner.setOutputRange(-5.0, 5.0);
    pid_auto_tuner.setZNMode(PIDAutotuner::ZNModeLessOvershoot);
    pid_auto_tuner.setTuningCycles(10);
    pid_auto_tuner.startTuningLoop(global_timer.read_us());

    steering_pid.reset();
    left_motor_pid.reset();
    right_motor_pid.reset();
    
    left_motor.setBipolarMode(false);
    right_motor.setBipolarMode(false);

    left_motor.setForward();
    right_motor.setForward();

    prev_left_power = 0.0;
    prev_right_power = 0.0;

    motor_en.write(1);

    //hm10.printf("enter steer state\n");
}

/// ISR to run when data is received
void hm10_received_isr() 
{
    // get character from serial
    char c = hm10.getc();
    // end of message
    if ((c == '\n') || (c == '\r')){
        hm10_buffer[hm10_buffer_cursor] = '\0';
        hm10_command_ready = true;
        hm10_buffer_cursor = 0;

    } else{
            // only fill buffer
            if (hm10_buffer_cursor < BLE_BUFFER_SIZE - 1){
                // store character and increment counter
                hm10_buffer[hm10_buffer_cursor++] = c;
            }
    }
    
}

// task running at 4kHz to get samples
void sensor_sample_task(){
    float position = sensor_array.read_distance_from_centre();

    sensor_samples[sensor_sample_count++] = position;
    sensor_sample_count %= SENSOR_SAMPLE_COUNT;
}


// main control
void state_machine_task(){
    // get the mean position
    float position = 0.0;

    for (int i=0;i < SENSOR_SAMPLE_COUNT;i++){
        position += sensor_samples[i];
    }

    position /= SENSOR_SAMPLE_COUNT;

    // get rpm of both motors
    float left_motor_rpm = left_motor.getRPM(CTL_LOOP_FREQUENCY);
    float right_motor_rpm = right_motor.getRPM(CTL_LOOP_FREQUENCY);
    
    // update lcd
    lcd_left_rpm = left_motor_rpm;
    lcd_right_rpm = right_motor_rpm;
    lcd_sensor_position = position;

    // actual state machine
    switch (state){
        case STATE_RUN:{
            state_run:

            // check if line is detected
            if ((isnan(position))){
                // switch to gap state
                state = STATE_GAP;
                gap_counter = 0;
                // jump
                goto state_gap;
            }

            // calculate differential factor
            float diff_factor = steering_pid.update(position);

            // smooth diff factor
            // diff_factor = 0.6f * diff_factor + 0.4f * prev_diff_factor;

            // clamp diff factor
            if (diff_factor > MAX_DIFF_FACTOR){
                diff_factor = MAX_DIFF_FACTOR;
            }
            if (diff_factor < -MAX_DIFF_FACTOR){
                diff_factor = -MAX_DIFF_FACTOR;
            }

            // store diff factor for next iteration
            prev_diff_factor = diff_factor;
            lcd_steer_factor = diff_factor;

            // calculate turn factor
            float turn_factor = fabsf(diff_factor) / MAX_DIFF_FACTOR;
            // adjust base target rpm
            float adjusted_base_target = RUN_TARGET_RPM * (1.0f -  TURN_FACTOR_CONSTANT * turn_factor);

            // calculate target RPM
            float left_target = adjusted_base_target - diff_factor;
            float right_target = adjusted_base_target + diff_factor;

            lcd_left_target = left_target;
            lcd_right_target = right_target;

            // calculate rpm error
            float left_error = left_target - left_motor_rpm;
            float right_error = right_target - right_motor_rpm;

            // update motor pid
            float left_power = MOTOR_BASE_POWER + left_motor_pid.update(left_error);
            float right_power = MOTOR_BASE_POWER + right_motor_pid.update(right_error);

            left_power = (1.0f - MOTOR_SMOOTH_FACTOR) * left_power + MOTOR_SMOOTH_FACTOR * prev_left_power;
            right_power = (1.0f - MOTOR_SMOOTH_FACTOR) * right_power + MOTOR_SMOOTH_FACTOR * prev_right_power;

            prev_left_power = left_power;
            prev_right_power = right_power;
            
            // set motor power
            left_motor.setPower(left_power);
            right_motor.setPower(right_power);

            break;
        }
        case STATE_GAP:{
            state_gap:

            // check if line is detected
            if (!(isnan(position))){
                // switch to run state
                state = STATE_RUN;
                goto state_run;
            }

            // increment gap count
            gap_counter++;
            // if gap count exceeds max count, stop buggy
            if (gap_counter >= GAP_MAX_COUNT) {
                // enter idle to stop
                enter_idle_state();
            }   

            float diff_factor = 0.95f * prev_diff_factor;
            prev_diff_factor = diff_factor;

            // calculate turn factor
            float turn_factor = fabsf(diff_factor) / MAX_DIFF_FACTOR;
            // adjust base target rpm
            float adjusted_base_target = GAP_TARGET_RPM * (1.0f -  TURN_FACTOR_CONSTANT * turn_factor);

            // calculate target RPM
            float left_target = adjusted_base_target - diff_factor;
            float right_target = adjusted_base_target + diff_factor;

            // calculate rpm error
            float left_error = left_target - left_motor_rpm;
            float right_error = right_target - right_motor_rpm;

            // update motor pid
            float left_power = MOTOR_BASE_POWER + left_motor_pid.update(left_error);
            float right_power = MOTOR_BASE_POWER + right_motor_pid.update(right_error);

            left_power = (1.0f - MOTOR_SMOOTH_FACTOR) * left_power + MOTOR_SMOOTH_FACTOR * prev_left_power;
            right_power = (1.0f - MOTOR_SMOOTH_FACTOR) * right_power + MOTOR_SMOOTH_FACTOR * prev_right_power;

            prev_left_power = left_power;
            prev_right_power = right_power;

            // set motor power
            left_motor.setPower(left_power);
            right_motor.setPower(right_power);

            break;
        }
        case STATE_UTURN:{

            // todo
            break;
        }
        case STATE_TEST_MOTOR:{
            float left_target = RUN_TARGET_RPM;
            float right_target = RUN_TARGET_RPM;

            lcd_left_target = left_target;
            lcd_right_target = right_target;

            // calculate rpm error
            float left_error = left_target - left_motor_rpm;
            float right_error = right_target - right_motor_rpm;

            // update motor pid
            float left_power = MOTOR_BASE_POWER + left_motor_pid.update(left_error);
            float right_power = MOTOR_BASE_POWER + right_motor_pid.update(right_error);

            left_power = (1.0f - MOTOR_SMOOTH_FACTOR) * left_power + MOTOR_SMOOTH_FACTOR * prev_left_power;
            right_power = (1.0f - MOTOR_SMOOTH_FACTOR) * right_power + MOTOR_SMOOTH_FACTOR * prev_right_power;

            prev_left_power = left_power;
            prev_right_power = right_power;

            // set motor power
            left_motor.setPower(left_power);
            right_motor.setPower(right_power);

            //hm10.printf("%f,%f,%f,%f", left_motor_rpm, left_power, right_motor_rpm, right_power);

            break;
        }
        case STATE_MOTOR_TUNE_RIGHT:{
            if (pid_auto_tuner.isFinished()){
                lcd_tune_kp = pid_auto_tuner.getKp();
                lcd_tune_ki = pid_auto_tuner.getKi();
                lcd_tune_kd = pid_auto_tuner.getKd();
                enter_idle_state();
                break;
            }

            float power = MOTOR_BASE_POWER + pid_auto_tuner.tunePID(right_motor_rpm, global_timer.read_us());

            power = (1.0f - MOTOR_SMOOTH_FACTOR) * power + MOTOR_SMOOTH_FACTOR * prev_left_power;
            prev_left_power = power;

            left_motor.setPower(0.0);
            right_motor.setPower(power);

            break;
        }
        case STATE_MOTOR_TUNE_LEFT:{
            if (pid_auto_tuner.isFinished()){
                lcd_tune_kp = pid_auto_tuner.getKp();
                lcd_tune_ki = pid_auto_tuner.getKi();
                lcd_tune_kd = pid_auto_tuner.getKd();
                enter_idle_state();
                break;
            }

            float power = MOTOR_BASE_POWER + pid_auto_tuner.tunePID(left_motor_rpm, global_timer.read_us());

            power = (1.0f - MOTOR_SMOOTH_FACTOR) * power + MOTOR_SMOOTH_FACTOR * prev_left_power;
            prev_left_power = power;

            left_motor.setPower(power);
            right_motor.setPower(0.0);

            break;
        }
        case STATE_STEER_TUNE:{
            if (pid_auto_tuner.isFinished()){
                lcd_tune_kp = pid_auto_tuner.getKp();
                lcd_tune_ki = pid_auto_tuner.getKi();
                lcd_tune_kd = pid_auto_tuner.getKd();
                enter_idle_state();
                break;
            }

            if (isnan(position)){
                enter_idle_state();
                break;
            }

            // calculate differential factor
            float diff_factor = pid_auto_tuner.tunePID(position, global_timer.read_us());

            // smooth diff factor
            diff_factor = 0.6f * diff_factor + 0.4f * prev_diff_factor;

            // clamp diff factor
            if (diff_factor > MAX_DIFF_FACTOR){
                diff_factor = MAX_DIFF_FACTOR;
            }
            if (diff_factor < -MAX_DIFF_FACTOR){
                diff_factor = -MAX_DIFF_FACTOR;
            }

            // store diff factor for next iteration
            prev_diff_factor = diff_factor;
            lcd_steer_factor = diff_factor;

            // calculate turn factor
            float turn_factor = fabsf(diff_factor) / MAX_DIFF_FACTOR;
            // adjust base target rpm
            float adjusted_base_target = RUN_TARGET_RPM * (1.0f -  TURN_FACTOR_CONSTANT * turn_factor);

            // calculate target RPM
            float left_target = adjusted_base_target - diff_factor;
            float right_target = adjusted_base_target + diff_factor;

            lcd_left_target = left_target;
            lcd_right_target = right_target;

            // calculate rpm error
            float left_error = left_target - left_motor_rpm;
            float right_error = right_target - right_motor_rpm;

            // update motor pid
            float left_power = left_motor_pid.update(left_error);
            float right_power = right_motor_pid.update(right_error);

            left_power = (1.0f - MOTOR_SMOOTH_FACTOR) * left_power + MOTOR_SMOOTH_FACTOR * prev_left_power;
            right_power = (1.0f - MOTOR_SMOOTH_FACTOR) * right_power + MOTOR_SMOOTH_FACTOR * prev_right_power;

            prev_left_power = left_power;
            prev_right_power = right_power;

            // set motor power
            left_motor.setPower(left_power);
            right_motor.setPower(right_power);
            
            break;
        }
        default:
            break;
    }
}

void lcd_update_task(){
    lcd.cls();
    lcd.locate(0, 0);

    lcd.printf("rpm: %7.2f, %7.2f", lcd_left_rpm, lcd_right_rpm);

    lcd.locate(0, 10);

    lcd.printf("distance: %5.3f, sd: %.3f", lcd_sensor_position, sensor_array.read_sd());

    lcd.locate(0, 20);

    lcd.printf("%.5f, %.5f, %.5f", lcd_tune_kp, lcd_tune_ki, lcd_tune_kd);

    lcd.copy_to_lcd();

    if (lcd_steer_factor != 0.0){
       // hm10.printf("distance: %f\n", lcd_sensor_position);
        //hm10.printf("steer factor: %f\n", lcd_steer_factor);
        //hm10.printf("target: %f, %f\n", lcd_left_target, lcd_right_target);
        //hm10.printf("rpm: %f, %f\n\n", lcd_left_rpm, lcd_right_rpm);
    }
    
    lcd_steer_factor = 0.0;

}

/// the main function
int main() {
    global_timer.start();

    // clear display
    lcd.cls();
    // disable auto update
    lcd.set_auto_up(0);
    
    // disable motors
    motor_en.write(0);
    
    // set initial stage
    state = STATE_IDLE;
    
    // setup bluetooth serial
    hm10.baud(9600);
    memset(hm10_buffer, 0, BLE_BUFFER_SIZE);
    // attach interrupt
    hm10.attach(&hm10_received_isr, Serial::RxIrq);

    Ticker state_machine_ticker;
    Ticker lcd_update_ticker;
    Ticker sample_ticker;

    sample_ticker.attach_us(sensor_sample_task, 250);
    state_machine_ticker.attach_us(state_machine_task, CTL_LOOP_PERIOD_US);
    lcd_update_ticker.attach_us(lcd_update_task, 200000);

    // start the control loop
    while(true) {
        if (hm10_command_ready) {
            hm10_command_ready = false;

            if (strcmp(hm10_buffer, "start") == 0) {
                enter_run_state();
            } else if (strcmp(hm10_buffer, "stop") == 0) {
                enter_idle_state();
            } else if (strcmp(hm10_buffer, "turn") == 0) {
                enter_uturn_state();
            } else if (strcmp(hm10_buffer, "test") == 0) {
                enter_test_state();
            } else if (strcmp(hm10_buffer, "right") == 0){
                enter_motor_tune_right_state();
            } else if (strcmp(hm10_buffer, "left") == 0){
                enter_motor_tune_left_state();
            } else if (strcmp(hm10_buffer, "steer") == 0){
                enter_steer_tune_state();
            }

            memset(hm10_buffer, 0, BLE_BUFFER_SIZE);
        }
    }
}
