#include <cmath>
#include <cstdlib>
#include <cstring>

#include "mbed.h"
#include "C12832.h"
#include "QEI.h"
#include "ds2781.h"
#include "motor.h"
#include "pid.h"
#include "sensor.h"

#define BLE_BUFFER_SIZE 6

// control loop frequency
#define CTL_LOOP_FREQUENCY 1000
#define CTL_LOOP_PERIOD_US (1000000/CTL_LOOP_FREQUENCY)

// maximum differential factor, defines the maximum difference between two motor
#define MAX_DIFF_FACTOR 140
// turn factor constant, defines how much a turn reduces target speed
#define TURN_FACTOR_CONSTANT 0.5f

/// target rpm in run state
#define RUN_TARGET_RPM 360
#define GAP_TARGET_RPM 250
#define UTURN_TARGET_RPM 120

#define STEER_PID_KP 0.0f
#define STEER_PID_KI 0.0f
#define STEER_PID_KD 0.0f

#define LEFT_PID_KP 0.0f
#define LEFT_PID_KI 0.0f
#define LEFT_PID_KD 0.0f

#define RIGHT_PID_KP 0.0f
#define RIGHT_PID_KI 0.0f
#define RIGHT_PID_KD 0.0f

/// enable pin for both motors
DigitalOut motor_en(PB_13);
/// left motor, aka motor A
MotorControl left_motor(PC_8, PC_5, PB_2, PC_10, PC_12, false);
/// right motot, aka motor B
MotorControl right_motor(PA_15, PB_12, PB_14, PA_13, PA_14, true);

/// put in calibrated gain
PidControl left_motor_pid(LEFT_PID_KP, LEFT_PID_KI, LEFT_PID_KD);
PidControl right_motor_pid(RIGHT_PID_KP, RIGHT_PID_KI, RIGHT_PID_KD);
PidControl steering_pid(STEER_PID_KP, STEER_PID_KI, STEER_PID_KD);

SensorArray sensor_array(A5, A4, A3, A2, A1, A0, PC_3);

/// LCD display on the mbed shield
C12832 lcd(D11,D13, D12,D7,D10);

/// serial connection to the bluetooth module
Serial hm10(PA_11, PA_12);
/// buffer for reading from serial
char hm10_buffer[BLE_BUFFER_SIZE];
int hm10_buffer_cursor = 0;

///  OneWire pin for the DS2781
DigitalInOut   one_wire_pin(PD_2);

/// state machine states
enum {
    STATE_IDLE, 
    STATE_TEST_MOTOR, 
    STATE_SQUARE, 
    STATE_UTURN, 
    STATE_RUN, 
    STATE_GAP
} state;

void enter_idle_state(){
    state = STATE_IDLE;
    steering_pid.reset();
    left_motor_pid.reset();
    right_motor_pid.reset();

    motor_en.write(0);
}

void enter_run_state(){
    state = STATE_RUN;
    steering_pid.reset();
    left_motor_pid.reset();
    right_motor_pid.reset();

    left_motor.getRPM(1000);
    right_motor.getRPM(1000);
    
    left_motor.setBipolarMode(false);
    right_motor.setBipolarMode(false);

    left_motor.setForward();
    right_motor.setForward();

    left_motor.setPower(0.0);
    right_motor.setPower(0.0);

    motor_en.write(1);
}

void enter_uturn_state(){
    state = STATE_UTURN;
    steering_pid.reset();
    left_motor_pid.reset();
    right_motor_pid.reset();

    left_motor.getRPM(1000);
    right_motor.getRPM(1000);
    
    left_motor.setBipolarMode(false);
    right_motor.setBipolarMode(false);

    left_motor.setForward();
    right_motor.setBackward();

    left_motor.setPower(0.0);
    right_motor.setPower(0.0);

    motor_en.write(1);
}

/// ISR to run when data is received
void hm10_received_isr() 
{
    // get character from serial
    char c = hm10.getc();
    // store character and increment counter
    if ((c != '\n') && (c != '\r')){
        hm10_buffer[hm10_buffer_cursor++] = c;
    }
    // reset cursor if buffer is full
    if (hm10_buffer_cursor == BLE_BUFFER_SIZE - 1){
        hm10_buffer_cursor = 0;
    }
}

// sensor array multi sampling
float sensor_sample_sum = 0.0;
int sensor_sample_count = 0;
// previous differential factor
float prev_diff_factor = 0.0;

// task running at 4kHz to get samples
void state_machine_task(){
    float position = sensor_array.read_distance_from_centre();

    if (sensor_sample_count < 3){
        if (std::isnan(position) || std::isnan(sensor_sample_sum)){
            // set sum to NaN
            sensor_sample_sum = std::sqrtf(-1.0f);
        } else{
            // add position to sum
            sensor_sample_sum = sensor_sample_sum + position;
        }
        sensor_sample_count++;

    } else{
        // this will run at 1kHz
        // executes every 4th sample

        // get the mean position
        position = (sensor_sample_sum +position)/ 4.0f;
        // reset sampling variables
        sensor_sample_count = 0;
        sensor_sample_sum = 0.0;

        // get rpm of both motors
        float left_motor_rpm = left_motor.getRPM(1000);
        float right_motor_rpm = right_motor.getRPM(1000);

        // actual state machine
        switch (state){
            case STATE_IDLE:{
                break;
            }
            case STATE_RUN:{
                state_run:

                // check if line is detected
                if (std::isnan(position)){
                    // switch to gap state
                    state = STATE_GAP;
                    // reset steering pid
                    steering_pid.reset();
                    prev_diff_factor = 0.0;
                    // jump
                    goto state_gap;
                }

                // calculate differential factor
                float diff_factor = steering_pid.update(position);

                // smooth diff factor
                diff_factor = 0.6f * diff_factor + 0.4f * prev_diff_factor;

                // clamp diff factor
                diff_factor = std::fmaxf(std::fminf(diff_factor, MAX_DIFF_FACTOR), -MAX_DIFF_FACTOR);

                // store diff factor for next iteration
                prev_diff_factor = diff_factor;

                // calculate turn factor
                float turn_factor = std::fabsf(diff_factor) / MAX_DIFF_FACTOR;
                // adjust base target rpm
                float adjusted_base_target = RUN_TARGET_RPM * (1.0f -  TURN_FACTOR_CONSTANT * turn_factor);

                // calculate target RPM
                float left_target = adjusted_base_target - diff_factor;
                float right_target = adjusted_base_target + diff_factor;

                // calculate rpm error
                float left_error = left_target - left_motor_rpm;
                float right_error = right_target - right_motor_rpm;

                // update motor pid
                float left_power = left_motor_pid.update(left_error);
                float right_power = right_motor_pid.update(right_error);

                // set motor power
                left_motor.setPower(left_power);
                right_motor.setPower(right_power);

                break;
            }
            case STATE_GAP:{
                state_gap:

                // check if line is detected
                if (!std::isnan(position)){
                    // switch to run state
                    state = STATE_RUN;
                    goto state_run;
                }
                
                break;
            }
            case STATE_UTURN:{
                break;
            }
            default:
                break;
        }
    }
}

/// the main function
int main() {
    // clear display
    lcd.cls();
    // disable auto update
    lcd.set_auto_up(0);
    
    // disable motors
    motor_en.write(0);
    
    // setup bluetooth serial
    hm10.baud(9600);
    memset(hm10_buffer, 0, BLE_BUFFER_SIZE);
    // attach interrupt
    hm10.attach(&hm10_received_isr, Serial::RxIrq);

    // set initial stage
    state = STATE_TEST_MOTOR;

    // timer for control loop
    Timer t;
    // time in us when last control update is ran
    int last_ctl_time = 0;
    // time in us when last display update is ran
    int last_display_time = 0;

    // start the timer
    t.start();

    // gap run distance, used by state_gap
    float gap_run_distance;

    // start the control loop
    while(true) {
        // get the current time
        int current_time = t.read_us();

        // check if a state change is required
        if (strcmp(hm10_buffer, "stop") == 0){
            // set state
            state = STATE_IDLE;
            motor_en.write(0);
            // clear buffer
            memset(hm10_buffer, 0, BLE_BUFFER_SIZE);
            hm10_buffer_cursor = 0;

        } else if (strcmp(hm10_buffer, "test") == 0){
            // set state
            state = STATE_TEST_MOTOR;
            motor_en.write(0);
            // clear buffer
            memset(hm10_buffer, 0, BLE_BUFFER_SIZE);
            hm10_buffer_cursor = 0;

        } else if (strcmp(hm10_buffer, "squar") == 0){
            // set state
            state = STATE_SQUARE;
            motor_en.write(0);
            // clear buffer
            memset(hm10_buffer, 0, BLE_BUFFER_SIZE);
            hm10_buffer_cursor = 0;

        } else if (strcmp(hm10_buffer, "uturn") == 0) {
            // set state
            state = STATE_UTURN;
            motor_en.write(0);
            // clear buffer
            memset(hm10_buffer, 0, BLE_BUFFER_SIZE);
            hm10_buffer_cursor = 0;
        }

        // only run every 1000 us, aka 1kHz
        if (current_time - last_ctl_time >= 1000){
             // update control time
            last_ctl_time = current_time;

            switch (state){
                ///////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////
                ////////////////                          IDLE  STATE                       ////////////////////
                ///////////////////////////////////////////////////////////////////
                case STATE_IDLE:{
                    // run every 200 ms
                    if (current_time - last_display_time >= 200000){
                        last_display_time = current_time;

                        lcd.cls();

                        lcd.locate(0, 0);
                        lcd.printf("voltage: %.2f v", (float)(ReadVoltage()) * 0.00976f);

                        lcd.locate(0, 10);
                        lcd.printf("current %.2f A", (float)(ReadCurrent())/ 6400.0f);

                        lcd.locate(0, 20);
                        lcd.printf("temperature: %.2f °C", (float)(ReadTemperature()) * 0.125f);

                        lcd.copy_to_lcd();
                    }
                    break;
                }

                ///////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////
                ////////////////                          TEST  STATE                       ////////////////////
                ///////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////
                case STATE_TEST_MOTOR: {              
                    // run every 200 ms
                    if (current_time - last_display_time >= 200000){
                        last_display_time = current_time;

                        int left = left_motor.getPulses();
                        int right = right_motor.getPulses();

                        lcd.cls();
                        lcd.locate(0, 0);

                        float left_rpm = left_motor.getRPM(200000);
                        float right_rpm = right_motor.getRPM(200000);

                        lcd.printf("rpm: %.2f, %.2f", left_rpm, right_rpm);

                        lcd.locate(0, 10);

                        float a0, a1, a2, a3, a4, a5;

                        sensor_array.read_raw(&a0, &a1, &a2, &a3, &a4, &a5);

                        lcd.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", a0, a1, a2, a3, a4,a5);

                        lcd.locate(0, 20);

                        lcd.printf("distance: %.3f, sd: %.3f", sensor_array.read_distance_from_centre(), sensor_array.read_sd());
                            
                        lcd.copy_to_lcd();
                    }
                    break;
                }

                ///////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////
                ////////////////                        U-TURN  STATE                     ////////////////////
                ///////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////
                case STATE_UTURN:{
                    break;
                }

                ///////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////
                ////////////////                           RUN  STATE                        ////////////////////
                ///////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////
                case STATE_RUN:{
                    // disable bipolar mode
                    left_motor.setBipolarMode(false);
                    right_motor.setBipolarMode(false);

                    motor_en.write(0);

                    // get the distance from centre
                    float sensor_position = sensor_array.read_distance_from_centre();
                    // get motor rpm from encoder
                    float left_encoder_rpm = left_motor.getRPM(CTL_LOOP_PERIOD_US);
                    float right_encoder_rpm = right_motor.getRPM(CTL_LOOP_PERIOD_US);

                    // if position is NaN, no white line is detected
                    if (std::isnan(sensor_position)){
                        // reset pid controller
                        steering_pid.reset();
                        left_motor_pid.reset();
                        right_motor_pid.reset();

                        // enter gap detect state
                        gap_run_distance = 0.0;
                        state = STATE_GAP;
                        break;
                    }

                    float steer_factor = steering_pid.update(std::fabsf(sensor_position));


                    break;
                }
                default:
                    break;
            };
        };
    }
}
