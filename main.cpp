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

// maximum differential factor, defines half the maximum difference between two motor
#define MAX_DIFF_FACTOR 140
// turn factor constant, defines how much a turn reduces target speed
#define TURN_FACTOR_CONSTANT 0.5f
// number of counts before stopping buggy in gap, 1 count per ms
#define GAP_MAX_COUNT 10

/// target rpm in run state
#define RUN_TARGET_RPM 360
#define GAP_TARGET_RPM 250
#define UTURN_TARGET_RPM 120

#define STEER_PID_KP 3.0f
#define STEER_PID_KI 0.0f
#define STEER_PID_KD 8.0f

#define LEFT_PID_KP 0.002f
#define LEFT_PID_KI 0.1f
#define LEFT_PID_KD 0.0f

#define RIGHT_PID_KP 0.002f
#define RIGHT_PID_KI 0.1f
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
// counter for gap state
int gap_counter;

float lcd_left_rpm = 0.0;
float lcd_right_rpm = 0.0;
float lcd_sensor_position = 0.0;

// task running at 4kHz to get samples
// get 4 samples of sensor array position before running contol update
void state_machine_task(){
    float position = sensor_array.read_distance_from_centre();

    if (sensor_sample_count < 3){
        if ((isnan(position))|| (isnan(sensor_sample_sum))){
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
        float left_motor_rpm = left_motor.getRPM_1KHz();
        float right_motor_rpm = right_motor.getRPM_1KHz();
        
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

                // calculate turn factor
                float turn_factor = fabsf(diff_factor) / MAX_DIFF_FACTOR;
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
                float left_power = left_motor_pid.update(left_error);
                float right_power = right_motor_pid.update(right_error);

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
                // todo
                break;
            }
            default:
                break;
        }
    }
}

void lcd_update_task(){
    lcd.cls();
    lcd.locate(0, 0);

    lcd.printf("rpm: %7.2f, %7.2f", lcd_left_rpm, lcd_right_rpm);

    lcd.locate(0, 10);

    lcd.printf("distance: %5.3f, sd: %f", lcd_sensor_position, sensor_array.read_sd());

    lcd.copy_to_lcd();
}

/// the main function
int main() {
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

    state_machine_ticker.attach_us(state_machine_task, 250);
    lcd_update_ticker.attach_us(lcd_update_task, 200000);

    // start the control loop
    while(true) {
        // check if a state change is required
        if (strcmp(hm10_buffer, "stop") == 0){
            // set state
            enter_idle_state();
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

        } else if (strcmp(hm10_buffer, "start") == 0){
            // set state
            enter_run_state();
            // clear buffer
            memset(hm10_buffer, 0, BLE_BUFFER_SIZE);
            hm10_buffer_cursor = 0;

        } else if (strcmp(hm10_buffer, "uturn") == 0) {
            // set state
            enter_uturn_state();
            // clear buffer
            memset(hm10_buffer, 0, BLE_BUFFER_SIZE);
            hm10_buffer_cursor = 0;
        }
    }
}
