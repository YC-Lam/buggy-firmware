#include <cstdlib>
#include <cstring>

#include "mbed.h"
#include "C12832.h"
#include "QEI.h"
#include "ds2781.h"
#include "motor.h"
#include "pot.h"
#include "pid.h"
#include "sensor.h"

#define BLE_BUFFER_SIZE 6

/// number of pulses when running square
#define SQUARE_STRAIGHT_PULSES 2048
#define SQUARE_RIGHT_TURN_PULSES 1600
#define SQUARE_LEFT_TURN_PULSES 1600
#define U_TURN_PULSES 1400

#define CTL_LOOP_FREQUENCY 1000
#define CTL_LOOP_PERIOD_US (1000000/CTL_LOOP_FREQUENCY)

/// target rpm in run state
#define RUN_TARGET_RPM 360
#define RUN_SQUARE_CORNER_RPM 180

/// enable pin for both motors
DigitalOut motor_en(PB_13);
/// left motor, aka motor A
MotorControl left_motor(PC_8, PC_5, PB_2, PC_10, PC_12, false);
/// right motot, aka motor B
MotorControl right_motor(PA_15, PB_12, PB_14, PA_13, PA_14, true);

PidControl left_motor_pid(0.00278, 0.0, 0.0);
PidControl right_motor_pid(0.00278, 0.0, 0.0);
PidControl steering_pid();

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
    STATE_IDLE, STATE_TEST_MOTOR, STATE_SQUARE, STATE_UTURN, STATE_RUN
} state;

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

/// function to simplify repeat code in square state.
/// This function takes in the number of pulses for each motor,
/// if the number is negative, motor goes backwards
void run_motor_pulses_blocking(int left_motor_pulses, int right_motor_pulses, float power){
    // motor is done if no pulses to run
    bool left_motor_done = left_motor_pulses == 0;
    bool right_motor_done = right_motor_pulses == 0;

    // get the pulses before running
    int init_left_pulses = left_motor.getPulses();
    int init_right_pulses = right_motor.getPulses();

    // determind direction of motor
    if (left_motor_pulses < 0){
        left_motor.setBackward();
        left_motor_pulses = -left_motor_pulses;
    } else{
        left_motor.setForward();
    }
    // determind direction of motor
    if (right_motor_pulses < 0 ){
        right_motor.setBackward();
        right_motor_pulses = -right_motor_pulses;
    } else{
        right_motor.setForward();
    }

    // enable motor if there's pulses to run
    if (left_motor_pulses > 0){
        left_motor.setPower(power);
    }
    if (right_motor_pulses > 0){
        right_motor.setPower(power);
    }

    while (true){
        if (std::abs(left_motor.getPulses() - init_left_pulses) >= left_motor_pulses){
            left_motor.setPower(0.0);
            left_motor_done = true;
        }
        if (std::abs(right_motor.getPulses() - init_right_pulses) >= right_motor_pulses){
            right_motor.setPower(0.0);
            right_motor_done = true;
        }
        if (right_motor_done && left_motor_done){
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

                    // disable bipolar mode
                    left_motor.setBipolarMode(false);
                    right_motor.setBipolarMode(false);

                    // disable motors
                    left_motor.setPower(0.0);
                    right_motor.setPower(0.0);

                    motor_en.write(1);// disable bipolar mode// disable bipolar mode
                    
                    // run every 200 ms
                    if (current_time - last_display_time >= 200000){
                        last_display_time = current_time;

                        int left = left_motor.getPulses();
                        int right = right_motor.getPulses();

                        lcd.cls();
                        lcd.locate(0, 0);
                        lcd.printf("pulses: %d, %d", left , right);
                        lcd.locate(0, 10);

                        float left_rpm = left_motor.getRPM(200000);
                        float right_rpm = right_motor.getRPM(200000);

                        lcd.printf("rpm: %.2f, %.2f", left_rpm, right_rpm);

                        lcd.locate(0, 20);

                        float a0, a1, a2, a3, a4, a5;

                        sensor_array.read_raw(&a0, &a1, &a2, &a3, &a4, &a5);

                        lcd.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", a0, a1, a2, a3, a4,a5);
                            
                        lcd.copy_to_lcd();
                    }
                    break;
                }\

                ///////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////
                ////////////////                        SQUARE  STATE                     ////////////////////
                ///////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////
                case STATE_SQUARE: {
                    // disable bipolar mode
                    left_motor.setBipolarMode(false);
                    right_motor.setBipolarMode(false);

                    // disable motors
                    left_motor.setPower(0.0);
                    right_motor.setPower(0.0);

                    motor_en.write(1);

                    // walk 50cm
                    run_motor_pulses_blocking(SQUARE_STRAIGHT_PULSES, SQUARE_STRAIGHT_PULSES, 0.8);
                    wait(1.0);
                    // right turn
                    run_motor_pulses_blocking(SQUARE_RIGHT_TURN_PULSES, 0, 0.6);
                    wait(1.0);
                    // walk 50cm
                    run_motor_pulses_blocking(SQUARE_STRAIGHT_PULSES, SQUARE_STRAIGHT_PULSES, 0.8);
                    wait(1.0);
                    // right turn
                    run_motor_pulses_blocking(SQUARE_RIGHT_TURN_PULSES, 0, 0.6);
                    wait(1.0);
                    // walk 50cm
                    run_motor_pulses_blocking(SQUARE_STRAIGHT_PULSES, SQUARE_STRAIGHT_PULSES, 0.8);
                    wait(1.0);
                    // right turn
                    run_motor_pulses_blocking(SQUARE_RIGHT_TURN_PULSES, 0, 0.6);
                    wait(1.0);
                    // walk 50cm
                    run_motor_pulses_blocking(SQUARE_STRAIGHT_PULSES, SQUARE_STRAIGHT_PULSES, 0.8);
                    wait(1.0);
                    // U-turn
                    run_motor_pulses_blocking(U_TURN_PULSES, -U_TURN_PULSES, 0.6);
                    wait(1.0);
                    // walk 50cm
                    run_motor_pulses_blocking(SQUARE_STRAIGHT_PULSES, SQUARE_STRAIGHT_PULSES, 0.8);
                    wait(1.0);
                    // left turn
                    run_motor_pulses_blocking(0, SQUARE_LEFT_TURN_PULSES, 0.6);
                    wait(1.0);
                    // walk 50cm
                    run_motor_pulses_blocking(SQUARE_STRAIGHT_PULSES, SQUARE_STRAIGHT_PULSES, 0.8);
                    wait(1.0);
                    // left turn
                    run_motor_pulses_blocking(0, SQUARE_LEFT_TURN_PULSES, 0.6);
                    wait(1.0);
                    // walk 50cm
                    run_motor_pulses_blocking(SQUARE_STRAIGHT_PULSES, SQUARE_STRAIGHT_PULSES, 0.8);
                    wait(1.0);
                    // left turn
                    run_motor_pulses_blocking(0, SQUARE_LEFT_TURN_PULSES, 0.6);
                    wait(1.0);
                    // walk 50cm
                    run_motor_pulses_blocking(SQUARE_STRAIGHT_PULSES, SQUARE_STRAIGHT_PULSES, 0.8);
                    wait(1.0);

                    motor_en.write(0);
                    state = STATE_IDLE;
                    break;
                }

                ///////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////
                ////////////////                        U-TURN  STATE                     ////////////////////
                ///////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////
                case STATE_UTURN:{
                    // disable bipolar mode
                    left_motor.setBipolarMode(false);
                    right_motor.setBipolarMode(false);

                    // disable motors
                    left_motor.setPower(0.0);
                    right_motor.setPower(0.0);

                    motor_en.write(1);

                    wait(0.5);
                    // U-turn
                    run_motor_pulses_blocking(U_TURN_PULSES, -U_TURN_PULSES, 0.6);
                    wait(0.5);

                    motor_en.write(0);
                    state = STATE_IDLE;
                    break;
                }
                default:
                    break;
            };
        };
    }
}
