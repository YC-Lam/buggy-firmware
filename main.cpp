#include <cstdlib>
#include <cstring>

#include "mbed.h"
#include "C12832.h"
#include "QEI.h"
#include "motor.h"
#include "pot.h"

/// number of pulses when running square
#define SQUARE_TURN_PULSES 1600
#define SQUARE_STRAIGHT_PULSES 2048
#define SQUARE_UTURN_PULSES 1600

#define RUN_TARGET_RPM 360
#define RUN_SQUARE_CORNER_RPM 180

/// enable pin for both motors
DigitalOut motor_en(PB_13);
/// left motor, aka motor A
MotorControl left_motor(PC_8, PC_5, PB_2, PC_10, PC_12, false);
/// right motot, aka motor B
MotorControl right_motor(PA_15, PB_12, PB_14, PC_14, PC_15, true);

/// potentiometer on the mbed shield
Potentiometer pot1(A0, 3.3);
Potentiometer pot2(A1, 3.3);

/// LCD display on the mbed shield
C12832 lcd(D11,D13, D12,D7,D10);

/// serial connection to the bluetooth module
Serial hm10(PA_11, PA_12);
/// buffer for reading from serial
char hm10_buffer[6];
int hm10_buffer_cursor = 0;

///  OneWire pin for the DS2781
DigitalInOut   one_wire_pin(PD_2);

/// state machine states
enum {
    STATE_IDLE, STATE_TEST_MOTOR, STATE_SQUARE
} state;

/// ISR to run when data is received
void hm10_received_isr() 
{
    // get character from serial and increment counter
    hm10_buffer[hm10_buffer_cursor++] = hm10.getc();
    // reset cursor if buffer is full
    if (hm10_buffer_cursor == 5){
        hm10_buffer_cursor = 0;
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
    memset(hm10_buffer, 0, 10);
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
    while(1) {
        // get the current time
        int current_time = t.read_us();

        // check if a state change is required
        if (strcmp(hm10_buffer, "squar") == 0){
            state = STATE_SQUARE;
            memset(hm10_buffer, 0, 5);
        }

        // only run every 1000 us, aka 1kHz
        if (current_time - last_ctl_time >= 1000){
             // update control time
            last_ctl_time = current_time;

            switch (state){
                case STATE_IDLE:
                    break;

                ///////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////
                ////////////////                          TEST  STATE                       ////////////////////
                ///////////////////////////////////////////////////////////////////
                ///////////////////////////////////////////////////////////////////
                case STATE_TEST_MOTOR: {
                    // set to bipolar mode
                    if (!right_motor.isBipolarMode()){
                        right_motor.setBipolarMode(true);
                    }
                    if (!left_motor.isBipolarMode()){
                        left_motor.setBipolarMode(true);
                    }
                    
                    // run every 200 ms
                    if (current_time - last_display_time >= 200000){
                        last_display_time = current_time;

                        float left_duty = pot1.amplitudeNorm();
                        float right_duty = pot2.amplitudeNorm();
                        left_motor.setPWM(left_duty);
                        right_motor.setPWM(right_duty);

                        if (motor_en.read() == 0){
                            motor_en.write(1);
                        }

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

                        lcd.printf("duty: %.2f, %.2f ", left_duty, right_duty);
                            
                        lcd.copy_to_lcd();
                    }
                    break;
                }

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

                    int last_left_pulses = 0;
                    int last_right_pulses = 0;
                    bool left_motor_done, right_motor_done;

                     //first square
                    for (int i=0; i < 3; i++){
                        
                        // straight line

                        left_motor.setForward();
                        right_motor.setForward();

                        left_motor.setPower(0.8);
                        right_motor.setPower(0.8);

                        last_left_pulses = left_motor.getPulses();
                        last_right_pulses = right_motor.getPulses();

                        left_motor_done = false;
                        right_motor_done = false;

                        // walk 500 mm
                        while (1){
                            if (left_motor.getPulses() - last_left_pulses >= SQUARE_STRAIGHT_PULSES){
                                left_motor.setPower(0.0);
                                left_motor_done = true;
                            }
                            if (right_motor.getPulses() - last_right_pulses >= SQUARE_STRAIGHT_PULSES){
                                right_motor.setPower(0.0);
                               right_motor_done = true;
                            }

                            if (right_motor_done && left_motor_done){
                                break;
                            }
                        }

                        // right turn
                        wait(1.0);

                        last_left_pulses = left_motor.getPulses();
                        last_right_pulses = right_motor.getPulses();

                        left_motor_done = true;
                        right_motor_done = false;
                        
                        left_motor.setPower(0.0);
                        right_motor.setPower(0.6);

                        // turn 90 degrees
                        while (1){

                            if (right_motor.getPulses() - last_right_pulses >= SQUARE_TURN_PULSES){
                                right_motor.setPower(0.0);
                                right_motor_done = true;
                            }

                            if (right_motor_done && left_motor_done){
                                break;
                            }
                        }

                        wait(0.5);
                    }

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
