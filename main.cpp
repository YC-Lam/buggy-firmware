#include "mbed.h"
#include "C12832.h"

#include "QEI.h"
#include "motor.h"
#include "pot.h"
#include <cstdlib>
#include <cstring>

#define SQUARE_TURN_PULSES 1600
#define SQUARE_STRAIGHT_PULSES 2048
#define SQUARE_UTURN_PULSES 1600

DigitalOut motor_en(PB_13);
Motor left_motor(PC_8, PC_5, PB_2, PC_10, PC_12, false);
Motor right_motor(PA_15, PB_12, PB_14, PC_14, PC_15, true);

Potentiometer pot1(A0, 3.3);
Potentiometer pot2(A1, 3.3);

C12832 lcd(D11,D13, D12,D7,D10);

Serial hm10(PA_11, PA_12);
char hm10_buffer[6];
int hm10_buffer_cursor = 0;

// OneWire pin for the DS2781
DigitalInOut   one_wire_pin(PD_2);

enum {
    STATE_IDLE, STATE_TEST_MOTOR, STATE_SQUARE
} state;

//ISR to run when data is received
void hm10_received_isr() 
{
    hm10_buffer[hm10_buffer_cursor++] = hm10.getc();
    if (hm10_buffer_cursor == 5){
        hm10_buffer_cursor = 0;
    }
}

int main() {
    lcd.cls();
    lcd.set_auto_up(0);
    
    motor_en.write(0);
    
    hm10.baud(9600);
    memset(hm10_buffer, 0, 10);
    hm10.attach(&hm10_received_isr, Serial::RxIrq);

    state = STATE_TEST_MOTOR;

    int last_left_pulses = 0;
    int last_right_pulses = 0;

    Timer t;
    int last_ctl_time = 0;
    int last_display_time = 0;

    t.start();

    while(1) {
        int current_time = t.read_us();

        if (strcmp(hm10_buffer, "squar") == 0){
            state = STATE_SQUARE;
            memset(hm10_buffer, 0, 5);
        }

        if (current_time - last_ctl_time >= 1000){
             // update control time
            last_ctl_time = current_time;

            switch (state){
                case STATE_IDLE:
                    break;
                case STATE_TEST_MOTOR:
                    {
                        right_motor.setBipolar(1);
                        left_motor.setBipolar(1);
                        
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

                            int left_rpm = ((left - last_left_pulses) *5 * 300) / 1024;
                            int right_rpm = ((right - last_right_pulses) *5 * 300) / 1024;

                            lcd.printf("rpm: %d, %d", left_rpm, right_rpm);

                            lcd.locate(0, 20);

                            lcd.printf("duty: %.2f, %.2f ", left_duty, right_duty);
                                
                            lcd.copy_to_lcd();

                            last_left_pulses = left;
                            last_right_pulses = right;
                        }
                        break;
                    }
                    break;
                case STATE_SQUARE: {
                    left_motor.setBipolar(0);
                    right_motor.setBipolar(0);

                    left_motor.setPWM(1.0);
                    right_motor.setPWM(1.0);

                    motor_en.write(1);

                    bool left_motor_done, right_motor_done;

                     //first square
                    for (int i=0; i < 3; i++){
                        
                        // straight line

                        left_motor.setDir(1);
                        right_motor.setDir(0);

                        left_motor.setPWM(0.2);
                        right_motor.setPWM(0.2);

                        last_left_pulses = left_motor.getPulses();
                        last_right_pulses = right_motor.getPulses();

                        left_motor_done = false;
                        right_motor_done = false;

                        // walk 500 mm
                        while (1){
                            if (left_motor.getPulses() - last_left_pulses >= SQUARE_STRAIGHT_PULSES){
                                left_motor.setPWM(1.0);
                                left_motor_done = true;
                            }
                            if (right_motor.getPulses() - last_right_pulses >= SQUARE_STRAIGHT_PULSES){
                                right_motor.setPWM(1.0);
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
                        
                        left_motor.setPWM(1.0);
                        right_motor.setPWM(0.4);

                        // turn 90 degrees
                        while (1){

                            if (right_motor.getPulses() - last_right_pulses >= SQUARE_TURN_PULSES){
                                right_motor.setPWM(1.0);
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
                break;
                default:
                    break;
            };
        };
    }
}
