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

// maximum differential factor, defines half the maximum difference between two motors
#define MAX_DIFF_FACTOR 140
// turn factor constant, defines how much a turn reduces target speed
#define TURN_FACTOR_CONSTANT 0.8f
// number of counts before stopping buggy in gap
#define GAP_MAX_COUNT 20
// smoothing factor of motor output
#define MOTOR_SMOOTH_FACTOR 0.3f
#define MOTOR_BASE_POWER 0.5f

/// target rpm in different states
#define RUN_TARGET_RPM 250
#define GAP_TARGET_RPM 200
#define UTURN_TARGET_RPM 80

// turnaround tuning constants
#define UTURN_COARSE_PULSES 1000 // tune this on the real track
#define UTURN_ALIGN_RPM 45     // slow speed for final alignment
#define UTURN_CENTER_TOL 3.0f      // mm: how close to centre the line must be
#define UTURN_ALIGN_TIMEOUT 200    // ticks at 200 Hz = 2 s safety cap on phase 1

#define STEER_PID_KP 3.5f
#define STEER_PID_KI 0.0f
#define STEER_PID_KD 0.0f

#define LEFT_PID_KP 0.05f
#define LEFT_PID_KI 0.002f
#define LEFT_PID_KD 0.0f

#define RIGHT_PID_KP 0.05f
#define RIGHT_PID_KI 0.002f
#define RIGHT_PID_KD 0.0f

#define UTURN_PID_KP 0.08f
#define UTURN_LEFT_KP 0.08f
#define UTURN_RIGHT_KP 0.08f
// global timer
Timer global_timer;

/// enable pin for both motors
DigitalOut motor_en(PB_13);
/// left motor, aka motor A
MotorControl left_motor(PC_8, PC_5, PB_2, PC_10, PC_12, false);
/// right motor, aka motor B
MotorControl right_motor(PA_15, PB_12, PB_14, PA_13, PA_14, true);

/// PID controllers
PidControl left_motor_pid(LEFT_PID_KP, LEFT_PID_KI, LEFT_PID_KD, CTL_LOOP_FREQUENCY);
PidControl right_motor_pid(RIGHT_PID_KP, RIGHT_PID_KI, RIGHT_PID_KD, CTL_LOOP_FREQUENCY);
PidControl steering_pid(STEER_PID_KP, STEER_PID_KI, STEER_PID_KD, CTL_LOOP_FREQUENCY);

// pid autotuner
PIDAutotuner pid_auto_tuner;

SensorArray sensor_array(A5, A4, A3, A2, A1, A0, PC_3);

/// LCD display on the mbed shield
C12832 lcd(D11, D13, D12, D7, D10);

/// serial connection to the bluetooth module
Serial hm10(PA_11, PA_12);
DigitalInOut one_wire_pin(PD_2);
/// buffer for reading from serial
char hm10_buffer[BLE_BUFFER_SIZE];
volatile int hm10_buffer_cursor = 0;
volatile bool hm10_command_ready = false;

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
};
volatile int state;

// sensor array multi sampling
int sensor_sample_count = 0;
float sensor_samples[SENSOR_SAMPLE_COUNT] = {};

// previous differential factor / motor powers
float prev_diff_factor = 0.0f;
float prev_left_power = 0.0f;
float prev_right_power = 0.0f;

// counter for gap state
int gap_counter = 0;

// turnaround variables
bool uturn_started = false;
int uturn_left_start_pulses = 0;
int uturn_right_start_pulses = 0;
int uturn_phase = 0;   // 0 = coarse turn, 1 = fine alignment
int uturn_align_counter = 0;

// lcd/debug variables
float lcd_left_rpm = 0.0f;
float lcd_right_rpm = 0.0f;
float lcd_sensor_position = 0.0f;
float lcd_steer_factor = 0.0f;
float lcd_left_target = 0.0f;
float lcd_right_target = 0.0f;

float lcd_tune_kp = 0.0f;
float lcd_tune_ki = 0.0f;
float lcd_tune_kd = 0.0f;

float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void enter_idle_state() {
    state = STATE_IDLE;
    steering_pid.reset();
    left_motor_pid.reset();
    right_motor_pid.reset();
   left_motor_pid.set_kp(LEFT_PID_KP);
    right_motor_pid.set_kp(RIGHT_PID_KP);
    prev_diff_factor = 0.0f;
    prev_left_power = 0.0f;
    prev_right_power = 0.0f;

    motor_en.write(0);
}

void enter_run_state() {
    state = STATE_RUN;
    steering_pid.reset();
    left_motor_pid.reset();
    right_motor_pid.reset();

    left_motor.setBipolarMode(false);
    right_motor.setBipolarMode(false);
    left_motor_pid.set_kp(LEFT_PID_KP);
    right_motor_pid.set_kp(RIGHT_PID_KP);
    left_motor.setForward();
    right_motor.setForward();

    prev_left_power = 0.0f;
    prev_right_power = 0.0f;

    motor_en.write(1);
}

void enter_test_state() {
    state = STATE_TEST_MOTOR;
    steering_pid.reset();
    left_motor_pid.reset();
    right_motor_pid.reset();
    left_motor_pid.set_kp(LEFT_PID_KP);
    right_motor_pid.set_kp(RIGHT_PID_KP);
    left_motor.setBipolarMode(false);
    right_motor.setBipolarMode(false);

    left_motor.setBackward();
    right_motor.setBackward();

    prev_left_power = 0.0f;
    prev_right_power = 0.0f;

    motor_en.write(1);
}

void enter_uturn_state() {
    state = STATE_UTURN;
    steering_pid.reset();
    left_motor_pid.reset();
    right_motor_pid.reset();
    left_motor.setBipolarMode(false);
    right_motor.setBipolarMode(false);
    left_motor_pid.set_kp(UTURN_LEFT_KP);
    right_motor_pid.set_kp(UTURN_RIGHT_KP);
    // opposite directions so the buggy pivots on the spot
    left_motor.setBackward();
    right_motor.setForward();

    prev_diff_factor = 0.0f;
    prev_left_power = 0.0f;
    prev_right_power = 0.0f;

    // reset turnaround phase variables
    uturn_started = false;
    uturn_phase = 0;
    uturn_align_counter = 0;

    motor_en.write(1);
}

void enter_motor_tune_right_state() {
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
    left_motor_pid.set_kp(LEFT_PID_KP);
    right_motor_pid.set_kp(RIGHT_PID_KP);
    left_motor.setBipolarMode(false);
    right_motor.setBipolarMode(false);

    left_motor.setForward();
    right_motor.setForward();

    prev_left_power = 0.0f;
    prev_right_power = 0.0f;

    motor_en.write(1);
}

void enter_motor_tune_left_state() {
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
    left_motor_pid.set_kp(LEFT_PID_KP);
    right_motor_pid.set_kp(RIGHT_PID_KP);
    left_motor.setBipolarMode(false);
    right_motor.setBipolarMode(false);

    left_motor.setForward();
    right_motor.setForward();

    prev_left_power = 0.0f;
    prev_right_power = 0.0f;

    motor_en.write(1);
}

void enter_steer_tune_state() {
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
    left_motor_pid.set_kp(LEFT_PID_KP);
    right_motor_pid.set_kp(RIGHT_PID_KP);
    left_motor.setBipolarMode(false);
    right_motor.setBipolarMode(false);

    left_motor.setForward();
    right_motor.setForward();

    prev_left_power = 0.0f;
    prev_right_power = 0.0f;

    motor_en.write(1);
}

/// BLE receive interrupt
void hm10_received_isr() {
    char c = hm10.getc();

    // still accept newline if app sends it
    if ((c == '\n') || (c == '\r')) {
        hm10_buffer[hm10_buffer_cursor] = '\0';
        hm10_command_ready = true;
        hm10_buffer_cursor = 0;
        return;
    }

    // store character
    if (hm10_buffer_cursor < BLE_BUFFER_SIZE - 1) {
        hm10_buffer[hm10_buffer_cursor++] = c;
        hm10_buffer[hm10_buffer_cursor] = '\0';

        // accept known commands immediately, even without newline
        if (strcmp(hm10_buffer, "turn") == 0 ||
            strcmp(hm10_buffer, "start") == 0 ||
            strcmp(hm10_buffer, "stop") == 0 ||
            strcmp(hm10_buffer, "test") == 0 ||
            strcmp(hm10_buffer, "left") == 0 ||
            strcmp(hm10_buffer, "right") == 0 ||
            strcmp(hm10_buffer, "steer") == 0) {
            hm10_command_ready = true;
            hm10_buffer_cursor = 0;
        }
    } else {
        // overflow protection
        hm10_buffer_cursor = 0;
        hm10_buffer[0] = '\0';
    }
}

// task running at 4kHz to get samples
void sensor_sample_task() {
    float position = sensor_array.read_distance_from_centre();
    sensor_samples[sensor_sample_count++] = position;
    sensor_sample_count %= SENSOR_SAMPLE_COUNT;
}

// main control
void state_machine_task() {
    // get mean position, ignoring NaN samples
    float position_sum = 0.0f;
    int valid_count = 0;

    for (int i = 0; i < SENSOR_SAMPLE_COUNT; i++) {
        if (!isnan(sensor_samples[i])) {
            position_sum += sensor_samples[i];
            valid_count++;
        }
    }

    float position = NAN;
    if (valid_count > 0) {
        position = position_sum / valid_count;
    }

    // get rpm of both motors
    float left_motor_rpm = left_motor.getRPM(CTL_LOOP_FREQUENCY);
    float right_motor_rpm = right_motor.getRPM(CTL_LOOP_FREQUENCY);

    // update lcd variables
    lcd_left_rpm = left_motor_rpm;
    lcd_right_rpm = right_motor_rpm;
    lcd_sensor_position = position;

    switch (state) {
        case STATE_RUN: {
            state_run:

            // check if line is detected
            if (isnan(position)) {
                state = STATE_GAP;
                gap_counter = 0;
                goto state_gap;
            }

            // calculate differential factor
            float diff_factor = steering_pid.update(position);

            // clamp diff factor
            if (diff_factor > MAX_DIFF_FACTOR) diff_factor = MAX_DIFF_FACTOR;
            if (diff_factor < -MAX_DIFF_FACTOR) diff_factor = -MAX_DIFF_FACTOR;

            prev_diff_factor = diff_factor;
            lcd_steer_factor = diff_factor;

            // adjust target speed when turning
            float turn_factor = fabsf(diff_factor) / MAX_DIFF_FACTOR;
            float adjusted_base_target = RUN_TARGET_RPM * (1.0f - TURN_FACTOR_CONSTANT * turn_factor);

            float left_target = adjusted_base_target - diff_factor;
            float right_target = adjusted_base_target + diff_factor;

            lcd_left_target = left_target;
            lcd_right_target = right_target;

            float left_error = left_target - left_motor_rpm;
            float right_error = right_target - right_motor_rpm;

            float left_power = MOTOR_BASE_POWER + left_motor_pid.update(left_error);
            float right_power = MOTOR_BASE_POWER + right_motor_pid.update(right_error);

            left_power = (1.0f - MOTOR_SMOOTH_FACTOR) * left_power + MOTOR_SMOOTH_FACTOR * prev_left_power;
            right_power = (1.0f - MOTOR_SMOOTH_FACTOR) * right_power + MOTOR_SMOOTH_FACTOR * prev_right_power;

            left_power = clampf(left_power, 0.0f, 1.0f);
            right_power = clampf(right_power, 0.0f, 1.0f);

            prev_left_power = left_power;
            prev_right_power = right_power;

            left_motor.setPower(left_power);
            right_motor.setPower(right_power);
            break;
        }

        case STATE_GAP: {
            state_gap:

            // if line comes back, continue running
            if (!isnan(position)) {
                state = STATE_RUN;
                goto state_run;
            }

            gap_counter++;

            // if line lost for long enough, stop
            if (gap_counter >= GAP_MAX_COUNT) {
                enter_idle_state();
                break;
            }

            // keep fading previous steering command
            float diff_factor = 0.95f * prev_diff_factor;
            prev_diff_factor = diff_factor;

            float turn_factor = fabsf(diff_factor) / MAX_DIFF_FACTOR;
            float adjusted_base_target = GAP_TARGET_RPM * (1.0f - TURN_FACTOR_CONSTANT * turn_factor);

            float left_target = adjusted_base_target - diff_factor;
            float right_target = adjusted_base_target + diff_factor;

            float left_error = left_target - left_motor_rpm;
            float right_error = right_target - right_motor_rpm;

            float left_power = MOTOR_BASE_POWER + left_motor_pid.update(left_error);
            float right_power = MOTOR_BASE_POWER + right_motor_pid.update(right_error);

            left_power = (1.0f - MOTOR_SMOOTH_FACTOR) * left_power + MOTOR_SMOOTH_FACTOR * prev_left_power;
            right_power = (1.0f - MOTOR_SMOOTH_FACTOR) * right_power + MOTOR_SMOOTH_FACTOR * prev_right_power;

            left_power = clampf(left_power, 0.0f, 1.0f);
            right_power = clampf(right_power, 0.0f, 1.0f);

            prev_left_power = left_power;
            prev_right_power = right_power;

            left_motor.setPower(left_power);
            right_motor.setPower(right_power);
            break;
        }

        case STATE_UTURN: {
            // one-time setup when first entering turn
            if (!uturn_started) {
                uturn_left_start_pulses = left_motor.getPulses();
                uturn_right_start_pulses = right_motor.getPulses();
                uturn_started = true;
                uturn_phase = 0;
            }

            // how far each wheel has turned since turnaround started
            int left_turned = abs(left_motor.getPulses() - uturn_left_start_pulses);
            int right_turned = abs(right_motor.getPulses() - uturn_right_start_pulses);
            int avg_turned = (left_turned + right_turned) / 2;

            // default coarse turn speed
            float target_rpm = UTURN_TARGET_RPM;

            // phase 0: coarse encoder-based turn
            if (uturn_phase == 0) {
                if (avg_turned >= UTURN_COARSE_PULSES) {
                    uturn_phase = 1;

                    // reset speed PIDs so fine alignment starts cleanly
                    left_motor_pid.reset();
                    right_motor_pid.reset();

                    prev_left_power = 0.0f;
                    prev_right_power = 0.0f;
                    uturn_align_counter = 0;
                    enter_idle_state();
                }
            }

            // phase 1: slow final alignment using line sensor
           /*/ if (uturn_phase == 1) {
                target_rpm = UTURN_ALIGN_RPM;


                // safety: if line is never found, stop instead of spinning forever
                uturn_align_counter++;
                if (uturn_align_counter > UTURN_ALIGN_TIMEOUT) {
                    enter_idle_state();
                    break;
                }

                // if line is detected and near centre, go back to run
                if (!isnan(position) && fabsf(position) < UTURN_CENTER_TOL) {
                    enter_run_state();
                    break;
                }
            }*/

            // regulate both wheels using same speed PID structure
            float left_error = target_rpm - fabsf(left_motor_rpm);
            float right_error = target_rpm - fabsf(right_motor_rpm);

            lcd_left_target = target_rpm;
            lcd_right_target = target_rpm;
            lcd_steer_factor = 0.0f;

            float left_power = MOTOR_BASE_POWER + left_motor_pid.update(left_error);
            float right_power = MOTOR_BASE_POWER + right_motor_pid.update(right_error);

            left_power = (1.0f - MOTOR_SMOOTH_FACTOR) * left_power + MOTOR_SMOOTH_FACTOR * prev_left_power;
            right_power = (1.0f - MOTOR_SMOOTH_FACTOR) * right_power + MOTOR_SMOOTH_FACTOR * prev_right_power;

            left_power = clampf(left_power, 0.0f, 1.0f);
            right_power = clampf(right_power, 0.0f, 1.0f);

            prev_left_power = left_power;
            prev_right_power = right_power;

            left_motor.setPower(left_power);
            right_motor.setPower(right_power);
            break;
        }

        case STATE_TEST_MOTOR: {
            float left_target = RUN_TARGET_RPM;
            float right_target = RUN_TARGET_RPM;

            lcd_left_target = left_target;
            lcd_right_target = right_target;

            float left_error = left_target - left_motor_rpm;
            float right_error = right_target - right_motor_rpm;

            float left_power = MOTOR_BASE_POWER + left_motor_pid.update(left_error);
            float right_power = MOTOR_BASE_POWER + right_motor_pid.update(right_error);

            left_power = (1.0f - MOTOR_SMOOTH_FACTOR) * left_power + MOTOR_SMOOTH_FACTOR * prev_left_power;
            right_power = (1.0f - MOTOR_SMOOTH_FACTOR) * right_power + MOTOR_SMOOTH_FACTOR * prev_right_power;

            left_power = clampf(left_power, 0.0f, 1.0f);
            right_power = clampf(right_power, 0.0f, 1.0f);

            prev_left_power = left_power;
            prev_right_power = right_power;

            left_motor.setPower(left_power);
            right_motor.setPower(right_power);
            break;
        }

        case STATE_MOTOR_TUNE_RIGHT: {
            if (pid_auto_tuner.isFinished()) {
                lcd_tune_kp = pid_auto_tuner.getKp();
                lcd_tune_ki = pid_auto_tuner.getKi();
                lcd_tune_kd = pid_auto_tuner.getKd();
                enter_idle_state();
                break;
            }

            float power = MOTOR_BASE_POWER + pid_auto_tuner.tunePID(right_motor_rpm, global_timer.read_us());
            power = (1.0f - MOTOR_SMOOTH_FACTOR) * power + MOTOR_SMOOTH_FACTOR * prev_left_power;
            power = clampf(power, 0.0f, 1.0f);

            prev_left_power = power;

            left_motor.setPower(0.0f);
            right_motor.setPower(power);
            break;
        }

        case STATE_MOTOR_TUNE_LEFT: {
            if (pid_auto_tuner.isFinished()) {
                lcd_tune_kp = pid_auto_tuner.getKp();
                lcd_tune_ki = pid_auto_tuner.getKi();
                lcd_tune_kd = pid_auto_tuner.getKd();
                enter_idle_state();
                break;
            }

            float power = MOTOR_BASE_POWER + pid_auto_tuner.tunePID(left_motor_rpm, global_timer.read_us());
            power = (1.0f - MOTOR_SMOOTH_FACTOR) * power + MOTOR_SMOOTH_FACTOR * prev_left_power;
            power = clampf(power, 0.0f, 1.0f);

            prev_left_power = power;

            left_motor.setPower(power);
            right_motor.setPower(0.0f);
            break;
        }

        case STATE_STEER_TUNE: {
            if (pid_auto_tuner.isFinished()) {
                lcd_tune_kp = pid_auto_tuner.getKp();
                lcd_tune_ki = pid_auto_tuner.getKi();
                lcd_tune_kd = pid_auto_tuner.getKd();
                enter_idle_state();
                break;
            }

            if (isnan(position)) {
                enter_idle_state();
                break;
            }

            float diff_factor = pid_auto_tuner.tunePID(position, global_timer.read_us());

            diff_factor = 0.6f * diff_factor + 0.4f * prev_diff_factor;

            if (diff_factor > MAX_DIFF_FACTOR) diff_factor = MAX_DIFF_FACTOR;
            if (diff_factor < -MAX_DIFF_FACTOR) diff_factor = -MAX_DIFF_FACTOR;

            prev_diff_factor = diff_factor;
            lcd_steer_factor = diff_factor;

            float turn_factor = fabsf(diff_factor) / MAX_DIFF_FACTOR;
            float adjusted_base_target = RUN_TARGET_RPM * (1.0f - TURN_FACTOR_CONSTANT * turn_factor);

            float left_target = adjusted_base_target - diff_factor;
            float right_target = adjusted_base_target + diff_factor;

            lcd_left_target = left_target;
            lcd_right_target = right_target;

            float left_error = left_target - left_motor_rpm;
            float right_error = right_target - right_motor_rpm;

            float left_power = left_motor_pid.update(left_error);
            float right_power = right_motor_pid.update(right_error);

            left_power = (1.0f - MOTOR_SMOOTH_FACTOR) * left_power + MOTOR_SMOOTH_FACTOR * prev_left_power;
            right_power = (1.0f - MOTOR_SMOOTH_FACTOR) * right_power + MOTOR_SMOOTH_FACTOR * prev_right_power;

            left_power = clampf(left_power, 0.0f, 1.0f);
            right_power = clampf(right_power, 0.0f, 1.0f);

            prev_left_power = left_power;
            prev_right_power = right_power;

            left_motor.setPower(left_power);
            right_motor.setPower(right_power);
            break;
        }

        default:
            break;
    }
}

void lcd_update_task() {
    lcd.cls();
    lcd.locate(0, 0);
    lcd.printf("rpm: %7.2f, %7.2f", lcd_left_rpm, lcd_right_rpm);

    lcd.locate(0, 10);
    lcd.printf("distance: %5.3f, sd: %.3f", lcd_sensor_position, sensor_array.read_sd());

    lcd.locate(0, 20);
    lcd.printf("%.5f, %.5f, %.5f", lcd_tune_kp, lcd_tune_ki, lcd_tune_kd);

    lcd.copy_to_lcd();
}

/// main entry

int main() {
    global_timer.start();

    lcd.cls();
    lcd.set_auto_up(0);

    motor_en.write(0);
    state = STATE_IDLE;

    hm10.baud(9600);
    memset(hm10_buffer, 0, BLE_BUFFER_SIZE);
    hm10.attach(&hm10_received_isr, Serial::RxIrq);

    Ticker state_machine_ticker;
    Ticker lcd_update_ticker;
    Ticker sample_ticker;

    sample_ticker.attach_us(sensor_sample_task, 250);
    state_machine_ticker.attach_us(state_machine_task, CTL_LOOP_PERIOD_US);
    lcd_update_ticker.attach_us(lcd_update_task, 200000);

    while (true) {
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
            } else if (strcmp(hm10_buffer, "right") == 0) {
                enter_motor_tune_right_state();
            } else if (strcmp(hm10_buffer, "left") == 0) {
                enter_motor_tune_left_state();
            } else if (strcmp(hm10_buffer, "steer") == 0) {
                enter_steer_tune_state();
            }

            memset(hm10_buffer, 0, BLE_BUFFER_SIZE);
        }
    }
}