/************************************
  ____             __ _                       _   _
 / ___|___  _ __  / _(_) __ _ _   _ _ __ __ _| |_(_) ___  _ __
| |   / _ \| '_ \| |_| |/ _` | | | | '__/ _` | __| |/ _ \| '_ \
| |__| (_) | | | |  _| | (_| | |_| | | | (_| | |_| | (_) | | | |
 \____\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\__|_|\___/|_| |_|
                        |___/
*************************************/
#include "Arduino.h"

#ifndef _CONFIGURATION_H
#define _CONFIGURATION_H

// #define ENABLE_DISPLAY

// PIN definitions.
#define BUZZER_PIN 6

#define LED_YELLOW_PIN LED_BUILTIN
#define LED_GREEN_PIN 30
#define LED_RED_PIN 17

#define MOTOR_LEFT_PWM_PIN 10
#define MOTOR_LEFT_DIR_PIN 16
#define MOTOR_RIGHT_PWM_PIN 9
#define MOTOR_RIGHT_DIR_PIN 15

// Motor parameters
// It is a good idea to limit the maximum power
// sent to the motors. Using #define means we
// can set this value just once here, and it
// can be used in many places in the code below.
#define MOTOR_MAXIMUM_PWM 180.0f
#define MOTOR_MINIMUM_PWM 22.0f
#define MOTOR_FWD LOW
#define MOTOR_REV HIGH
#define SPEED_MINIMUM_MS 0.10f // m/s
#define SPEED_MAXIMUM_MS 1.05f // m/s

// PID controller for motors speed
#define K_P 125.0f
#define K_D 0.0f
#define K_I 0.003f

#define K_P_STR 85.0f
#define K_D_STR 0.1f
#define K_I_STR 0.005f

#define K_P_ROT 0.00316f
#define K_D_ROT 0.0f
#define K_I_ROT 0.0022f

// Line sensors parameters
// We will use all 5 line sensors (DN1 - 5)
// and so define a constant here, rather than
// type '5' in lots of places.
#define LINE_SENSORS_NUM 5
// This is the pin used to turn on the infra-
// red LEDs.
#define EMIT_PIN 11
// Pin definitions
// This time, we will use an array to store the
// pin definitions.  This is a bit like a list.
// This way, we can either loop through the
// list automatically, or we can ask for a pin
// by indexing, e.g. sensor_pins[0] is A11: leftmost,
// sensors_pins[1] is A0.
const int LINE_SENSOR_PINS[LINE_SENSORS_NUM] = {A11, A0, A2, A3, A4};

#define LINE_SENSORS_THRESHOLD 0.70f
#define MAGNETOMETER_DETECT_THRESHOLD 18

// Target threshold
#define TARGET_ROTATION_THRESHOLD DEG_TO_RAD * 10.0f
#define TARGET_TRANSLATION_THRESHOLD 8.5f

// Robot dimensions.  You will need
// to calibrate these to get the best
// performance. (see Lab sheet 4)
#define COUNT_PER_REV 358.3 // From documentation - correct.
#define WHEEL_RADIUS 16.0   // mm
#define WHEEL_SEP 43.5      // mm, from centre of robot to wheel centre

// Update rates and durations for operations
#define SPEED_EST_INTERVAL_MS 8
#define PID_UPDATE_INTERVAL_MS 15
#define CALIBRATION_INTERVAL_MS 10
#define POSE_EST_INTERVAL_MS 10

#ifdef ENABLE_DISPLAY
#define DISPLAY_INTERVAL_MS 10
#endif

// Finite State Machine Configuration
static const uint16_t DEFAULT_IDLE_TIMEOUT = 2000;
static const uint32_t DEFAULT_SEARCH_TIMEOUT = 120000; // 2 mins

// Command for search patterns
const uint8_t MAX_COMMANDS = 8;

template <typename T> T clamp(const T &value, const T &min, const T &max) {
  return (value < min) ? min : (value > max) ? max : value;
}

// Mapping coordinate
const float X_OFFSET = -10.0f;
const float Y_OFFSET = -225.0f;
const float X_SCALE = 60.0f;
const float Y_SCALE = 43.0f;

#endif