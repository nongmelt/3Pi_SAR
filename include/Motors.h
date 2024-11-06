/************************************
,        .       .           .      ,
|        |       |           |     '|
|    ,-: |-. ,-. |-. ,-. ,-. |-     |
|    | | | | `-. | | |-' |-' |      |
`--' `-` `-' `-' ' ' `-' `-' `-'    '
*************************************/
#include "Arduino.h"
#include "Configuration.h"

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MOTORS_H
#define _MOTORS_H

// Class to operate the motors.
class Motors_c {

public:
  enum class MotorMode { IDLE, FWD, REV, CW, CCW };

  MotorMode current_mode;
  Motors_c() : current_mode(MotorMode::IDLE) {}

  // Use this function to initialise the pins that
  // will control the motors, and decide what first
  // value they should have.
  void initialise() {

    pinMode(MOTOR_LEFT_PWM_PIN, OUTPUT);
    pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM_PIN, OUTPUT);
    pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);

    digitalWrite(MOTOR_LEFT_DIR_PIN, MOTOR_FWD);
    digitalWrite(MOTOR_RIGHT_DIR_PIN, MOTOR_FWD);

    analogWrite(MOTOR_LEFT_PWM_PIN, 0);
    analogWrite(MOTOR_RIGHT_PWM_PIN, 0);

  } // End of initialise()

  // This function will be used to send a power value
  // to the motors.
  //
  // The power sent to the motors is created by the
  // analogWrite() function, which is producing PWM.
  // analogWrite() is intended to use a range between
  // [0:255].
  //
  // This function takes two input arguments: "left_pwr"
  // and "right_pwr", (pwr = power) and they are of the
  // type float. A float might be a value like 0.01, or
  // -150.6
  void setPWM(float left_pwr, float right_pwr) {

    // What should happen if the request for left_pwr
    // is less than 0? Recall, how are these motors
    // operated in terms of the pins used?
    digitalWrite(MOTOR_LEFT_DIR_PIN, left_pwr < 0 ? MOTOR_REV : MOTOR_FWD);

    // What should happen if the request for right_pwr
    // is less than 0? Recall, how are these motors
    // operated in terms of the pins used?
    digitalWrite(MOTOR_RIGHT_DIR_PIN, right_pwr < 0 ? MOTOR_REV : MOTOR_FWD);

    // analogWrite() requires a value in the range
    // [0:255], and note this is positive only!
    // Write some code here to take the value of
    // left_pwr and ensure it is:
    // - positive only
    // - within the range 0 to 255
    // Note, we have used the sign to determine
    // the direction - so we don't care about the
    // sign of the value any more.
    if (left_pwr < 0) {
      left_pwr *= -1;
    }

    if (left_pwr > 0) {
      left_pwr = clamp(left_pwr, MOTOR_MINIMUM_PWM, MOTOR_MAXIMUM_PWM);
    }

    // analogWrite() requires a value in the range
    // [0:255], and note this is positive only!
    // Write some code here to take the value of
    // right_pwr and ensure it is:
    // - positive only
    // - within the range 0 to 255
    // Note, we have used the sign to determine
    // the direction - so we don't care about the
    // sign of the value any more.
    if (right_pwr < 0) {
      right_pwr *= -1;
    }

    if (right_pwr > 0) {
      right_pwr = clamp(right_pwr, MOTOR_MINIMUM_PWM, MOTOR_MAXIMUM_PWM);
    }

    // Lastly, write the requested power value to
    // the motors as a PWM signal.
    // Without the code above, this is likely to
    // produce very unexpected behaviors.
    analogWrite(MOTOR_LEFT_PWM_PIN, left_pwr);
    analogWrite(MOTOR_RIGHT_PWM_PIN, right_pwr);

    // Done!
    return;
  } // End of setPWM()

  void setForwards() { current_mode = MotorMode::FWD; }

  void setBackwards() { current_mode = MotorMode::REV; }

  void setStop() {
    setPWM(0.0, 0.0);
    current_mode = MotorMode::IDLE;
  }

  void setTurnCW() { current_mode = MotorMode::CW; }

  void setTurnCCW() { current_mode = MotorMode::CCW; }

  bool isActive() const { return current_mode != MotorMode::IDLE; }
  MotorMode getMode() { return current_mode; }

  const char *modeToString(MotorMode mode) {
    if (mode == MotorMode::IDLE) {
      return "IDLE";
    } else if (mode == MotorMode::FWD) {
      return "FWD";
    } else if (mode == MotorMode::REV) {
      return "REV";
    } else if (mode == MotorMode::CW) {
      return "CW";
    } else if (mode == MotorMode::CCW) {
      return "CCW";
    }
    return "UNK";
  }

}; // End of Motors_c class definition.

#endif
