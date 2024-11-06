/*************************************
  ____   ___  ____   ___ _____   _____ ____  __  __
 |  _ \ / _ \| __ ) / _ \_   _| |  ___/ ___||  \/  |
 | |_) | | | |  _ \| | | || |   | |_  \___ \| |\/| |
 |  _ <| |_| | |_) | |_| || |   |  _|  ___) | |  | |
 |_| \_\\___/|____/ \___/ |_|   |_|   |____/|_|  |_|

*************************************/

#include "Arduino.h"
#include "Configuration.h"

// These #include commands essentially "copy and paste"
// the above .h files (tabs above) into your code here.
// #include "Buzzer.h"       // Labsheet 0
#include "Kinematics.h" // Labsheet 4
// #include "LED.h"          // Labsheet 0
#include "LineSensors.h"  // Labsheet 2
#include "Magnetometer.h" // Labsheet 3
#include "Motors.h"       // Labsheet 1
#include "PID.h"          // Labsheet 1 - Advanced

// Encoders.h does not need modifying.
#include "Encoders.h" // For encoder counts

#ifdef ENABLE_DISPLAY
#include <PololuOLED.h>
PololuSH1106 display(1, 30, 0, 17, 13);
#endif

#ifndef _ROBOT_FSM_H
#define _ROBOT_FSM_H

struct PolarCoordinate {
  float angle; // The angle to rotate in rad
  float dist;  // The distance to translate
};

struct Point {
  float x;
  float y;
};

class RobotFSM_c {

public:
  // Used for Lab sheet 0. Defines which pin the buzzer
  // is attached to. #define works like a
  // find-and-replace through your code.
  // See Labs sheet 0.
  // Buzzer_c buzzer;
  // LEDYellow_c led_y;

  // Instance of a class to operate motors.
  // A class is like a template, and we name
  // an "instance" to use.
  // You can recognise a class type in the
  // code by the convention of "_c"
  // You will need to complete the class.
  // See Lab sheet 1.
  Motors_c motors;

  // Instance of a class to operate the line
  // sensors to measure the surface reflectance.
  // You will need to complete the class.
  // See Lab sheet 2.
  LineSensors_c line_sensors;

  // Instance of a class to operate the magnetometer.
  // Completing the class is a later exercise in
  // Labsheet 3, so you can leave this commented
  // out.
  // See Lab sheet 3.
  Magnetometer_c magnetometer;

  // Instance of a class to estimate the pose
  // of the robot.  You will need to calibrate
  // this, and potentially improve it.
  // See Lab sheet 4.
  Kinematics_c pose;

  // PID Controller
  PID_c left_pid;
  PID_c right_pid;
  PID_c straight_pid;
  PID_c rotation_pid;

  enum class State {
    IDLE,
    CALIBRATION,
    LEAVE_START_AREA,
    SEARCH,
    RETURN_HOME,
    RESCUE,
    COMPLETED,
  };

  // Function pointer type for command callbacks
  typedef void (RobotFSM_c::*CommandCallBack)(float);
  typedef bool (RobotFSM_c::*CompletedCallBack)();

  struct Command {
    CommandCallBack cb;
    CompletedCallBack comp;
    float value;
  };

  // Function pointer type for state callbacks
  typedef void (RobotFSM_c::*StateCallBack)();

  RobotFSM_c() : current_state(State::IDLE) {}

  void initialise() {
    while (!magnetometer.initialise()) {
      Serial.println("Failed to detect and initialize magnetometer!");
      delay(1000);
    }

    // Setup up the buzzer as an output for
    // This is used in Lab sheet 0
    // buzzer.initialise();
    // led_y.initialise();

    // Setup motors.  This is calling a function
    // from within the Motors_c class. You can
    // review this inside Motors.h
    // Complete by working through Lab sheet 1.
    motors.initialise();

    // Setup the line sensors.  This is calling a
    // function from within the LineSensors_c
    // class. You can review this inside
    // LineSensors.h.
    // Complete by working through Lab sheet 2.
    line_sensors.initialiseForADC();
    // line_sensors.initialiseForDigital();

    // These two functions are in encoders.h and
    // they activate the encoder sensors to
    // measure the wheel rotation.  You do not
    // need to change or update these.
    // These are used in Labsheet 4.
    // Encoder counts are counted automatically.
    setupEncoder0();
    setupEncoder1();

    left_pid.initialise(K_P, K_D, K_I);
    right_pid.initialise(K_P, K_D, K_I);
    straight_pid.initialise(K_P_STR, K_D_STR, K_I_STR);
    rotation_pid.initialise(K_P_ROT, K_D_ROT, K_I_ROT);

    // Setup the pose (kinematics). This is calling
    // a function within the Kinematics_c class.
    // You can review this within Kinematics.h
    // This is used in Labsheet 4.
    pose.initialise(0.0f, 0.0f, 0.0f);
    bound_x_min = -55.0f;
    bound_y_min = -200.0f;
    bound_x_max = 320.0f;
    bound_y_max = -460.0f;
    prev_x_point = 0.0f; // for relative distance calculate
    prev_y_point = 0.0f; // for relative distance calculate

    // Remember to reset your PID if you have
    // used any delay()
    left_pid.reset();
    right_pid.reset();
    straight_pid.reset();
    rotation_pid.reset();

    target_speed_left = sqrt(-1.0);  // set to NaN
    target_speed_right = sqrt(-1.0); // set to NaN
    target_angle = sqrt(-1.0);       // set to NaN
    target_dist = sqrt(-1.0);        // set to NaN

    last_pid_update_time = millis();
    last_pose_update_time = millis();

#ifdef ENABLE_DISPLAY
    display.noAutoDisplay();
    display.setLayout21x8();

    last_display_update_time = millis();
#endif
  }

  void setState(State new_state) {
    if (current_state != new_state) {
      StateCallBack exit_cb = getExitCallback(current_state);
      if (exit_cb) {
        (this->*exit_cb)();
      }

      current_state = new_state;

      StateCallBack enter_cb = getEnterCallback(current_state);
      if (enter_cb) {
        (this->*enter_cb)();
      }

      last_update_time = millis();
      // if ((new_state == State::REPEAT_SEARCH &&
      //      current_state == State::SEARCH) ||
      //     (new_state == State::SEARCH &&
      //      current_state == State::REPEAT_SEARCH)) {
      //   return;
      // }

      enter_state_time = millis();
    }
  }

  void update() {
    // buzzer.update();
    // led_y.update();

    if (millis() - last_update_time >= 10) {
      if (current_state > State::CALIBRATION) {
        line_sensors.calcCalibrated();
        magnetometer.calcCalibrated();
      }
      last_update_time = millis();
    }

    if (millis() - last_pose_update_time >= POSE_EST_INTERVAL_MS) {
      pose.update();
      last_pose_update_time = millis();
    }

    if (millis() - last_pid_update_time >= PID_UPDATE_INTERVAL_MS) {
      if (motors.isActive()) {
        speedControllerUpdate();
      }
      last_pid_update_time = millis();
    }
#ifdef ENABLE_DISPLAY
    if (millis() - last_display_update_time >= DISPLAY_INTERVAL_MS) {
      displayUpdate();
      last_display_update_time = millis();
    }
#endif

    StateCallBack update_cb = getUpdateCallback(current_state);

    if (update_cb) {
      (this->*update_cb)();
    }
  }

private:
  State current_state;
  // refresh when enter state, for timer in each state
  unsigned long last_update_time;
  // refresh when enter state, keep track elapsed time
  unsigned long enter_state_time;
  unsigned long last_pid_update_time;
  unsigned long last_pose_update_time;

  float target_speed_left, target_speed_right, target_angle, target_dist;
  float relative_dist_left, relative_dist_right;
  float bound_x_min, bound_x_max;
  float bound_y_min, bound_y_max;
  float prev_x_point, prev_y_point;

  Command commands[MAX_COMMANDS];
  uint8_t command_num = 0;
  int command_index = -1;
  uint8_t rotation_num = 0;

  Point target_point{0, 0};
  Point magnet_point{sqrt(-1), sqrt(-1)};

#ifdef ENABLE_DISPLAY
  unsigned long last_display_update_time;

  const char *stateToString(State state) {
    if (state == State::IDLE) {
      return "IDLE";
    } else if (state == State::CALIBRATION) {
      return "CALIBRATION";
    } else if (state == State::LEAVE_START_AREA) {
      return "LEAVE_START";
    } else if (state == State::SEARCH) {
      return "SEARCH";
    } else if (state == State::RETURN_HOME) {
      return "RETURN_HOME";
    } else if (state == State::RESCUE) {
      return "RESCUE";
    } else if (state == State::COMPLETED) {
      return "COMPLETED";
    }

    return "UNK";
  }
#endif

  StateCallBack getUpdateCallback(State state) {
    if (state == State::IDLE) {
      return &RobotFSM_c::onIdle;
    } else if (state == State::CALIBRATION) {
      return &RobotFSM_c::onCalibration;
    } else if (state == State::LEAVE_START_AREA) {
      return &RobotFSM_c::onLeaveStartArea;
    } else if (state == State::SEARCH) {
      return &RobotFSM_c::onSearch;
    } else if (state == State::RETURN_HOME) {
      return &RobotFSM_c::onReturnHome;
    } else if (state == State::RESCUE) {
      return &RobotFSM_c::onRescue;
    } else if (state == State::COMPLETED) {
      return &RobotFSM_c::onCompleted;
    }
    return nullptr;
  }

  StateCallBack getEnterCallback(State state) {
    if (state == State::CALIBRATION) {
      return &RobotFSM_c::enterCalibration;
    } else if (state == State::LEAVE_START_AREA) {
      return &RobotFSM_c::enterLeaveStartArea;
    } else if (state == State::SEARCH) {
      return &RobotFSM_c::enterSearch;
    } else if (state == State::RETURN_HOME) {
      return &RobotFSM_c::enterReturnHome;
    } else if (state == State::RESCUE) {
      return &RobotFSM_c::enterRescue;
    }
    return nullptr;
  }

  StateCallBack getExitCallback(State state) {
    if (state == State::CALIBRATION) {
      return &RobotFSM_c::exitCalibration;
    } else if (state == State::LEAVE_START_AREA) {
      return &RobotFSM_c::exitLeaveStartArea;
    } else if (state == State::SEARCH) {
      return &RobotFSM_c::exitSearch;
    }
    return nullptr;
  }

// debugging function
#ifdef ENABLE_DISPLAY
  void displayUpdate() {
    display.gotoXY(0, 0);
    display.print("x:");
    display.print(pose.x);
    display.gotoXY(10, 0);
    display.print("y:");
    display.print(pose.y);
    display.gotoXY(0, 1);
    display.print("th:");
    display.print(pose.theta * RAD_TO_DEG);
    display.gotoXY(0, 2);
    display.print("->x:");
    display.print(target_point.x);
    display.gotoXY(5, 2);
    display.print("->y:");
    display.print(target_point.y);
    display.gotoXY(0, 4);
    display.print("mag_m:");
    display.print(magnetometer.measurement);
    display.gotoXY(0, 5);
    display.print("mag_th:");
    display.print(
        atan2(magnetometer.calibrated[1], magnetometer.calibrated[0]) *
        RAD_TO_DEG);
    display.gotoXY(17, 7);
    display.print(motors.modeToString(motors.current_mode));
    display.gotoXY(0, 7);
    display.print(stateToString(current_state));
    display.display();
  }
#endif

  void addCommand(CommandCallBack cb, CompletedCallBack comp, float value) {
    if (command_num < MAX_COMMANDS) {
      commands[command_num] = {cb, comp, value};
      command_num++;
    }
  }

  void clearCommand() {
    command_num = 0;
    command_index = -1;
  }

  PolarCoordinate calcPolarCoordinate(float x1, float y1, float x2, float y2,
                                      float theta) {
    PolarCoordinate coord;
    coord.angle = atan2(y2 - y1, x2 - x1);
    if (coord.angle > 0.0f) {
      coord.angle -= -PI;
    } else if (coord.angle < 0.0f) {
      coord.angle += PI;
    }

    coord.angle = atan2(sin(coord.angle), cos(coord.angle));

    float xx = (x2 - x1) * (x2 - x1);
    float yy = (y2 - y1) * (y2 - y1);
    coord.dist = sqrtf(xx + yy);
    return coord;
  }

  void setPattern(int pattern_type) {
    clearCommand();
    if (pattern_type == 1) {
      // Leave start pattern
      addCommand(&RobotFSM_c::rotate, &RobotFSM_c::rotateCompleted, -88.0f);
      addCommand(&RobotFSM_c::translateForward, &RobotFSM_c::translateCompleted,
                 200.0f);

      return;

    } else if (pattern_type == 2) {
      // Search pattern, deprecated
      float _angle = pose.theta + 88.0f * DEG_TO_RAD;
      addCommand(&RobotFSM_c::translateForward,
                 &RobotFSM_c::frontLineSensorDetected, 260.0f);
      addCommand(&RobotFSM_c::rotate, &RobotFSM_c::rotateCompleted,
                 atan2(sin(_angle), cos(_angle)));
      addCommand(&RobotFSM_c::rotate, &RobotFSM_c::rotateCompleted, -88.0f);
      return;
    } else if (pattern_type == 3) {
      // Go to x, y coordinates, make sure setTargetPoint first
      PolarCoordinate cmd = calcPolarCoordinate(target_point.x, target_point.y,
                                                pose.x, pose.y, pose.theta);
      addCommand(&RobotFSM_c::rotate, &RobotFSM_c::rotateCompleted,
                 RAD_TO_DEG * cmd.angle);
      addCommand(&RobotFSM_c::translateForward, &RobotFSM_c::translateCompleted,
                 cmd.dist);
      return;
    }
  }

  void setTargetSpeed(float left_demand, float right_demand) {
    if (left_demand > 0) {
      target_speed_left =
          clamp(left_demand, SPEED_MINIMUM_MS, SPEED_MAXIMUM_MS);
    } else if (left_demand < 0) {
      target_speed_left =
          clamp(left_demand, -SPEED_MAXIMUM_MS, -SPEED_MINIMUM_MS);
    } else {
      target_speed_left = left_demand;
    }

    if (right_demand > 0) {
      target_speed_right =
          clamp(right_demand, SPEED_MINIMUM_MS, SPEED_MAXIMUM_MS);
    } else if (right_demand < 0) {
      target_speed_right =
          clamp(right_demand, -SPEED_MAXIMUM_MS, -SPEED_MINIMUM_MS);
    } else {
      target_speed_right = right_demand;
    }
  }

  void speedControllerUpdate() {
    if (!motors.isActive()) {
      left_pid.reset();
      right_pid.reset();
      straight_pid.reset();
      rotation_pid.reset();
      return;
    }
    float rot_correction = 0.0f; // speed
    if (!isnan(target_angle) &&
        (motors.current_mode == Motors_c::MotorMode::CW ||
         motors.current_mode == Motors_c::MotorMode::CCW)) {

      rot_correction = rotation_pid.update(target_angle, pose.theta);
      setTargetSpeed(-rot_correction, rot_correction);
      (rot_correction >= 0.0f) ? motors.setTurnCCW() : motors.setTurnCW();
    }
    float l_pwm = left_pid.update(target_speed_left, pose.speed_left);
    float r_pwm = right_pid.update(target_speed_right, pose.speed_right);

    motors.setPWM(l_pwm, r_pwm);
  }

  void setTargetAngle(float angle_demand) { target_angle = angle_demand; }

  void setTargetDist(float dist_demand) { target_dist = dist_demand; }

  void setTargetPoint(Point point) { target_point = point; }

  void rotate(float demand) {
    demand *= DEG_TO_RAD; // change to rad
    float angle_diff = demand - pose.theta;
    setTargetAngle(demand);
    if (demand >= 0.0f && demand <= PI) {
      (angle_diff >= 0.0f) ? motors.setTurnCCW() : motors.setTurnCW();
    } else if (demand >= -PI && demand <= -0.0f) {
      (angle_diff >= 0.0f) ? motors.setTurnCW() : motors.setTurnCCW();
    }
  }

  void rotateCCW(float demand) {
    demand *= DEG_TO_RAD; // change to rad
    setTargetAngle(demand);
    motors.setTurnCCW();
  }

  void rotateCW(float demand) {
    demand *= DEG_TO_RAD; // change to rad
    setTargetAngle(demand);
    motors.setTurnCW();
  }

  void translateForward(float demand) {
    setTargetDist(demand);
    motors.setForwards();
    setTargetSpeed(0.25f, 0.25f);
    // mark the starting point
    prev_x_point = pose.x;
    prev_y_point = pose.y;
    // save current distance for straight line control
    relative_dist_left = pose.total_dist_left;
    relative_dist_right = pose.total_dist_right;
  }

  void stop() {
    motors.setStop();
    setTargetSpeed(sqrt(-1.0), sqrt(-1.0));
    setTargetAngle(sqrt(-1.0));
    setTargetDist(sqrt(-1.0));
  }

  bool rotateCompleted() {
    return (abs(pose.theta - target_angle) <= TARGET_ROTATION_THRESHOLD);
  }

  bool translateCompleted() {
    float xx = (pose.x - prev_x_point) * (pose.x - prev_x_point);
    float yy = (pose.y - prev_y_point) * (pose.y - prev_y_point);
    float current_dist = sqrtf(xx + yy);
    if ((abs(current_dist - target_dist) <= TARGET_TRANSLATION_THRESHOLD)) {
      prev_x_point = 0.0f;
      prev_y_point = 0.0f;
      return true;
    }
    return false;
  }

  bool frontLineSensorDetected() {
    return line_sensors.calibrated[2] >= LINE_SENSORS_THRESHOLD;
  }

  bool frontAverageLineSensorDetected() {
    float avg = (line_sensors.calibrated[1] * 0.15f +
                 line_sensors.calibrated[2] * 0.70f +
                 line_sensors.calibrated[3] * 0.15f);
    return avg >= LINE_SENSORS_THRESHOLD;
  }

  bool leftFrontLineSensorDetected() {
    float avg =
        (line_sensors.calibrated[0] + line_sensors.calibrated[1]) / 2.0f;
    return avg >= LINE_SENSORS_THRESHOLD;
  }

  bool rightFrontLineSensorDetected() {
    float avg =
        (line_sensors.calibrated[3] + line_sensors.calibrated[4]) / 2.0f;
    return avg >= LINE_SENSORS_THRESHOLD;
  }

  bool leftLineSensorDetected() {
    return line_sensors.calibrated[0] >= LINE_SENSORS_THRESHOLD;
  }

  bool rightLineSensorDetected() {
    return line_sensors.calibrated[4] >= LINE_SENSORS_THRESHOLD;
  }

  bool allLineSensorDetected() {
    float avg = (line_sensors.calibrated[0] + line_sensors.calibrated[1] +
                 line_sensors.calibrated[2] + line_sensors.calibrated[3] +
                 line_sensors.calibrated[4]) /
                5.0f;
    return avg >= LINE_SENSORS_THRESHOLD;
  }

  bool boundingBoxDetected() { return (pose.y >= bound_y_min); }

  bool magnetDetected() {
    return magnetometer.measurement >= MAGNETOMETER_DETECT_THRESHOLD;
  }

  void onIdle() {
    if (millis() - enter_state_time >= DEFAULT_IDLE_TIMEOUT) {
      setState(State::CALIBRATION);
    }
  }

  void enterCalibration() {
    rotation_num = 0;

    motors.setTurnCCW();
    setTargetSpeed(-0.4f, 0.4f);
  }

  void onCalibration() {
    unsigned long current_time = millis();
    if (pose.theta > 0.0 && pose.prev_theta < 0.0) {
      rotation_num++;

      if (rotation_num == 2) {
        stop();
        setState(State::LEAVE_START_AREA);
      }
    }
    if (current_time - last_update_time >= CALIBRATION_INTERVAL_MS) {
      line_sensors.calibration();
      magnetometer.calibration();
      last_update_time = millis();
    }
    pose.prev_theta = pose.theta;
  }

  void exitCalibration() {
    line_sensors.postCalibrated();
    magnetometer.postCalibrated();
    rotation_num = 0;
    pose.initialise(0.0f, 0.0f, 25.0f * DEG_TO_RAD); // correct pose
  }

  void enterLeaveStartArea() { setPattern(1); }

  void onLeaveStartArea() {
    if (command_index == -1 && command_num > 0) {
      // Get first command
      command_index = 0;
      (this->*commands[command_index].cb)(commands[command_index].value);
    }

    if (command_index > -1 && command_num > 0) {
      if ((this->*commands[command_index].comp)()) {
        if (command_index == 0) {
          pose.initialise(0.0f, 0.0f, -75.0f * DEG_TO_RAD); // correct pose
        }
        stop();

        command_index++;
        if (command_index >= command_num) {
          // All finished
          clearCommand();
          setState(State::SEARCH);
          return;
        } else {
          (this->*commands[command_index].cb)(commands[command_index].value);
        }
      }
    }
  }

  void exitLeaveStartArea() {}

  void enterSearch() { translateForward(260.0f); }

  void onSearch() {
    if (motors.current_mode == Motors_c::MotorMode::FWD) {
      if (pose.theta < 0.0f * DEG_TO_RAD &&
          pose.theta >= -179.0f * DEG_TO_RAD && boundingBoxDetected()) {
        translateForward(260.0f);
      } else {
        if (boundingBoxDetected()) {
          pose.initialise(pose.x, bound_y_min, pose.theta);
          stop();
          float _angle = pose.theta + 88.0f * DEG_TO_RAD;
          _angle = atan2(sin(_angle), cos(_angle));
          rotate(_angle * RAD_TO_DEG);
        } else if (frontAverageLineSensorDetected()) {
          stop();
          float _angle = pose.theta + 50.0f * DEG_TO_RAD;
          _angle = atan2(sin(_angle), cos(_angle));
          rotate(_angle * RAD_TO_DEG);
          if ((pose.theta < 0.0f * DEG_TO_RAD &&
               pose.theta > -90.0f * DEG_TO_RAD) ||
              (pose.theta >= 0.0f * DEG_TO_RAD &&
               pose.theta < 90.0f * DEG_TO_RAD)) {
            pose.initialise(bound_x_max,
                            clamp(pose.y, bound_y_max, bound_y_min),
                            pose.theta);
          } else if ((pose.theta < 179.0f * DEG_TO_RAD &&
                      pose.theta >= 90.0f * DEG_TO_RAD) ||
                     (pose.theta < -90.0f * DEG_TO_RAD &&
                      pose.theta >= -179.0f * DEG_TO_RAD)) {
            pose.initialise(bound_x_min,
                            clamp(pose.y, bound_y_max, bound_y_min),
                            pose.theta);
          } else if (allLineSensorDetected()) {
            stop();
            float _angle = pose.theta + 160.0f * DEG_TO_RAD;
            _angle = atan2(sin(_angle), cos(_angle));
            rotate(_angle * RAD_TO_DEG);
          }
        }
      }

    } else if (motors.current_mode == Motors_c::MotorMode::CW ||
               motors.current_mode == Motors_c::MotorMode::CCW) {
      if (rotateCompleted()) {
        stop();
        if (allLineSensorDetected() || rightLineSensorDetected() ||
            leftLineSensorDetected()) {
          float _angle = pose.theta + 11.0f * DEG_TO_RAD;
          _angle = atan2(sin(_angle), cos(_angle));
          rotate(_angle * RAD_TO_DEG);
        } else {
          translateForward(260.0f);
        }
      }
    }

    if (magnetDetected()) {
      stop();
      clearCommand();
      // x - 33mm, y - 18 mm from center
      if ((pose.theta <= PI / 4.0f && pose.theta >= 0.0f) ||
          (pose.theta >= -PI / 4.0f && pose.theta <= 0.0f)) {
        magnet_point = Point{pose.x + 33.0f, pose.y + 18.0f};
      } else if (pose.theta >= PI / 4.0f && pose.theta <= 3.0f * PI / 4.0f) {
        magnet_point = Point{pose.x - 33.0f, pose.y + 18.0f};
      } else if ((pose.theta < PI && pose.theta >= 3.0f * PI / 4.0f) ||
                 (pose.theta > -PI && pose.theta <= -3.0f * PI / 4.0f)) {
        magnet_point = Point{pose.x - 33.0f, pose.y - 18.0f};
      } else if ((pose.theta >= -3.0f * PI / 4.0f &&
                  pose.theta <= -PI / 4.0f)) {
        magnet_point = Point{pose.x + 33.0f, pose.y - 18.0f};
      }

      setState(State::RETURN_HOME);
    }

    if (millis() - enter_state_time >= DEFAULT_SEARCH_TIMEOUT) {
      setState(State::RETURN_HOME);
    }

    pose.prev_x = pose.x;
    pose.prev_y = pose.y;
  }

  void exitSearch() { stop(); }

  void enterReturnHome() {
    setTargetPoint(Point{0, 0});
    setPattern(3);
  }

  void onReturnHome() {
    leftFrontLineSensorDetected() ? target_speed_left += 0.0016f
                                  : target_speed_left = 0.25f;

    if (command_index == -1 && command_num > 0) {
      // Get first command
      command_index = 0;
      (this->*commands[command_index].cb)(commands[command_index].value);
    }

    if (command_index > -1 && command_num > 0) {
      if ((this->*commands[command_index].comp)() ||
          (frontAverageLineSensorDetected())) {
        stop();
        command_index++;
        if (command_index >= command_num) {
          clearCommand();
          if (isnan(magnet_point.x)) {
            setState(State::COMPLETED);
            return;
          }
          setState(State::RESCUE);
          return;
        } else {
          (this->*commands[command_index].cb)(commands[command_index].value);
        }
      }
    }
  }

  void enterRescue() {
    setTargetPoint(magnet_point);
    setPattern(3);
  }

  void onRescue() {
    if (command_index == -1 && command_num > 0) {
      // Get first command
      command_index = 0;
      (this->*commands[command_index].cb)(commands[command_index].value);
    }

    if (command_index > -1 && command_num > 0) {
      if ((this->*commands[command_index].comp)()) {
        stop();
        command_index++;
        if (command_index >= command_num) {
          clearCommand();
          setState(State::COMPLETED);
          return;
        } else {
          (this->*commands[command_index].cb)(commands[command_index].value);
        }
      }
    }
  }

  void onCompleted() {}
};

#endif