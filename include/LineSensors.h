/***************************************
 ,        .       .           .     ,-.
 |        |       |           |        )
 |    ,-: |-. ,-. |-. ,-. ,-. |-      /
 |    | | | | `-. | | |-' |-' |      /
 `--' `-` `-' `-' ' ' `-' `-' `-'   '--'
****************************************/
#include "Arduino.h"
#include "Configuration.h"
// this #ifndef stops this file
// from being included more than
// once by the compiler.
#ifndef _LINE_SENSORS_H
#define _LINE_SENSORS_H

// Class to operate the line sensors.
class LineSensors_c {

public:
  // Store your readings into this array.
  // You can then access these readings elsewhere
  // by using the syntax line_sensors.readings[n];
  // Where n is a value [0:4]
  float readings[LINE_SENSORS_NUM];

  // Variables to store calibration constants.
  // Make use of these as a part of the exercises
  // in lab sheet 2.
  float minimum[LINE_SENSORS_NUM];
  float maximum[LINE_SENSORS_NUM];
  float range[LINE_SENSORS_NUM];

  // Variable to store the calculated calibrated
  // (corrected) readings. Needs to be updated via
  // a function call, which is completed in
  // lab sheet 2.
  float calibrated[LINE_SENSORS_NUM];

  LineSensors_c() : line_sensor_initialized(0) {}

  // Refer to Lab sheet 2: Approach 1
  // Use this function to setup the pins required
  // to perform an read of the line sensors using
  // the ADC.
  void initialiseForADC() {
    if (!line_sensor_initialized) {
      // Ensure that the IR LEDs are on
      // for line sensing
      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, HIGH);

      // Configure the line sensor pins
      // DN1, DN2, DN3, DN4, DN5.
      for (int sensor = 0; sensor < LINE_SENSORS_NUM; sensor++) {
        pinMode(LINE_SENSOR_PINS[sensor], INPUT_PULLUP);
        maximum[sensor] = 0.0;
        minimum[sensor] = 1023.0;
      }

      current_read_mode = MODE_ADC;
      line_sensor_initialized = 1;
    }

  } // End of initialiseForADC()

  // Part of the Advanced Exercises for Lab sheet 2
  void initialiseForDigital() {
    if (!line_sensor_initialized) {
      // Ensure that the IR LEDs are on
      // for line sensing
      pinMode(EMIT_PIN, INPUT);
      // Configure the line sensor pins
      // DN1, DN2, DN3, DN4, DN5.
      for (int sensor = 0; sensor < LINE_SENSORS_NUM; sensor++) {
        pinMode(LINE_SENSOR_PINS[sensor], INPUT);
        maximum[sensor] = DEFAULT_TIMEOUT;
        minimum[sensor] = 0.0;
      }

      current_read_mode = MODE_DIGITAL;
      line_sensor_initialized = 1;
    }

  } // End of initialiseForDigital()

  void readSensors() {
    if (current_read_mode == MODE_ADC) {
      readSensorsADC();
    } else if (current_read_mode == MODE_DIGITAL) {
      readSensorsDigital();
    }
  }

  void calibration() {
    readSensors();
    for (int sensor = 0; sensor < LINE_SENSORS_NUM; sensor++) {
      float cur_reading = readings[sensor];

      if (cur_reading > maximum[sensor]) {
        maximum[sensor] = cur_reading;
      }

      if (cur_reading < minimum[sensor]) {
        minimum[sensor] = cur_reading;
      }
    }
  }
  void calcCalibrated() {
    if (current_read_mode == MODE_ADC) {
      calcCalibratedADC();
    } else if (current_read_mode == MODE_DIGITAL) {
      calcCalibratedDigital();
    }
  }

  void postCalibrated() {
    for (int sensor = 0; sensor < LINE_SENSORS_NUM; sensor++) {
      range[sensor] = maximum[sensor] - minimum[sensor];
    }
  }

private:
  static const unsigned long DEFAULT_TIMEOUT = 4000;
  unsigned char line_sensor_initialized;
  enum LineSensorReadMode { MODE_ADC, MODE_DIGITAL };

  LineSensorReadMode current_read_mode;

  // Refer to Lab sheet 2: Approach 1
  // Fix areas marked ????
  // This function is as simple as using a call to
  // analogRead()
  void readSensorsADC() {

    // First, initialise the pins.
    // You need to complete this function (above).
    initialiseForADC();

    for (int sensor = 0; sensor < LINE_SENSORS_NUM; sensor++) {
      readings[sensor] = analogRead(LINE_SENSOR_PINS[sensor]);
    }

  } // End of readSensorsADC()

  // Use this function to apply the calibration values
  // that were captured in your calibration routine.
  // Therefore, you will need to write a calibration
  // routine (see Lab sheet 2)
  void calcCalibratedADC() {

    // Get latest readings (raw values)
    readSensorsADC();

    // Apply calibration values, store in calibrated[]
    for (int sensor = 0; sensor < LINE_SENSORS_NUM; sensor++) {
      calibrated[sensor] = (readings[sensor] - minimum[sensor]) / range[sensor];
    }

  } // End of calcCalibratedADC()

  // Part of the Advanced Exercises for Labsheet 2
  void readSensorsDigital() {
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, HIGH);

    for (int sensor = 0; sensor < LINE_SENSORS_NUM; sensor++) {
      pinMode(LINE_SENSOR_PINS[sensor], OUTPUT);
      digitalWrite(LINE_SENSOR_PINS[sensor], HIGH);
      readings[sensor] = DEFAULT_TIMEOUT;
    }
    delayMicroseconds(10);

    // Capture start time in microseconds
    unsigned long start_time = micros();

    for (int sensor = 0; sensor < LINE_SENSORS_NUM; sensor++) {
      pinMode(LINE_SENSOR_PINS[sensor], INPUT);
    }
    while (true) {
      unsigned long time = micros() - start_time;
      if (time >= DEFAULT_TIMEOUT) {
        break;
      }
      for (int sensor = 0; sensor < LINE_SENSORS_NUM; sensor++) {
        if (digitalRead(LINE_SENSOR_PINS[sensor]) == LOW &&
            time < readings[sensor]) {
          readings[sensor] = time;
        }
      }
    }

    pinMode(EMIT_PIN, INPUT);

  } // End of readSensorsDigital()

  void calcCalibratedDigital() {

    readSensorsDigital();

    // Apply calibration values, store in calibrated[]
    for (int sensor = 0; sensor < LINE_SENSORS_NUM; sensor++) {
      calibrated[sensor] = (readings[sensor] - minimum[sensor]) / range[sensor];
    }
  }
}; // End of LineSensor_c class defintion

#endif
