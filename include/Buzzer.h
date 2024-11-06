/************************************
  ____
 | __ ) _   _ ___________ _ __
 |  _ \| | | |_  /_  / _ \ '__|
 | |_) | |_| |/ / / /  __/ |
 |____/ \__,_/___/___\___|_|

*************************************/
#include "Arduino.h"
#include "Configuration.h"

// this #ifndef stops this file
// from being included more than
// once by the compiler.
#ifndef _BUZZER_H
#define _BUZZER_H

class Buzzer_c {

public:
  Buzzer_c()
      : beep_interval(BUZZER_DEFAULT_INTERVAL_MS),
        beep_duration(BUZZER_DEFAULT_DURATION_MS), beep_value(0), is_beeping(0),
        last_update_time(0.0), next_update_time(0.0), currentMode(MODE_IDLE) {}

  ~Buzzer_c() { reset(); }

  void initialise() {
    pinMode(BUZZER_PIN, OUTPUT);
    reset();

    currentMode = MODE_IDLE;
  }

  void startBeep() {
    analogWrite(BUZZER_PIN, beep_value);
    is_beeping = 1;
    last_update_time = millis();
  }

  void stopBeep() {
    analogWrite(BUZZER_PIN, 0);
    is_beeping = 0;
  }

  void reset() {
    stopBeep();

    currentMode = MODE_IDLE;
  }

  void setBeepOnce(uint8_t value = 1,
                   uint16_t duration_ms = BUZZER_DEFAULT_DURATION_MS) {
    beep_duration = duration_ms;
    beep_value = value;
    currentMode = MODE_ONCE;
    startBeep();
  }

  void setBeepWithInterval(uint8_t value = 1,
                           uint16_t duration_ms = BUZZER_DEFAULT_DURATION_MS,
                           uint16_t interval_ms = BUZZER_DEFAULT_INTERVAL_MS) {
    beep_duration = duration_ms;
    beep_interval = interval_ms;
    beep_value = value;
    currentMode = MODE_INTERVAL;
    startBeep();
  }

  void update() {
    unsigned long currentTime = millis();
    if (is_beeping && millis() - last_update_time >= beep_duration) {
      stopBeep();

      if (currentMode == MODE_INTERVAL) {
        next_update_time = currentTime + beep_interval;
      } else {
        currentMode = MODE_IDLE;
      }
    }

    if (!is_beeping && currentMode == MODE_INTERVAL &&
        currentTime >= next_update_time) {
      startBeep();
    }
  }

  bool isActive() const { return currentMode != MODE_IDLE; }

private:
  static const uint16_t BUZZER_DEFAULT_DURATION_MS = 250.0;
  static const uint16_t BUZZER_DEFAULT_INTERVAL_MS = 1000.0;
  uint16_t beep_interval;
  uint16_t beep_duration;
  uint8_t beep_value;
  unsigned char is_beeping;
  unsigned long last_update_time;
  unsigned long next_update_time;

  enum BuzzerMode { MODE_IDLE, MODE_ONCE, MODE_INTERVAL };

  BuzzerMode currentMode;
};

#endif
