/************************************
  _     _____ ____
 | |   | ____|  _ \
 | |   |  _| | | | |
 | |___| |___| |_| |
 |_____|_____|____/

*************************************/
#include "Arduino.h"
#include "Configuration.h"

#ifndef _LED_H
#define _LED_H

class LED_c {

public:
  uint16_t blink_count;

  LED_c(uint8_t pin, bool inverted = false)
      : blink_count(0), PIN_NUMBER(pin), led_inverted(inverted), led_state(0),
        blink_duration(BLINK_DEFAULT_DURATION_MS),
        blink_interval(BLINK_DEFAULT_INTERVAL_MS), is_blinking(0),
        last_update_time(0.0), next_update_time(0.0), current_mode(MODE_IDLE) {}

  ~LED_c() { reset(); }

  void initialise() {

    pinMode(PIN_NUMBER, OUTPUT);
    reset();

    current_mode = MODE_IDLE;
  }

  void startBlink() {
    is_blinking = 1;
    led_state = led_inverted ? LOW : HIGH;
    digitalWrite(PIN_NUMBER, led_state);
    last_update_time = millis();
    blink_count++;
  }

  void stopBlink() {
    is_blinking = 0;
    led_state = led_inverted ? HIGH : LOW;
    digitalWrite(PIN_NUMBER, led_state);
  }

  void reset() {
    stopBlink();
    blink_count = 0;

    current_mode = MODE_IDLE;
  }

  void setBlinkOnce(uint16_t duration_ms = BLINK_DEFAULT_DURATION_MS) {
    blink_duration = duration_ms;
    current_mode = MODE_ONCE;
    startBlink();
  }

  void setBlinkWithInterval(uint16_t duration_ms = BLINK_DEFAULT_DURATION_MS,
                            uint16_t interval_ms = BLINK_DEFAULT_INTERVAL_MS) {
    blink_duration = duration_ms;
    blink_interval = interval_ms;
    current_mode = MODE_INTERVAL;
    startBlink();
  }

  void update() {
    unsigned long currentTime = millis();
    if (is_blinking && millis() - last_update_time >= blink_duration) {
      stopBlink();

      if (current_mode == MODE_INTERVAL) {
        next_update_time = currentTime + blink_interval;
      } else {
        current_mode = MODE_IDLE;
      }
    }

    if (!is_blinking && current_mode == MODE_INTERVAL &&
        currentTime >= next_update_time) {
      startBlink();
    }
  }

  bool isActive() const { return current_mode != MODE_IDLE; }

private:
  static constexpr uint16_t BLINK_DEFAULT_DURATION_MS = 250.0;
  static constexpr uint16_t BLINK_DEFAULT_INTERVAL_MS = 500.0;
  const uint8_t PIN_NUMBER;
  bool led_inverted;
  uint8_t led_state;
  uint16_t blink_duration;
  uint16_t blink_interval;
  unsigned char is_blinking;
  unsigned long last_update_time;
  unsigned long next_update_time;

  enum BlinkMode { MODE_IDLE, MODE_ONCE, MODE_INTERVAL };

  BlinkMode current_mode;
};

class LEDYellow_c : public LED_c {
public:
  LEDYellow_c() : LED_c(LED_YELLOW_PIN) {}
};

#endif