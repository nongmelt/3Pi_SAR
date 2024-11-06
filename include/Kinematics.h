/***************************************
 ,        .       .           .       ,.
 |        |       |           |      / |
 |    ,-: |-. ,-. |-. ,-. ,-. |-    '--|
 |    | | | | `-. | | |-' |-' |        |
 `--' `-` `-' `-' ' ' `-' `-' `-'      '
****************************************/
#include "Arduino.h"
#include "Configuration.h"
// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <math.h>

// These two commands mean that this header file
// will attempt to use some global variables of
// the same name found in another header file.
// From encoders.h
extern volatile long count_e0;
extern volatile long count_e1;

// Take the circumference of the wheel and divide by the
// number of counts per revolution. This provides the mm
// travelled per encoder count.
const float MM_PER_COUNT = (2.0 * WHEEL_RADIUS * PI) / COUNT_PER_REV;

// Class to track robot position.
class Kinematics_c {
public:
  // Pose
  float x, y, theta;
  float prev_x, prev_y, prev_theta;

  // Speed
  float speed_left, speed_right, angular_velocity;
  float total_dist_left, total_dist_right;

  // To calculate the difference
  // in encoder counts for each
  // call to update()
  long last_e1, last_e0;
  long last_e1_speed, last_e0_speed;

  // Constructor, must exist.
  Kinematics_c() {}

  // Used to setup kinematics, and to set a start position
  void initialise(float start_x, float start_y, float start_th) {
    last_e0 = count_e0; // Initisalise last count to current count
    last_e1 = count_e1; // Initisalise last count to current count
    last_e0_speed = count_e0;
    last_e1_speed = count_e1;
    x = start_x;
    y = start_y;
    theta = start_th;

    speed_left = 0.0f;
    speed_right = 0.0f;
    angular_velocity = 0.0f;
    total_dist_left = 0.0f;
    total_dist_right = 0.0f;
    prev_dist_left = 0.0f;
    prev_dist_right = 0.0f;
    prev_x = start_x;
    prev_y = start_y;
    prev_theta = start_th;

    last_speed_update_time = millis();
  }

  // Here I have opted to use encoder counts rather than
  // wheel velocity.  Either way will work.
  // With velocity, the difference in time between updates
  // is required (distance = speed / time ).
  // If we use the velocity, it means we have to do
  // extra computation just to get back to distance, which
  // we had in the first place (as change of encoder counts)
  void update() {

    long delta_e1, delta_e0; // change in counts for pose
    float mean_delta;

    float x_contribution;  // linear translation
    float th_contribution; // rotation

    // How many counts since last update()?
    delta_e1 = count_e1 - last_e1;
    delta_e0 = count_e0 - last_e0;

    // Used last encoder values, so now update to
    // current for next iteration
    last_e1 = count_e1;
    last_e0 = count_e0;

    // Work out x contribution in local frame.
    mean_delta = (float)delta_e1;
    mean_delta += (float)delta_e0;
    mean_delta /= 2.0;

    x_contribution = mean_delta * MM_PER_COUNT;

    // Work out rotation in local frame
    th_contribution = (float)delta_e0;
    th_contribution -= (float)delta_e1;
    th_contribution *= MM_PER_COUNT;
    th_contribution /= (WHEEL_SEP * 2.0);

    // Update global frame by taking these
    // local contributions, projecting a point
    // and adding to global pose.
    x = x + x_contribution * cos(theta);
    y = y + x_contribution * sin(theta);
    theta = theta + th_contribution;

    // Normalize theta to [-π, π]
    theta = atan2(sin(theta), cos(theta));

    unsigned long elapsed_time = millis() - last_speed_update_time;

    if (elapsed_time >= SPEED_EST_INTERVAL_MS) {
      long delta_e1_speed, delta_e0_speed; // change in counts for speed
      float dist_left, dist_right;

      // How many counts since last update()?
      delta_e1_speed = count_e1 - last_e1_speed;
      delta_e0_speed = count_e0 - last_e0_speed;

      // Used last encoder values, so now update to
      // current for next iteration
      last_e1_speed = count_e1;
      last_e0_speed = count_e0;

      dist_left = (float)delta_e1_speed * MM_PER_COUNT;
      dist_right = (float)delta_e0_speed * MM_PER_COUNT;

      total_dist_left += dist_left;
      total_dist_right += dist_right;

      speed_left =
          lowPassFilter(dist_left, prev_dist_left) / (float)elapsed_time;
      speed_right =
          lowPassFilter(dist_right, prev_dist_right) / (float)elapsed_time;

      prev_dist_left = dist_left;
      prev_dist_right = dist_right;
      last_speed_update_time = millis();
    }

    // Done!
  } // End of update()

private:
  static constexpr float ALPHA = 0.9;

  float prev_dist_left;
  float prev_dist_right;

  unsigned long last_speed_update_time; // timestamp for speed estimation

  float lowPassFilter(float cur, float prev) {
    return (ALPHA * cur + (1.0 - ALPHA) * prev);
  }

}; // End of Kinematics_c class defintion

#endif
