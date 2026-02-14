#include "global.h"
#include "interface.h"

volatile uint16_t OMRarr_l;
volatile uint16_t OMRarr_r;
volatile int16_t OMRpulse_l;
volatile int16_t OMRpulse_r;

volatile uint16_t last_pulse_l;
volatile uint16_t last_pulse_r;

SpeedController_t get_current_speed(int16_t pulse_l, int16_t pulse_r) {
  SpeedController_t result;
  float delta_l = (float)pulse_l;
  float delta_r = (float)pulse_r;

  float tmp_vect = (delta_l + delta_r) * PULSE_DISTANCE / 2.0f / TIMER_CLOCK;
  float tmp_neko = (delta_l - delta_r) * PULSE_DISTANCE / WHEEL_DISTANCE /
                   TIMER_CLOCK * 180.0f / M_PI;

  result.vect = tmp_vect;
  result.neko = tmp_neko;

  return result;
}

Position_t get_current_position(float dist, float angle, int16_t pulse_l,
                                int16_t pulse_r) {
  Position_t result;

  float delta_l = (float)pulse_l;
  float delta_r = (float)pulse_r;

  float delta_s = (delta_l + delta_r) * PULSE_DISTANCE / 2.0f;
  float delta_deg =
      (delta_l - delta_r) * PULSE_DISTANCE / WHEEL_DISTANCE * 180.0f / M_PI;

  result.dist = dist + delta_s;
  result.angle = angle + delta_deg;

  return result;
}

void reset_current_position(void) {
  current_position.dist = 0.0f;
  current_position.angle = 0.0f;
  reset_odometry_pulses();
  last_pulse_l = 0;
  last_pulse_r = 0;
}

void reset_odometry_pulses(void) {
  OMRpulse_l = 0;
  OMRpulse_r = 0;
}
