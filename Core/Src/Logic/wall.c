#include "global.h"
#include "logic.h"
#include "params.h"
#include <stdlib.h>

volatile int16_t delta_l_cnt = 0;
volatile int16_t delta_r_cnt = 0;

void get_base_sensor_values(void) { base_sensor_values = sensor_values; }
void get_base_sensor_value(void) { get_base_sensor_values(); }

uint8_t get_wall_info(SensorValues_t s) {
  uint8_t info = 0x00;

  if (s.SensorValueR > WALL_BASE_R)
    info |= 0x44;
  if (s.SensorValueL > WALL_BASE_L)
    info |= 0x11;
  if (s.SensorValueFR > WALL_BASE_FR || s.SensorValueFL > WALL_BASE_FL)
    info |= 0x88;

  return info;
}

uint16_t wall_check(uint16_t sensor_val, uint16_t pre_val, float current_v,
                    uint16_t current_cnt, uint16_t limit) {
  int16_t delta = (int16_t)sensor_val - (int16_t)pre_val;

  if (abs(delta) >= KABEKIRE_SHIKIICHI || sensor_val < (limit / 2)) {
    if (current_v < 100.0f)
      return 0;
    return (uint16_t)(10000.0f / current_v);
  }

  return (current_cnt > 0) ? current_cnt - 1 : 0;
}

void update_wall_state(SensorValues_t s, SensorValues_t pre_s, float v) {
  delta_l_cnt = wall_check(s.SensorValueL, pre_s.SensorValueL, v, delta_l_cnt,
                           WALL_BASE_L);
  delta_r_cnt = wall_check(s.SensorValueR, pre_s.SensorValueR, v, delta_r_cnt,
                           WALL_BASE_R);
}

float side_wall_control(SensorValues_t s, SensorValues_t base_s,
                        float current_v, bool is_ctrl_on, bool vanish_l,
                        bool vanish_r) {
  if (!is_ctrl_on)
    return 0.0f;

  float speed_factor = (current_v > 50.0f) ? (500.0f / current_v) : 1.0f;
  float comp_l =
      (!vanish_l) ? ((int16_t)s.SensorValueL - (int16_t)base_s.SensorValueL) *
                        speed_factor
                  : 0.0f;
  float comp_r =
      (!vanish_r) ? ((int16_t)s.SensorValueR - (int16_t)base_s.SensorValueR) *
                        speed_factor
                  : 0.0f;

  float wall_comp = comp_l - comp_r;
  wall_comp *= SIDE_WALL_NEKO_P;
  return max(min(wall_comp, CTRL_MAX), -CTRL_MAX);
}

bool is_wall_vanishing(int16_t delta_cnt) { return delta_cnt > 0; }