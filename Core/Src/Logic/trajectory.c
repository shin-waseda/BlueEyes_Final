#include "global.h"
#include "logic.h"
#include <math.h>

static float calc_v_from_s(float v0, float a, float s) {
  float v2 = (v0 * v0) + (2.0f * a * s);
  if (v2 < 0)
    return 0;
  return sqrtf(v2);
}

float get_target_v(float current_dist, float target_dist, float target_accel,
                   float max_v, float start_v, float end_v) {
  float remain_dist = target_dist - current_dist;

  if (remain_dist <= 0)
    return end_v;

  float accel_limit_v = calc_v_from_s(start_v, target_accel, current_dist);

  float decel_limit_v = calc_v_from_s(end_v, target_accel, remain_dist);

  float target_v = accel_limit_v;
  if (target_v > max_v)
    target_v = max_v;
  if (target_v > decel_limit_v)
    target_v = decel_limit_v;

  return target_v;
}
