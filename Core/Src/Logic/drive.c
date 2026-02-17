#include "global.h"
#include "interface.h"
#include "logic.h"
#include "params.h"

void drive_calc_offset(float dist) {
  enable_motor();
  reset_current_position();

  output_speed.vect = target_speed.vect;

  output_speed.target_catnip = 0;
  bool done = 1;
  drive_start();
  while (current_position.dist < dist) {
    if (done) {
      conf_route_dijkstra();
      done = 0;
    }
  }
  drive_stop();
  MF.FLAG.CALC_OFFSET = 1;
}

void half_sectionA(void) {
  MF.FLAG.CTRL = 0;
  drive_A((MF.FLAG.CALC_OFFSET) ? HALF_SEC_DIST - CALC_OFFSET_DIST
                                : HALF_SEC_DIST);
}

void half_sectionD(void) {
  MF.FLAG.CTRL = 0;
  drive_D((MF.FLAG.CALC_OFFSET) ? HALF_SEC_DIST - CALC_OFFSET_DIST
                                : HALF_SEC_DIST);
}

void one_sectionU(void) {
  MF.FLAG.CTRL = 1;
  drive_U((MF.FLAG.CALC_OFFSET) ? HALF_SEC_DIST * 2 - CALC_OFFSET_DIST
                                : HALF_SEC_DIST * 2);
}

void turn_right(bool is_slalom) {
  if (is_slalom) {
    drive_S_R90();
  } else {
    half_sectionD();
    drive_wait();
    drive_R_90R();
    drive_wait();
    half_sectionA();
  }
}

void turn_left(bool is_slalom) {
  if (is_slalom) {
    drive_S_L90();
  } else {
    half_sectionD();
    drive_wait();
    drive_R_90L();
    drive_wait();
    half_sectionA();
  }
}

void turn_back(bool is_last) {
  if (!is_last) {
    half_sectionD();
  }
  uint8_t current_wall = maze_wall[mouse.y][mouse.x];
  uint8_t front_wall_mask;
  uint8_t left_wall_mask;
  uint8_t right_wall_mask;

  switch (mouse.dir) {
  case 0: // 北向き
    front_wall_mask = 0x08;
    left_wall_mask = 0x01;
    right_wall_mask = 0x04;
    break;
  case 1: // 東向き
    front_wall_mask = 0x04;
    left_wall_mask = 0x08;
    right_wall_mask = 0x02;
    break;
  case 2: // 南向き
    front_wall_mask = 0x02;
    left_wall_mask = 0x04;
    right_wall_mask = 0x01;
    break;
  case 3: // 西向き
    front_wall_mask = 0x01;
    left_wall_mask = 0x02;
    right_wall_mask = 0x08;
    break;
  }
  if ((current_wall & front_wall_mask) && (current_wall & right_wall_mask)) {
    drive_R_90L();
    drive_wait();
    set_position(0);
    drive_wait();
    drive_R_90L();
    drive_wait();
    set_position(0);
  } else if ((current_wall & front_wall_mask) &&
             (current_wall & left_wall_mask)) {
    drive_R_90R();
    drive_wait();
    set_position(0);
    drive_wait();
    drive_R_90R();
    drive_wait();
    set_position(0);
  } else {
    drive_R_180();
  }
  if (!is_last) {
    half_sectionA();
  }
}

void set_position(bool sw) {
  MF.FLAG.CTRL = 0;
  drive_c(-SETPOS_BACK);
  drive_wait();
  if (sw) {
    get_base_sensor_values();
    drive_wait();
  }
  drive_c(SETPOS_FRONT);
  drive_wait();
}

void hitting_wall(void) {
  hitting_step(drive_R_90R);
  hitting_step(drive_R_90L);
}

void last_run(void) {
  MF.FLAG.CTRL = 0;
  MF.FLAG.CALC_OFFSET = 0;
  half_sectionD();
  drive_wait();

  turn_back(true);
}

void drive_R_90R(void) {
  MF.FLAG.CTRL = 0;
  drive_R(R90_ANGLE, DEFAULT_NEKO);
}

void drive_R_90L(void) {
  MF.FLAG.CTRL = 0;
  drive_R(-L90_ANGLE, -DEFAULT_NEKO);
}

void drive_R_180(void) {
  MF.FLAG.CTRL = 0;
  drive_R(R180_ANGLE, DEFAULT_NEKO);
}

void drive_S_R90(void) { drive_slalom(current_slalom_profile, true); }

void drive_S_L90(void) { drive_slalom(current_slalom_profile, false); }

void drive_c(float dist) {
  enable_motor();
  reset_current_position();

  output_speed.vect = (dist > 0) ? DEFAULT_VECT : -DEFAULT_VECT;
  output_speed.neko = 0;

  drive_start();
  while (fabsf(current_position.dist) < fabsf(dist))
    ;
  drive_stop();
}

void start_sequence(void) {
  drive_R_90R();
  drive_wait();
  set_position(0);
  drive_wait();
  drive_R_90L();
  drive_wait();
  set_position(0);
  drive_wait();

  get_base_sensor_values();
}

// 下位レイヤー
void drive_A(float dist) {
  drive_trapezoid(dist, current_speed.vect, target_speed.vect,
                  target_speed.vect);
}

void drive_D(float dist) {
  drive_trapezoid(dist, current_speed.vect, DEFAULT_VECT, target_speed.vect);
}

void drive_U(float dist) {
  drive_trapezoid(dist, current_speed.vect, target_speed.vect,
                  target_speed.vect);
}

void drive_wait(void) { HAL_Delay(50); }

void hitting_step(void (*rotate_func)(void)) {
  rotate_func();
  drive_wait();
  set_position(0);
  drive_wait();
}

void drive_straight(float dist, float target_v, float catnip) {
  enable_motor();
  reset_current_position();

  output_speed.vect = target_v;

  output_speed.target_catnip = catnip;
  drive_start();
  while (current_position.dist < dist) {
  }
  drive_stop();
}

void drive_trapezoid(float dist, float target_v, float end_v, float max_v) {
  enable_motor();
  reset_current_position();

  float sign = (dist > 0) ? 1.0f : -1.0f;

  output_speed.vect = target_v;
  output_speed.neko = 0;
  output_speed.target_catnip = 0;

  drive_start();

  float effective_dist = dist;
  if (MF.FLAG.CALC_OFFSET) {
    effective_dist = dist - ((dist > 0) ? CALC_OFFSET_DIST : -CALC_OFFSET_DIST);
    MF.FLAG.CALC_OFFSET = 0;
  }

  float abs_dist = fabsf(effective_dist);
  while (fabsf(current_position.dist) < abs_dist) {
    float mag_v = get_target_v(fabsf(current_position.dist), abs_dist, ACCEL,
                               fabsf(max_v), fabsf(target_v), fabsf(end_v));
    output_speed.vect = mag_v * sign;
  }
  drive_stop();
}

void drive_R(float angle, float target_neko) {
  enable_motor();
  reset_current_position();

  output_speed.vect = 0.0;
  output_speed.neko = target_neko;
  output_speed.target_catnip = 0;

  drive_start();
  float target_abs_angle = fabsf(angle);
  while (fabsf(current_position.angle) < target_abs_angle)
    ;
  drive_stop();
  HAL_Delay(50);
}

void drive_slalom(SlalomProfile p, bool direction) {
  enable_motor();
  reset_current_position();

  output_speed.vect = p.target_v;
  output_speed.neko = 0;
  output_speed.target_catnip = 0;

  drive_start();

  float cumulative_dist = 0;
  float sign = direction ? -1.0f : 1.0f;
  float catnip = p.target_catnip * sign;

  cumulative_dist +=
      (MF.FLAG.CALC) ? p.pre_offset_dist - CALC_OFFSET_DIST : p.pre_offset_dist;
  output_speed.target_catnip = 0;
  while (current_position.dist < cumulative_dist)
    ;

  cumulative_dist += p.acc_dist;
  output_speed.target_catnip = catnip;
  while (current_position.dist < cumulative_dist)
    ;

  cumulative_dist += p.const_dist;
  output_speed.target_catnip = 0;
  output_speed.neko = p.max_neko * sign;
  while (current_position.dist < cumulative_dist)
    ;

  cumulative_dist += p.decel_dist;
  output_speed.target_catnip = -catnip;
  while (current_position.dist < cumulative_dist)
    ;
  cumulative_dist += p.post_offset_dist;
  output_speed.target_catnip = 0;
  output_speed.neko = 0;
  while (current_position.dist < cumulative_dist)
    ;

  drive_stop();
}
