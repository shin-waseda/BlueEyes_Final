#include "app.h"
#include "global.h"
#include "interface.h"
#include "logic.h"
#include <stdio.h>

const SlalomProfile S90_400 = {.acc_dist = 25.6,
                               .decel_dist = 25.6,
                               .const_dist = 44.0,
                               .pre_offset_dist = 10.0 + 23,
                               .post_offset_dist = 20.0 + 3,
                               .target_catnip = 5000,
                               .max_neko = 350,
                               .target_v = 400};

const SlalomProfile S90_500 = {.acc_dist = 43.0,
                               .decel_dist = 43.0,
                               .const_dist = 32.0,
                               .pre_offset_dist = 20.0,
                               .post_offset_dist = 10.0,
                               .target_catnip = 7000,
                               .max_neko = 600,
                               .target_v = 500};

SlalomProfile select_S90_param(uint8_t mode) {
  switch (mode) {
  case 1:
    return S90_400;
  case 2:
    return S90_500;
  default:
    return S90_400;
  }
}

uint8_t execute_mode(uint8_t mode) {
  switch (mode) {
  case 0:
    LC.LED.right = 1;
    LC.LED.front = 1;
    LC.LED.left = 1;
    led_write(LC);
    HAL_Delay(100);
    LC.LED.right = 0;
    LC.LED.front = 0;
    LC.LED.left = 0;
    led_write(LC);
    break;

  case 1:
    select_speed(select_mode(1));
    enable_motor();
    MF.FLAG.SCND = 0;
    goal_x = GOAL_X;
    goal_y = GOAL_Y;

    start_sequence();

    led_pattern_search();
    searchB_adachi(0);
    HAL_Delay(500);

    goal_x = goal_y = 0;
    MF.FLAG.RETURN = 1;
    led_pattern_search();
    searchB_adachi(0);

    goal_x = GOAL_X;
    goal_y = GOAL_Y;
    MF.FLAG.RETURN = 0;

    disable_motor();

    break;

  case 2:
    select_speed(select_mode(1));
    current_slalom_profile = select_S90_param(select_mode(1));
    enable_motor();
    MF.FLAG.SCND = 0;
    goal_x = GOAL_X;
    goal_y = GOAL_Y;

    start_sequence();

    MF.FLAG.RETURN = 0;
    led_pattern_search();
    searchB_dijkstra(1);
    HAL_Delay(500);

    goal_x = goal_y = 0;
    MF.FLAG.RETURN = 1;
    led_pattern_search();
    searchB_dijkstra(1);

    goal_x = GOAL_X;
    goal_y = GOAL_Y;
    MF.FLAG.RETURN = 0;

    disable_motor();

    break;

  case 5:
    printf("Dijkstra Test.\n");

    load_map_from_flash();

    make_smap_adachi();
    make_route();
    dump_adachi_map();

    dijkstra_multi_goal(goals, GOAL_NUM);
    make_route_dijkstra(mouse.y, mouse.x, mouse.dir);
    dump_dijkstra_map(mouse.y, mouse.x, mouse.dir);
    dump_route_dijkstra();
    break;
  case 6:
    test_drive();
    break;

  case 7:
    printf("Sensor Check.\n");
    while (1) {
      uint8_t wall_info = get_wall_info(sensor_values);
      LEDinfo tmp_led;
      tmp_led.LED.right = (wall_info & 0x44) != 0;
      tmp_led.LED.front = (wall_info & 0x88) != 0;
      tmp_led.LED.left = (wall_info & 0x11) != 0;
      tmp_led.LED.side = 0;
      led_write(tmp_led);

      printf(" ad_l : %4d, ad_fl : %4d, ad_fr : %4d, ad_r : %4d\n",
             sensor_values.SensorValueL, sensor_values.SensorValueFL,
             sensor_values.SensorValueFR, sensor_values.SensorValueR);
      if (wall_info & 0x11) {
        printf("Left : [X], ");
      } else {
        printf("Left : [ ], ");
      }
      if (wall_info & 0x88) {
        printf("Front : [X], ");
      } else {
        printf("Front : [ ], ");
      }
      if (wall_info & 0x44) {
        printf("Right : [X]\n");
      } else {
        printf("Right : [ ]\n");
      }

      HAL_Delay(333);
    }
    break;
  default:
    break;
  }

  return (uint8_t)select_mode(0);
}

void test_straight(void) {
  uint16_t dist_idx = select_mode(1); // 1-7 区画
  select_speed(select_mode(1));
  float dist = 180.0f * (float)dist_idx;
  float target_v = target_speed.vect;

  printf("Straight Test: %.0fmm, %.0fmm/s\n", dist, target_v);
  get_base_sensor_values();
  MF.FLAG.CTRL = 1;
  // drive_trapezoid(dist, 200, 200, target_v);
  drive_U(dist);
}

void test_rotate(void) {
  uint16_t type = select_mode(1); // 1:R90, 2:L90, 3:R180

  printf("Rotate Test: Type %d\n", type);

  switch (type) {
  case 1:
    for (int i = 0; i < 16; i++) {
      drive_R_90R();
    }
    break;
  case 2:
    for (int i = 0; i < 16; i++) {
      drive_R_90L();
    }
    break;
  case 3:
    for (int i = 0; i < 8; i++) {
      drive_R_180();
    }
    break;
  }
}

void test_slalom(void) {
  MF.FLAG.CALC_OFFSET = 0;
  SlalomProfile p = select_S90_param(select_mode(1));
  uint16_t dir = select_mode(1);        // 1:Right, 2:Left
  uint16_t count_mode = select_mode(1); // 1:1time, other:16times

  bool is_right = (dir == 1);
  uint16_t count = (count_mode == 1) ? 1 : 16;

  printf("Slalom Test: Profile %.0f, %s, %d times\n", p.target_v,
         is_right ? "Right" : "Left", count);
  set_position(0);
  drive_trapezoid(90, current_speed.vect, 400, p.target_v);
  for (int i = 0; i < count; i++) {
    drive_slalom(p, is_right);
  }
  // drive_trapezoid(180, current_speed.vect, 200, p.target_v);
}

void test_drive(void) {
  uint8_t mode = 0;
  while (1) {
    mode = select_mode(mode); // 1:Straight, 2:Rotate, 3:Slalom
    switch (mode) {
    case 0:
      set_position(0);
      break;
    case 1:
      test_straight();
      break;
    case 2:
      test_rotate();
      break;
    case 3:
      test_slalom();
      break;
    case 4:
      select_speed(select_mode(1));
      drive_calc_offset(CALC_OFFSET_DIST);

      break;
    case 7:
      drive_straight(180, 0, 100);
      return;
    default:
      return;
    }
  }
}

void select_speed(uint8_t mode) {
  switch (mode) {
  case 1:
    target_speed.vect = 400;
    break;
  case 2:
    target_speed.vect = 500;
    break;
  case 3:
    target_speed.vect = 600;
    break;
  case 4:
    target_speed.vect = 800;
    break;
  case 5:
    target_speed.vect = 1000;
    break;
  case 6:
    target_speed.vect = 1200;
    break;
  case 7:
    target_speed.vect = 1400;
    break;
  }
}
