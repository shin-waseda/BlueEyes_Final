#include "app.h"
#include "global.h"
#include "interface.h"
#include "logic.h"
#include "params.h"

uint8_t wall_temp;

void search_init(void);
void get_wall(void);
void write_map(void);
void conf_route(void);

void search_init(void) {
  MF.FLAGS = 0;
  goal_x = GOAL_X;
  goal_y = GOAL_Y;
  map_init();
  mouse.x = 0;
  mouse.y = 0;
  mouse.dir = 0;
}

void searchB_adachi(bool is_slalom) {
  MF.FLAG.CTRL = 1;

  if (MF.FLAG.SCND) {
    load_map_from_flash();
  }

  if (!MF.FLAG.RETURN) {
    get_wall();
    wall_temp &= ~0x88;
    write_map();
  }

  half_sectionA();
  adv_pos();
  get_wall();
  write_map();

  r_cnt = 0;
  make_smap_adachi();
  make_route();

  do {
    switch (route[r_cnt++]) {
    case 0x88:
      one_sectionU();
      break;
    case 0x44:
      turn_right(is_slalom);
      turn_dir(DIR_TURN_R90);
      break;
    case 0x22:
      turn_back(false);
      turn_dir(DIR_TURN_180);
      break;
    case 0x11:
      turn_left(is_slalom);
      turn_dir(DIR_TURN_L90);
      break;
    }
    adv_pos();
    conf_route();
  } while ((mouse.x != goal_x) || (mouse.y != goal_y));

  last_run();
  turn_dir(DIR_TURN_180);

  led_pattern_goal();

  if (!MF.FLAG.SCND)
    store_map_in_flash();
}

void get_wall(void) {
  wall_temp = get_wall_info(sensor_values);
  LEDinfo led_state;
  led_state.LEDs = 0; // 全消灯

  if (wall_temp & 0x88)
    led_state.LED.front = 1;
  if (wall_temp & 0x44)
    led_state.LED.right = 1;
  if (wall_temp & 0x11)
    led_state.LED.left = 1;

  led_write(led_state);
}
void write_map(void) { update_map_info(mouse, wall_temp); }
void conf_route(void) {
  get_wall();
  write_map();

  if (wall_temp & route[r_cnt]) {
    make_smap_adachi();
    make_route();
    r_cnt = 0;
  }
}
