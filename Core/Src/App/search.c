#include "app.h"
#include "global.h"
#include "interface.h"
#include "logic.h"
#include "params.h"

uint8_t wall_temp;

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

  // led_write(led_state);
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

void conf_route_dijkstra(void) {
  get_wall();
  write_map();

  dijkstra_multi_goal(goals, GOAL_NUM);
  make_route_dijkstra(mouse.y, mouse.x, mouse.dir);
  // dump_dijkstra_map(mouse.y, mouse.x, mouse.dir);
  // dump_route_dijkstra();
  r_cnt = 0;
}
// void conf_route_dijkstra(void) {
//   get_wall();
//   write_map();

//   if (wall_temp & route[r_cnt]) {
//     dijkstra_multi_goal(goals, GOAL_NUM);
//     make_route_dijkstra(mouse.y, mouse.x, mouse.dir);
//     r_cnt = 0;
//   }
// }

void searchB_dijkstra(bool is_slalom) {
  MF.FLAG.CTRL = 1;
  MF.FLAG.CALC = 1;

  if (MF.FLAG.SCND) {
    load_map_from_flash();
  }

  if (!MF.FLAG.RETURN) {
    get_wall();
    wall_temp &= ~0x88;
    write_map();
    dijkstra_multi_goal(goals, GOAL_NUM);
    make_route_dijkstra(mouse.y, mouse.x, mouse.dir);
    dump_dijkstra_map(mouse.y, mouse.x, mouse.dir);
    dump_route_dijkstra();
  }

  half_sectionA();
  adv_pos();
  get_wall();
  write_map();

  r_cnt = 0;

  do {
    LEDinfo led_state;
    led_state.LEDs = 0;
    led_write(led_state);
    // conf_route_dijkstra();
    drive_calc_offset(CALC_OFFSET_DIST);
    led_state.LED.left = led_state.LED.right = led_state.LED.front = 1;
    led_write(led_state);

    // printf("%02X ", route[r_cnt]);
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
    // conf_route();
  } while ((mouse.x != goal_x) || (mouse.y != goal_y));

  last_run();
  turn_dir(DIR_TURN_180);

  led_pattern_goal();

  if (!MF.FLAG.SCND)
    store_map_in_flash();
}

void dump_adachi_map(void) {
  printf("\r\n--- Adachi Maze Map (Cost & Wall) ---\r\n");

  for (int y = 15; y >= 0; y--) {
    // 1. 北側の壁を描画
    for (int x = 0; x < 16; x++) {
      // 北(bit3: 0x08)に壁があるかチェック
      printf("+%s", (maze_wall[y][x] & 0x08) ? "---" : "   ");
    }
    printf("+\r\n");

    // 2. 西壁とコスト(歩数)を描画
    for (int x = 0; x < 16; x++) {
      // 西(bit0: 0x01)に壁があるかチェック
      printf("%c", (maze_wall[y][x] & 0x01) ? '|' : ' ');

      // 足立法のコスト(smap)を表示。Dijkstraなら st[x][y][dir].dist に変更可
      uint16_t cost = smap[y][x];
      if (cost == 0xFFFF) {
        printf("###"); // 未到達・到達不能
      } else {
        printf("%3d", cost % 1000); // 3桁で表示
      }
    }
    // 最東端(東bit2: 0x04)の壁
    printf("%c\r\n", (maze_wall[y][15] & 0x04) ? '|' : ' ');
  }

  // 3. 最南端の底辺を描画
  for (int x = 0; x < 16; x++) {
    printf("+---");
  }
  printf("+\r\n");
}
