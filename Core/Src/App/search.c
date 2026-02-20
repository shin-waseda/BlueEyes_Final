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
    case 0x81:
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

void conf_route_dijkstra(void) {
  get_wall();
  write_map();

#ifdef DEBUG_PQ
  calc_cnt = 0;
  MF.FLAG.DEBUG_MODE = 1;
#endif

  if (!MF.FLAG.RETURN) {
    dijkstra_multi_goal(fw_goals, GOAL_NUM);
  } else {
    dijkstra_multi_goal(rt_goals, 1);
  }
  make_route_dijkstra(mouse.y, mouse.x, mouse.dir);
#ifdef DEBUG_PQ
  MF.FLAG.DEBUG_MODE = 0;
  printf("calc_cnt : %d\n", calc_cnt);
  dump_dijkstra_map(mouse.y, mouse.x, mouse.dir, fw_goals, GOAL_NUM);
  dump_route_dijkstra();
#endif
  r_cnt = 0;
}

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
  }
  if (!MF.FLAG.RETURN) {
    dijkstra_multi_goal(fw_goals, GOAL_NUM);
  } else {
    dijkstra_multi_goal(rt_goals, 1);
  }
  make_route_dijkstra(mouse.y, mouse.x, mouse.dir);
  if (!MF.FLAG.RETURN) {
    dump_dijkstra_map(mouse.y, mouse.x, mouse.dir, fw_goals, GOAL_NUM);
  } else {
    dump_dijkstra_map(mouse.y, mouse.x, mouse.dir, rt_goals, 1);
  }
  dump_route_dijkstra();

  half_sectionA();
  adv_pos();
  get_wall();
  write_map();

  r_cnt = (!MF.FLAG.SCND) ? 0 : 1;

  do {
    drive_calc_offset(CALC_OFFSET_DIST);
    // printf("route : %02X\n", route[r_cnt]);
    switch (route[r_cnt++]) {
    case 0x81:
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

    case 0xFF:
      drive_stop();
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

// ゴール判定ヘルパー
static bool is_goal(void) {
  for (int i = 0; i < GOAL_NUM; i++) {
    if (mouse.x == fw_goals[i].x && mouse.y == fw_goals[i].y) {
      return true;
    }
  }
  return false;
}

void run_route_continuous(void) {
  MF.FLAG.CTRL = 1;
  MF.FLAG.CALC = 0;
  MF.FLAG.CALC_OFFSET = 0;

  half_sectionA();
  adv_pos();
  LEDinfo tmp;

  r_cnt = 1;
  float start_v = current_speed.vect;

  while (1) {
    uint8_t cmd = route[r_cnt++];
    // printf("cmd:%02X next:%02X\n", cmd, next_cmd);
    if (cmd == 0xFF) {
      drive_stop();
      break;
    }

    if ((cmd & 0xF0) == 0x80) {
      tmp.LEDs = 0;
      tmp.LED.front = 1;
      led_write(tmp);
      int n = cmd & 0x0F;
      float dist = n * (HALF_SEC_DIST * 2);

      float end_v = current_slalom_profile.target_v;

      MF.FLAG.CTRL = 1;

      drive_trapezoid(dist, start_v, end_v, target_speed.vect);
      start_v = current_slalom_profile.target_v;

      for (int i = 0; i < n; i++) {
        adv_pos();
      }
    } else if (cmd == 0x44) {
      tmp.LEDs = 0;
      tmp.LED.right = 1;
      led_write(tmp);
      MF.FLAG.CTRL = 0;
      drive_S_R90();
      turn_dir(DIR_TURN_R90);
      adv_pos();
      start_v = current_slalom_profile.target_v;
    } else if (cmd == 0x11) {
      tmp.LEDs = 0;
      tmp.LED.left = 1;
      led_write(tmp);
      MF.FLAG.CTRL = 0;
      drive_S_L90();
      turn_dir(DIR_TURN_L90);
      adv_pos();
      start_v = current_slalom_profile.target_v;
    }
    // printf("loop end   v=%d\n", (int)current_speed.vect);
    if (is_goal()) {
      break;
    }
  }

  one_sectionU();
  turn_dir(DIR_TURN_180);
  led_pattern_goal();
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
