#include "global.h"
#include "logic.h"
#include "params.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

MazePosition goals[GOAL_NUM] = {{GOAL_X, GOAL_Y}};

static const uint8_t turn_cost[4][4] = {
    /* pdir\udir */
    {0, 7, 100, 7},
    {7, 0, 7, 100},
    {100, 7, 0, 7},
    {7, 100, 7, 0}};

typedef struct {
  uint16_t dist;        // 0 ~ 65535 (2byte)
  uint32_t nx : 4;      // 0 ~ 15 (4bit)
  uint32_t ny : 4;      // 0 ~ 15 (4bit)
  uint32_t nd : 2;      // 0 ~ 3  (2bit)
  uint32_t visited : 1; // 0 or 1 (1bit)
  uint32_t padding : 5; // 合計32bit(4byte)になるように調整
} State;

State st[16][16][4];
static const int8_t dx[4] = {0, 1, 0, -1};
static const int8_t dy[4] = {1, 0, -1, 0};

uint8_t determine_turn_op(uint8_t current_dir, uint8_t next_dir) {
  int diff = (next_dir - current_dir + 4) % 4;

  switch (diff) {
  case 1:
    return 0x44;
  case 3:
    return 0x11;
  case 2:
    return 0x22;
  default:
    return 0x00;
  }
}

void dijkstra_multi_goal(MazePosition goals[], uint8_t goal_count) {
  static const uint8_t wall_mask[4] = {0x08, 0x04, 0x02, 0x01};

  for (int y = 0; y < 16; y++) {
    for (int x = 0; x < 16; x++) {
      for (int d = 0; d < 4; d++) {
        st[y][x][d].dist = MAX_COST;
        st[y][x][d].visited = false;
        st[y][x][d].nx = 0xF;
        st[y][x][d].ny = 0xF;
        st[y][x][d].nd = 0;
      }
    }
  }

  pq_init();

  for (int i = 0; i < goal_count; i++) {
    uint8_t gx = goals[i].x;
    uint8_t gy = goals[i].y;

    for (int d = 0; d < 4; d++) {
      st[gy][gx][d].dist = 0;
      pq_push(gy, gx, d, 0);
    }
  }

  while (!pq_empty()) {
    PQNode u = pq_pop();

    if (st[u.y][u.x][u.dir].visited)
      continue;

    st[u.y][u.x][u.dir].visited = true;

    uint16_t cd = st[u.y][u.x][u.dir].dist;

    int8_t px = u.x - dx[u.dir];
    int8_t py = u.y - dy[u.dir];

    if (px < 0 || px >= 16 || py < 0 || py >= 16)
      continue;

    uint8_t w = maze_wall[py][px];
    if (MF.FLAG.SCND)
      w >>= 4;

    if (w & wall_mask[u.dir])
      continue;

    for (uint8_t pdir = 0; pdir < 4; pdir++) {
      uint16_t nd = cd + 1 + turn_cost[pdir][u.dir];

      if (nd < st[py][px][pdir].dist) {
        st[py][px][pdir].dist = nd;

        st[py][px][pdir].ny = u.y;
        st[py][px][pdir].nx = u.x;
        st[py][px][pdir].nd = u.dir;

        pq_push(py, px, pdir, nd);
      }
    }
  }
}

void make_route_dijkstra(uint8_t start_y, uint8_t start_x, uint8_t start_dir) {
  uint8_t x = start_x;
  uint8_t y = start_y;
  uint8_t dir = start_dir;
  int r_idx = 0;

  while (st[y][x][dir].dist > 0 && r_idx < 500) {
    uint8_t next_x = st[y][x][dir].nx;
    uint8_t next_y = st[y][x][dir].ny;
    uint8_t next_dir = st[y][x][dir].nd;

    if (next_x == 0xF || (next_x == x && next_y == y && next_dir == dir)) {
      break;
    }

    if (dir == next_dir) {
      route[r_idx++] = 0x88;
    } else {
      route[r_idx++] = determine_turn_op(dir, next_dir);
    }

    x = next_x;
    y = next_y;
    dir = next_dir;
  }
  route[r_idx] = 0xFF;
}

void dump_dijkstra_map(uint8_t my, uint8_t mx, uint8_t md) {
  static const uint8_t wall_mask[4] = {0x08, 0x04, 0x02, 0x01};

  printf("\r\n=== WALL + COST MAP (S=Start/Pos, G=Goal) ===\r\n");

  for (int y = 15; y >= 0; y--) {
    /* --- 上側（北壁） --- */
    for (int x = 0; x < 16; x++) {
      printf("+");

      uint8_t w = maze_wall[y][x];
      if (MF.FLAG.SCND)
        w >>= 4;

      printf((w & wall_mask[0]) ? "---" : "   ");
    }
    printf("+\r\n");

    /* --- セル内部と西壁 --- */
    for (int x = 0; x < 16; x++) {
      uint8_t w = maze_wall[y][x];
      if (MF.FLAG.SCND)
        w >>= 4;

      /* 西壁 dir=3 */
      printf((w & wall_mask[3]) ? "|" : " ");

      /* --- 中身表示 --- */

      /* ゴール判定 */
      bool is_goal = false;
      for (int i = 0; i < GOAL_NUM; i++) {
        if (x == goals[i].x && y == goals[i].y) {
          is_goal = true;
          break;
        }
      }

      if (is_goal) {
        printf(" G ");
      }
      /* 現在地（矢印） */
      else if (x == mx && y == my) {
        const char *arrows[4] = {" ^ ", " > ", " v ", " < "};
        printf("%s", arrows[md]);
      }
      /* コスト表示 */
      else {
        uint16_t best = MAX_COST;
        for (int d = 0; d < 4; d++) {
          if (st[y][x][d].dist < best)
            best = st[y][x][d].dist;
        }

        if (best == MAX_COST)
          printf(" --");
        else
          printf("%3d", best);
      }
    }

    /* --- 右端（東壁） --- */
    {
      uint8_t w = maze_wall[y][15];
      if (MF.FLAG.SCND)
        w >>= 4;

      printf((w & wall_mask[1]) ? "|\r\n" : " \r\n");
    }
  }

  /* --- 最下段 --- */
  for (int x = 0; x < 16; x++)
    printf("+---");
  printf("+\r\n");
}

void dump_route_dijkstra(void) {
  printf("\r\n=== ROUTE ACTIONS ===\r\n");

  for (int i = 0; route[i] != 0xFF; i++) {
    uint8_t a = route[i];

    if (a == 0x88)
      printf("%3d: FWD\r\n", i);
    else if (a == 0x44)
      printf("%3d: RIGHT\r\n", i);
    else if (a == 0x11)
      printf("%3d: LEFT\r\n", i);
    else if (a == 0x22)
      printf("%3d: BACK\r\n", i);
    else
      printf("%3d: UNKNOWN %02X\r\n", i, a);
  }

  printf("END\r\n");
}
