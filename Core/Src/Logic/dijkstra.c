#include "global.h"
#include "logic.h"
#include "params.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

MazePosition goals[GOAL_NUM] = {{GOAL_Y, GOAL_X}};

typedef enum {
  OP_NONE = 0,
  OP_FORWARD,
  OP_TURN_RIGHT,
  OP_TURN_LEFT,
  OP_TURN_180
} PrevOp;

typedef struct {
  uint16_t dist;
  uint8_t prev_dir : 3;
  uint8_t prev_op : 3;
  uint8_t visited : 1;
} __attribute__((packed)) State;

static State st[16][16][4];

static bool has_wall(int8_t x, int8_t y, uint8_t dir) {
  if (x < 0 || x > 15 || y < 0 || y > 15)
    return true;

  uint8_t w = maze_wall[y][x];

  if (MF.FLAG.SCND)
    w >>= 4;

  switch (dir) {
  case 0:
    return (w & 0x08) != 0;
  case 1:
    return (w & 0x04) != 0;
  case 2:
    return (w & 0x02) != 0;
  case 3:
    return (w & 0x01) != 0;
  }

  return true;
}

void dijkstra_multi_goal(MazePosition goals[], uint8_t goal_count) {
  // 初期化
  for (int y = 0; y < 16; y++) {
    for (int x = 0; x < 16; x++) {
      for (int d = 0; d < 4; d++) {
        st[y][x][d].dist = MAX_COST;
        st[y][x][d].visited = false;
        st[y][x][d].prev_dir = -1;
        st[y][x][d].prev_op = OP_NONE;
      }
    }
  }

  pq_init();

  static const int8_t dx[4] = {0, 1, 0, -1};
  static const int8_t dy[4] = {1, 0, -1, 0};

  const uint16_t forward_cost = 1;
  const uint16_t turn90_cost = 7;
  const uint16_t turn180_cost = 100;

  // Multi-Source（全ゴール dist=0）
  for (int i = 0; i < goal_count; i++) {
    uint8_t gx = goals[i].x;
    uint8_t gy = goals[i].y;

    for (int d = 0; d < 4; d++) {
      st[gy][gx][d].dist = 0;
      pq_push(gy, gx, d, 0);
    }
  }

  // ダイクストラ本体：スタートに向かって逆方向に探索
  while (!pq_empty()) {
    PQNode u = pq_pop();

    if (st[u.y][u.x][u.dir].visited)
      continue;

    st[u.y][u.x][u.dir].visited = true;

    uint16_t cd = st[u.y][u.x][u.dir].dist;

    /* ============================
       逆遷移：FORWARD
       prev_dir = curr_dir （スタート時も同じ向き）
    ============================ */
    {
      uint8_t prev_dir = u.dir;

      int px = u.x - dx[u.dir];
      int py = u.y - dy[u.dir];

      if (px >= 0 && px < 16 && py >= 0 && py < 16) {
        // (px,py)から u.dir方向に進んで(u.x,u.y)に到達できるか
        if (!has_wall(px, py, u.dir)) {
          uint8_t back = (u.dir + 2) % 4;
          if (!has_wall(u.x, u.y, back)) {
            uint16_t nd = cd + forward_cost;

            if (nd < st[py][px][prev_dir].dist) {
              st[py][px][prev_dir].dist = nd;
              st[py][px][prev_dir].prev_dir = u.dir;
              st[py][px][prev_dir].prev_op = OP_FORWARD;
              pq_push(py, px, prev_dir, nd);
            }
          }
        }
      }
    }

    /* ============================
       逆遷移：TURN_RIGHT
       (u.dir + 1) % 4 = 右折後の向き
       prev_dir = (u.dir + 3) % 4 = 元の向き
    ============================ */
    {
      uint8_t prev_dir = (u.dir + 3) % 4;

      int px = u.x - dx[u.dir];
      int py = u.y - dy[u.dir];

      if (px >= 0 && px < 16 && py >= 0 && py < 16) {
        // (px,py)から u.dir方向（=右折後）に進んで(u.x,u.y)に到達できるか
        if (!has_wall(px, py, u.dir)) {
          uint8_t back = (u.dir + 2) % 4;
          if (!has_wall(u.x, u.y, back)) {
            uint16_t nd = cd + turn90_cost;

            if (nd < st[py][px][prev_dir].dist) {
              st[py][px][prev_dir].dist = nd;
              st[py][px][prev_dir].prev_dir = u.dir;
              st[py][px][prev_dir].prev_op = OP_TURN_RIGHT;
              pq_push(py, px, prev_dir, nd);
            }
          }
        }
      }
    }

    /* ============================
       逆遷移：TURN_LEFT
       (u.dir + 3) % 4 = 左折後の向き
       prev_dir = (u.dir + 1) % 4 = 元の向き
    ============================ */
    {
      uint8_t prev_dir = (u.dir + 1) % 4;

      int px = u.x - dx[u.dir];
      int py = u.y - dy[u.dir];

      if (px >= 0 && px < 16 && py >= 0 && py < 16) {
        if (!has_wall(px, py, u.dir)) {
          uint8_t back = (u.dir + 2) % 4;
          if (!has_wall(u.x, u.y, back)) {
            uint16_t nd = cd + turn90_cost;

            if (nd < st[py][px][prev_dir].dist) {
              st[py][px][prev_dir].dist = nd;
              st[py][px][prev_dir].prev_dir = u.dir;
              st[py][px][prev_dir].prev_op = OP_TURN_LEFT;
              pq_push(py, px, prev_dir, nd);
            }
          }
        }
      }
    }

    /* ============================
       逆遷移：TURN_180
       (u.dir + 2) % 4 = 180度後の向き
       prev_dir = (u.dir + 2) % 4 = 元の向き
    ============================ */
    {
      uint8_t prev_dir = (u.dir + 2) % 4;

      int px = u.x - dx[u.dir];
      int py = u.y - dy[u.dir];

      if (px >= 0 && px < 16 && py >= 0 && py < 16) {
        if (!has_wall(px, py, u.dir)) {
          uint8_t back = (u.dir + 2) % 4;
          if (!has_wall(u.x, u.y, back)) {
            uint16_t nd = cd + turn180_cost;

            if (nd < st[py][px][prev_dir].dist) {
              st[py][px][prev_dir].dist = nd;
              st[py][px][prev_dir].prev_dir = u.dir;
              st[py][px][prev_dir].prev_op = OP_TURN_180;
              pq_push(py, px, prev_dir, nd);
            }
          }
        }
      }
    }
  }
}

void make_route_dijkstra(uint8_t goal_y, uint8_t goal_x) {
  for (int i = 0; i < 512; i++)
    route[i] = 0xFF;

  uint16_t best = MAX_COST;
  int8_t dir_goal = -1;

  for (int d = 0; d < 4; d++) {
    if (st[goal_y][goal_x][d].dist < best) {
      best = st[goal_y][goal_x][d].dist;
      dir_goal = d;
    }
  }

  if (dir_goal == -1)
    return;

  static const int8_t dx[4] = {0, 1, 0, -1};
  static const int8_t dy[4] = {1, 0, -1, 0};

  uint8_t path[512];
  int idx = 0;

  int8_t x = goal_x;
  int8_t y = goal_y;
  int8_t dir = dir_goal;

  while (st[y][x][dir].prev_dir != -1) {
    uint8_t op = st[y][x][dir].prev_op;
    int8_t pdir = st[y][x][dir].prev_dir;

    switch (op) {
    case OP_FORWARD:
      path[idx++] = 0x88;
      break;
    case OP_TURN_RIGHT:
      path[idx++] = 0x44;
      break;
    case OP_TURN_LEFT:
      path[idx++] = 0x11;
      break;
    case OP_TURN_180:
      path[idx++] = 0x22;
      break;
    default:
      break;
    }

    dir = pdir;
    x -= dx[dir];
    y -= dy[dir];

    if (idx >= 511)
      break;
  }

  int r = 0;
  while (idx > 0) {
    route[r++] = path[--idx];
  }
  route[r] = 0xFF;
}

void dump_wall_cost_map(void) {
  printf("\r\n=== WALL + COST MAP ===\r\n");

  for (int y = 15; y >= 0; y--) {
    for (int x = 0; x < 16; x++) {
      printf("+");
      if (has_wall(x, y, 0))
        printf("---");
      else
        printf("   ");
    }
    printf("+\r\n");

    for (int x = 0; x < 16; x++) {
      if (has_wall(x, y, 3))
        printf("|");
      else
        printf(" ");

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

    if (has_wall(15, y, 1))
      printf("|");
    else
      printf(" ");

    printf("\r\n");
  }

  for (int x = 0; x < 16; x++)
    printf("+---");
  printf("+\r\n");
}

void dump_route(void) {
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

void dump_path_on_map(uint8_t sy, uint8_t sx, uint8_t gy, uint8_t gx) {
  char mark[16][16] = {0};

  int8_t goal_dir = 0;
  uint16_t best = MAX_COST;

  for (int d = 0; d < 4; d++) {
    if (st[gy][gx][d].dist < best) {
      best = st[gy][gx][d].dist;
      goal_dir = d;
    }
  }

  int x = gx, y = gy, dir = goal_dir;

  while (st[y][x][dir].dist > 0) {
    mark[y][x] = '*';

    int pd = st[y][x][dir].prev_dir;
    if (pd < 0)
      break;

    static const int dx[4] = {0, 1, 0, -1};
    static const int dy[4] = {1, 0, -1, 0};

    x -= dx[pd];
    y -= dy[pd];
    dir = pd;
  }

  mark[sy][sx] = 'S';
  mark[gy][gx] = 'G';

  printf("\r\n=== PATH MAP ===\r\n");
  for (int yy = 15; yy >= 0; yy--) {
    for (int xx = 0; xx < 16; xx++) {
      char c = mark[yy][xx];
      if (c == 0)
        c = '.';
      printf("%c ", c);
    }
    printf("\r\n");
  }
}