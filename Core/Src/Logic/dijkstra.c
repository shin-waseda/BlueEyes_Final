#include "global.h"
#include "logic.h"
#include "params.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

MazePosition goals[GOAL_NUM] = {{GOAL_X, GOAL_Y}};

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
// 1. 無効値の定義
#define DIR_NONE 7

void dijkstra_multi_goal(MazePosition goals[], uint8_t goal_count) {
  for (int y = 0; y < 16; y++) {
    for (int x = 0; x < 16; x++) {
      for (int d = 0; d < 4; d++) {
        st[y][x][d].dist = MAX_COST;
        st[y][x][d].visited = false;
        st[y][x][d].prev_dir = DIR_NONE; // -1 の代わりに 7
        st[y][x][d].prev_op = OP_NONE;
      }
    }
  }

  pq_init();
  static const int8_t dx[4] = {0, 1, 0, -1};
  static const int8_t dy[4] = {1, 0, -1, 0};

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

    for (uint8_t move_dir = 0; move_dir < 4; move_dir++) {
      int px = u.x - dx[move_dir];
      int py = u.y - dy[move_dir];

      if (px < 0 || px >= 16 || py < 0 || py >= 16)
        continue;
      if (has_wall(px, py, move_dir))
        continue;

      // 遷移コストの設定
      uint16_t nd;
      PrevOp op;

      if (move_dir == u.dir) {
        nd = cd + 1;
        op = OP_FORWARD;
      } else if (move_dir == (u.dir + 3) % 4) {
        nd = cd + 7;
        op = OP_TURN_RIGHT;
      } else if (move_dir == (u.dir + 1) % 4) {
        nd = cd + 7;
        op = OP_TURN_LEFT;
      } else {
        nd = cd + 100;
        op = OP_TURN_180;
      }

      if (nd < st[py][px][move_dir].dist) {
        st[py][px][move_dir].dist = nd;
        st[py][px][move_dir].prev_dir = u.dir;
        st[py][px][move_dir].prev_op = op;
        pq_push(py, px, move_dir, nd);
      }
    }
  }
}

// 引数を「マウスの現在地」に変更！
void make_route_dijkstra(uint8_t start_y, uint8_t start_x, uint8_t start_dir) {
  for (int i = 0; i < 512; i++)
    route[i] = 0xFF;

  int r_idx = 0;
  uint8_t x = start_x;
  uint8_t y = start_y;
  uint8_t dir = start_dir;

  static const int8_t dx[4] = {0, 1, 0, -1};
  static const int8_t dy[4] = {1, 0, -1, 0};

  // コストが 0（ゴール）になるまで、記録された prev ポインタを順に辿る
  // ゴールから探索を広げたので、prev は自動的にゴールへの道順になっている
  while (st[y][x][dir].dist > 0) {
    uint8_t op = st[y][x][dir].prev_op;
    uint8_t next_dir = st[y][x][dir].prev_dir;

    if (next_dir == DIR_NONE)
      break;

    switch (op) {
    case OP_FORWARD:
      route[r_idx++] = 0x88;
      break;
    case OP_TURN_RIGHT:
      route[r_idx++] = 0x44;
      break;
    case OP_TURN_LEFT:
      route[r_idx++] = 0x11;
      break;
    case OP_TURN_180:
      route[r_idx++] = 0x22;
      break;
    default:
      break;
    }

    // 次の座標へ（探索時とは逆に、dxを足して進む）
    x += dx[dir];
    y += dy[dir];
    dir = next_dir;

    if (r_idx >= 511)
      break;
  }
  route[r_idx] = 0xFF;
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