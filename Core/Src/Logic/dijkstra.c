#include "global.h"
#include "logic.h"
#include "params.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// 前の状態から来た時の動作
typedef enum {
  OP_NONE = 0,
  OP_FORWARD,
  OP_TURN_RIGHT,
  OP_TURN_LEFT,
  OP_TURN_180
} PrevOp;

typedef struct {
  uint16_t dist;        // 距離 (0-65535)
  uint8_t prev_dir : 3; // 0-7あれば十分 (3bit)
  uint8_t prev_op : 3;  // PrevOpの列挙型 (3bit)
  uint8_t visited : 1;  // 0 or 1 (1bit)
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
    return (w & 0x08) != 0; // 北
  case 1:
    return (w & 0x04) != 0; // 東
  case 2:
    return (w & 0x02) != 0; // 南
  case 3:
    return (w & 0x01) != 0; // 西
  }

  return true;
}

void dijkstra(uint8_t start_y, uint8_t start_x, uint8_t start_dir,
              uint8_t goal_y_, uint8_t goal_x_) {

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

  st[start_y][start_x][start_dir].dist = 0;
  pq_push(start_y, start_x, start_dir, 0);

  const int8_t dx[4] = {0, 1, 0, -1};
  const int8_t dy[4] = {1, 0, -1, 0};

  // コスト設定（スラロームモデル）
  const uint16_t forward_cost = 1;
  const uint16_t turn_90_cost = 7;
  const uint16_t turn_180_cost = 100; // 非常に高いコスト

  while (!pq_empty()) {
    PQNode u = pq_pop();

    if (st[u.y][u.x][u.dir].visited)
      continue;
    st[u.y][u.x][u.dir].visited = true;

    uint16_t current_dist = st[u.y][u.x][u.dir].dist;

    // スラロームモデル：すべての遷移で「回転しながら前進」
    // 1. 直進（向きそのまま）
    if (!has_wall(u.x, u.y, u.dir)) {
      int8_t nx = u.x + dx[u.dir];
      int8_t ny = u.y + dy[u.dir];

      if (nx >= 0 && nx <= 15 && ny >= 0 && ny <= 15) {
        uint16_t new_dist = current_dist + forward_cost;

        if (new_dist < st[ny][nx][u.dir].dist) {
          st[ny][nx][u.dir].dist = new_dist;
          st[ny][nx][u.dir].prev_dir = u.dir;
          st[ny][nx][u.dir].prev_op = OP_FORWARD;
          pq_push(ny, nx, u.dir, new_dist);
        }
      }
    }

    // 2. 右折（回転しながら前進）
    uint8_t right_dir = (u.dir + 1) % 4;
    if (!has_wall(u.x, u.y, right_dir)) {
      int8_t nx = u.x + dx[right_dir];
      int8_t ny = u.y + dy[right_dir];

      if (nx >= 0 && nx <= 15 && ny >= 0 && ny <= 15) {
        uint16_t new_dist = current_dist + turn_90_cost;

        if (new_dist < st[ny][nx][right_dir].dist) {
          st[ny][nx][right_dir].dist = new_dist;
          st[ny][nx][right_dir].prev_dir = u.dir;
          st[ny][nx][right_dir].prev_op = OP_TURN_RIGHT;
          pq_push(ny, nx, right_dir, new_dist);
        }
      }
    }

    // 3. 左折（回転しながら前進）
    uint8_t left_dir = (u.dir + 3) % 4;
    if (!has_wall(u.x, u.y, left_dir)) {
      int8_t nx = u.x + dx[left_dir];
      int8_t ny = u.y + dy[left_dir];

      if (nx >= 0 && nx <= 15 && ny >= 0 && ny <= 15) {
        uint16_t new_dist = current_dist + turn_90_cost;

        if (new_dist < st[ny][nx][left_dir].dist) {
          st[ny][nx][left_dir].dist = new_dist;
          st[ny][nx][left_dir].prev_dir = u.dir;
          st[ny][nx][left_dir].prev_op = OP_TURN_LEFT;
          pq_push(ny, nx, left_dir, new_dist);
        }
      }
    }

    // 4. 180度ターン（回転しながら前進、非常に高コスト）
    uint8_t back_dir = (u.dir + 2) % 4;
    if (!has_wall(u.x, u.y, back_dir)) {
      int8_t nx = u.x + dx[back_dir];
      int8_t ny = u.y + dy[back_dir];

      if (nx >= 0 && nx <= 15 && ny >= 0 && ny <= 15) {
        uint16_t new_dist = current_dist + turn_180_cost;

        if (new_dist < st[ny][nx][back_dir].dist) {
          st[ny][nx][back_dir].dist = new_dist;
          st[ny][nx][back_dir].prev_dir = u.dir;
          st[ny][nx][back_dir].prev_op = OP_TURN_180;
          pq_push(ny, nx, back_dir, new_dist);
        }
      }
    }
  }
}

void make_route_dijkstra(uint8_t goal_y_, uint8_t goal_x_) {
  for (int i = 0; i < 512; i++)
    route[i] = 0xFF;
  uint16_t min_goal_dist = MAX_COST;
  int8_t goal_dir = -1;

  for (int d = 0; d < 4; d++) {
    if (st[goal_y_][goal_x_][d].dist < min_goal_dist) {
      min_goal_dist = st[goal_y_][goal_x_][d].dist;
      goal_dir = d;
    }
  }

  if (goal_dir == -1) {
    return;
  }

  static uint8_t path_actions[512];
  int path_idx = 0;

  int8_t curr_x = goal_x_;
  int8_t curr_y = goal_y_;
  int8_t curr_dir = goal_dir;

  const int8_t dx[4] = {0, 1, 0, -1};
  const int8_t dy[4] = {1, 0, -1, 0};

  // スラロームモデル：すべての遷移で座標が変化する
  while (st[curr_y][curr_x][curr_dir].dist > 0) {
    // printf("BACK (%d,%d,%d) dist=%d op=%d prev_dir=%d\r\n", curr_x, curr_y,
    //        curr_dir, st[curr_x][curr_y][curr_dir].dist,
    //        st[curr_x][curr_y][curr_dir].prev_op,
    //        st[curr_x][curr_y][curr_dir].prev_dir);
    uint8_t op = st[curr_y][curr_x][curr_dir].prev_op;
    int8_t prev_dir = st[curr_y][curr_x][curr_dir].prev_dir;

    if (prev_dir == -1)
      break;

    // すべての動作で座標を戻す（スラロームモデル）
    curr_x -= dx[curr_dir];
    curr_y -= dy[curr_dir];

    // 動作コードを記録
    if (op == OP_FORWARD) {
      path_actions[path_idx++] = 0x88;
    } else if (op == OP_TURN_RIGHT) {
      path_actions[path_idx++] = 0x44;
    } else if (op == OP_TURN_LEFT) {
      path_actions[path_idx++] = 0x11;
    } else if (op == OP_TURN_180) {
      path_actions[path_idx++] = 0x22;
    }

    curr_dir = prev_dir;

    if (path_idx >= 511)
      break;
  }

  int r_i = 0;
  while (path_idx > 0) {
    route[r_i++] = path_actions[--path_idx];
  }
  route[r_i] = 0xFF;
}

void dump_wall_cost_map(void) {
  printf("\r\n=== WALL + COST MAP ===\r\n");

  for (int y = 15; y >= 0; y--) {
    /* --- 上壁行 --- */
    for (int x = 0; x < 16; x++) {
      printf("+");
      if (has_wall(x, y, 0)) // North
        printf("---");
      else
        printf("   ");
    }
    printf("+\r\n");

    /* --- 中身行（左壁 + cost + 右壁） --- */
    for (int x = 0; x < 16; x++) {
      /* West壁 */
      if (has_wall(x, y, 3))
        printf("|");
      else
        printf(" ");

      /* cost表示 */
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

    /* East壁（右端） */
    if (has_wall(15, y, 1))
      printf("|");
    else
      printf(" ");

    printf("\r\n");
  }

  /* --- 最下段 --- */
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

  /* ゴールから復元して印を付ける */
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

    /* 戻る */
    static const int dx[4] = {0, 1, 0, -1};
    static const int dy[4] = {1, 0, -1, 0};

    x -= dx[dir];
    y -= dy[dir];
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
