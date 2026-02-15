#include "global.h"
#include "logic.h"
#include "params.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define MAX_COST 0xFFFF

// 前の状態から来た時の動作
typedef enum {
  OP_NONE = 0,
  OP_FORWARD,
  OP_TURN_RIGHT,
  OP_TURN_LEFT,
  OP_TURN_180
} PrevOp;

typedef struct {
    uint16_t dist;      // 距離 (0-65535)
    uint8_t prev_dir:3; // 0-7あれば十分 (3bit)
    uint8_t prev_op:3;  // PrevOpの列挙型 (3bit)
    uint8_t visited:1;  // 0 or 1 (1bit)
} __attribute__((packed)) State;

static State st[16][16][4];

static bool has_wall(int8_t x, int8_t y, uint8_t dir) {
  if (x < 0 || x > 15 || y < 0 || y > 15)
    return true;

  uint8_t w = maze_wall[y][x];

  switch (dir) {
  case 0: // 北
    return (w & 0x88) != 0;
  case 1: // 東
    return (w & 0x44) != 0;
  case 2: // 南
    return (w & 0x22) != 0;
  case 3: // 西
    return (w & 0x11) != 0;
  }

  return true;
}

void dijkstra(uint8_t start_x, uint8_t start_y, uint8_t start_dir,
              uint8_t goal_x_, uint8_t goal_y_) {

  // 初期化
  for (int x = 0; x < 16; x++) {
    for (int y = 0; y < 16; y++) {
      for (int d = 0; d < 4; d++) {
        st[x][y][d].dist = MAX_COST;
        st[x][y][d].visited = false;
        st[x][y][d].prev_dir = -1;
        st[x][y][d].prev_op = OP_NONE;
      }
    }
  }

  pq_init();

  st[start_x][start_y][start_dir].dist = 0;
  pq_push(start_x, start_y, start_dir, 0);

  const int8_t dx[4] = {0, 1, 0, -1};
  const int8_t dy[4] = {1, 0, -1, 0};

  // コスト設定（スラロームモデル）
  const uint16_t forward_cost = 1;
  const uint16_t turn_90_cost = 7;
  const uint16_t turn_180_cost = 100; // 非常に高いコスト

  while (!pq_empty()) {
    PQNode u = pq_pop();

    if (st[u.x][u.y][u.dir].visited)
      continue;
    st[u.x][u.y][u.dir].visited = true;

    uint16_t current_dist = st[u.x][u.y][u.dir].dist;

    // スラロームモデル：すべての遷移で「回転しながら前進」
    // 1. 直進（向きそのまま）
    if (!has_wall(u.x, u.y, u.dir)) {
      int8_t nx = u.x + dx[u.dir];
      int8_t ny = u.y + dy[u.dir];

      if (nx >= 0 && nx <= 15 && ny >= 0 && ny <= 15) {
        uint16_t new_dist = current_dist + forward_cost;

        if (new_dist < st[nx][ny][u.dir].dist) {
          st[nx][ny][u.dir].dist = new_dist;
          st[nx][ny][u.dir].prev_dir = u.dir;
          st[nx][ny][u.dir].prev_op = OP_FORWARD;
          pq_push(nx, ny, u.dir, new_dist);
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

        if (new_dist < st[nx][ny][right_dir].dist) {
          st[nx][ny][right_dir].dist = new_dist;
          st[nx][ny][right_dir].prev_dir = u.dir;
          st[nx][ny][right_dir].prev_op = OP_TURN_RIGHT;
          pq_push(nx, ny, right_dir, new_dist);
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

        if (new_dist < st[nx][ny][left_dir].dist) {
          st[nx][ny][left_dir].dist = new_dist;
          st[nx][ny][left_dir].prev_dir = u.dir;
          st[nx][ny][left_dir].prev_op = OP_TURN_LEFT;
          pq_push(nx, ny, left_dir, new_dist);
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

        if (new_dist < st[nx][ny][back_dir].dist) {
          st[nx][ny][back_dir].dist = new_dist;
          st[nx][ny][back_dir].prev_dir = u.dir;
          st[nx][ny][back_dir].prev_op = OP_TURN_180;
          pq_push(nx, ny, back_dir, new_dist);
        }
      }
    }
  }
}

void make_route_dijkstra(uint8_t goal_x_, uint8_t goal_y_) {
  uint16_t min_goal_dist = MAX_COST;
  int8_t goal_dir = -1;

  for (int d = 0; d < 4; d++) {
    if (st[goal_x_][goal_y_][d].dist < min_goal_dist) {
      min_goal_dist = st[goal_x_][goal_y_][d].dist;
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
  while (st[curr_x][curr_y][curr_dir].dist > 0) {
    uint8_t op = st[curr_x][curr_y][curr_dir].prev_op;
    int8_t prev_dir = st[curr_x][curr_y][curr_dir].prev_dir;

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
