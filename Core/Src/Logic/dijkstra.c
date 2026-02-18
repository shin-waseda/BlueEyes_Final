// #include "global.h"
// #include "logic.h"
// #include "params.h"
// #include <stdbool.h>
// #include <stdint.h>
// #include <stdio.h>

// MazePosition goals[GOAL_NUM] = {GOAL_X, GOAL_Y};
// // MazePosition goals[GOAL_NUM] = {{GOAL_X, GOAL_Y},
// //                                 {GOAL_X + 1, GOAL_Y},
// //                                 {GOAL_X, GOAL_Y + 1},
// //                                 {GOAL_X + 1, GOAL_Y + 1}};

// static const uint8_t turn_cost[4][4] = {
//     /* pdir\udir */
//     {0, 7, 100, 7},
//     {7, 0, 7, 100},
//     {100, 7, 0, 7},
//     {7, 100, 7, 0}};

// State st[16][16][4];
// static const int8_t dx[4] = {0, 1, 0, -1};
// static const int8_t dy[4] = {1, 0, -1, 0};

// uint8_t determine_turn_op(uint8_t current_dir, uint8_t next_dir) {
//   int diff = (next_dir - current_dir + 4) % 4;

//   switch (diff) {
//   case 1:
//     return 0x44;
//   case 3:
//     return 0x11;
//   case 2:
//     return 0x22;
//   default:
//     return 0x00;
//   }
// }

// void dijkstra_multi_goal(MazePosition goals[], uint8_t goal_count) {
//   static const uint8_t wall_mask[4] = {0x08, 0x04, 0x02, 0x01};

//   for (int y = 0; y < 16; y++) {
//     for (int x = 0; x < 16; x++) {
//       for (int d = 0; d < 4; d++) {
//         st[y][x][d].dist = MAX_COST;
//         st[y][x][d].visited = false;
//         st[y][x][d].nx = 0xF;
//         st[y][x][d].ny = 0xF;
//         st[y][x][d].nd = 0;
//       }
//     }
//   }

//   pq_init();

//   for (int i = 0; i < goal_count; i++) {
//     uint8_t gx = goals[i].x;
//     uint8_t gy = goals[i].y;

//     for (int d = 0; d < 4; d++) {
//       st[gy][gx][d].dist = 0;
//       pq_push(gy, gx, d, 0);
//     }
//   }

//   for (int i = 0; i < goal_count; i++) {
//     printf("GOAL[%d]: x=%d y=%d, st[%d][%d] dist=%d\r\n", i, goals[i].x,
//            goals[i].y, goals[i].y, goals[i].x,
//            st[goals[i].y][goals[i].x][0].dist);
//   }

//   while (!pq_empty()) {

//     PQNode u = pq_pop();

//     if (st[u.y][u.x][u.dir].visited)
//       continue;
//     st[u.y][u.x][u.dir].visited = true;

//     uint16_t cd = st[u.y][u.x][u.dir].dist;

//     /* predecessor は後ろ側 */
//     int8_t px = u.x - dx[u.dir];
//     int8_t py = u.y - dy[u.dir];

//     if (u.y == 8 && u.x == 7 && u.dir == 1) {
//       uint8_t w = maze_wall[py][px];
//       if (MF.FLAG.SCND)
//         w >>= 4;
//       printf("DEBUG: u=(8,7,E) cd=%d, px=%d py=%d, wall=0x%02X, mask=0x%02X,
//       "
//              "blocked=%d\r\n",
//              cd, px, py, w, wall_mask[u.dir], (w & wall_mask[u.dir]) != 0);
//     }

//     if (px < 0 || px >= 16 || py < 0 || py >= 16)
//       continue;

//     /* 壁判定：px→u に進めるか */
//     uint8_t w = maze_wall[py][px];
//     if (MF.FLAG.SCND)
//       w >>= 4;

//     if (w & wall_mask[u.dir])
//       continue;

//     /* predecessor の向き pdir を全探索 */
//     for (uint8_t pdir = 0; pdir < 4; pdir++) {

//       uint16_t nd = cd + 1 + turn_cost[pdir][u.dir];

//       if (nd < st[py][px][pdir].dist) {

//         st[py][px][pdir].dist = nd;

//         st[py][px][pdir].nx = u.x;
//         st[py][px][pdir].ny = u.y;
//         st[py][px][pdir].nd = u.dir;

//         pq_push(py, px, pdir, nd);
//       }
//     }
//   }
// }

// void make_route_dijkstra(uint8_t start_y, uint8_t start_x, uint8_t start_dir)
// {
//   uint8_t x = start_x;
//   uint8_t y = start_y;
//   uint8_t dir = start_dir;
//   int r_idx = 0;

//   while (st[y][x][dir].dist > 0 && r_idx < 500) {
//     uint8_t next_x = st[y][x][dir].nx;
//     uint8_t next_y = st[y][x][dir].ny;
//     uint8_t next_dir = st[y][x][dir].nd;

//     if (next_x == 0xF || (next_x == x && next_y == y && next_dir == dir)) {
//       break;
//     }

//     if (dir == next_dir) {
//       route[r_idx++] = 0x88;
//     } else {
//       route[r_idx++] = determine_turn_op(dir, next_dir);
//     }

//     x = next_x;
//     y = next_y;
//     dir = next_dir;
//   }
//   route[r_idx] = 0xFF;
// }

// void dump_dijkstra_map(uint8_t my, uint8_t mx, uint8_t md) {
//   static const uint8_t wall_mask[4] = {0x08, 0x04, 0x02, 0x01}; // N, E, S, W

//   printf("\r\n=== DOUBLE WALL MAP (FIXED LAYOUT) ===\r\n");

//   for (int y = 15; y >= 0; y--) {
//     /* --- 1. 北側の境界行（上：隣セルの南壁 / 下：自セルの北壁） --- */
//     // 隣セルの南壁
//     for (int x = 0; x < 16; x++) {
//       printf("+");
//       if (y < 15) {
//         uint8_t w_above = maze_wall[y + 1][x];
//         if (MF.FLAG.SCND)
//           w_above >>= 4;
//         printf((w_above & wall_mask[2]) ? "--- " : "    ");
//       } else {
//         printf("    ");
//       }
//     }
//     printf("+\r\n");

//     // 自セルの北壁
//     for (int x = 0; x < 16; x++) {
//       printf("+");
//       uint8_t w = maze_wall[y][x];
//       if (MF.FLAG.SCND)
//         w >>= 4;
//       printf((w & wall_mask[0]) ? "--- " : "    ");
//     }
//     printf("+\r\n");

//     /* --- 2. セル内部と東西の壁（1セル＝5文字幅） --- */
//     for (int x = 0; x < 16; x++) {
//       uint8_t w = maze_wall[y][x];
//       if (MF.FLAG.SCND)
//         w >>= 4;

//       // 西側の壁描写
//       if (x > 0) {
//         uint8_t w_left = maze_wall[y][x - 1];
//         if (MF.FLAG.SCND)
//           w_left >>= 4;
//         printf((w_left & wall_mask[1]) ? "|" : " ");
//         printf((w & wall_mask[3]) ? "|" : " ");
//       } else {
//         printf(" ");
//         printf((w & wall_mask[3]) ? "|" : " ");
//       }

//       /* --- コンテンツ描写 (3文字幅) --- */
//       bool is_goal = false;
//       for (int i = 0; i < GOAL_NUM; i++) {
//         if (x == goals[i].x && y == goals[i].y) {
//           is_goal = true;
//           break;
//         }
//       }

//       if (is_goal) {
//         printf(" G ");
//       } else if (x == mx && y == my) {
//         const char *arrows[4] = {" ^ ", " > ", " v ", " < "};
//         printf("%s", arrows[md]);
//       } else {
//         uint16_t best = MAX_COST;
//         for (int d = 0; d < 4; d++) {
//           if (st[y][x][d].dist < best)
//             best = st[y][x][d].dist;
//         }

//         // 【修正箇所】コスト未確定(MAX_COST)なら "*" を表示
//         if (best == MAX_COST) {
//           printf(" * ");
//         } else {
//           printf("%3d", best);
//         }
//       }
//     }
//     // 東端の壁
//     uint8_t w_east = maze_wall[y][15];
//     if (MF.FLAG.SCND)
//       w_east >>= 4;
//     printf((w_east & wall_mask[1]) ? "|" : " ");
//     printf("\r\n");
//   }

//   /* --- 3. 最下段（南端） --- */
//   for (int x = 0; x < 16; x++) {
//     uint8_t w = maze_wall[0][x];
//     if (MF.FLAG.SCND)
//       w >>= 4;
//     printf("+%s ", (w & wall_mask[2]) ? "---" : "   ");
//   }
//   printf("+\r\n");
// }

// void dump_route_dijkstra(void) {
//   printf("\r\n=== ROUTE ACTIONS ===\r\n");

//   for (int i = 0; route[i] != 0xFF; i++) {
//     uint8_t a = route[i];

//     if (a == 0x88)
//       printf("%3d: FWD\r\n", i);
//     else if (a == 0x44)
//       printf("%3d: RIGHT\r\n", i);
//     else if (a == 0x11)
//       printf("%3d: LEFT\r\n", i);
//     else if (a == 0x22)
//       printf("%3d: BACK\r\n", i);
//     else
//       printf("%3d: UNKNOWN %02X\r\n", i, a);
//   }

//   printf("END\r\n");
// }
// =====================================================================
//  priority_queue.c
//  最小ヒープ（min-heap）実装の優先度付きキュー
//  ダイクストラ法用：コストが小さいノードを先に取り出す
// =====================================================================
#include "global.h"
#include "logic.h"
#include "params.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// =====================================================================
//  State：4バイト／ノード、合計 4KB
//  parent は 0~1023 のフラットインデックス（10bit）
//  NO_PARENT(0x3FF) = 未設定（ゴールノード or 未到達）
// =====================================================================

#define NO_PARENT 0x3FFu
#define IDX(y, x, d) ((uint16_t)((y) * 64u + (x) * 4u + (d)))
#define IDX_Y(i) ((uint8_t)((i) >> 6))
#define IDX_X(i) ((uint8_t)(((i) >> 2) & 0xF))
#define IDX_D(i) ((uint8_t)((i) & 0x3))

State st[1024]; // 4バイト × 1024 = 4KB

// =====================================================================
//  優先度付きキュー（静的最小ヒープ、malloc 不使用）
//  PQNode: cost(2) + index(2) = 4バイト × 128 = 512バイト
// =====================================================================
#define PQ_CAPACITY 1024

typedef struct {
  uint16_t cost;
  uint16_t idx;
} PQNode; // 4バイト

static PQNode pq_data[PQ_CAPACITY];
static int pq_size = 0;

#ifdef DEBUG_PQ
static int pq_peak = 0;
#endif

static void pq_swap(int a, int b) {
  PQNode tmp = pq_data[a];
  pq_data[a] = pq_data[b];
  pq_data[b] = tmp;
}

void pq_init(void) {
  pq_size = 0;
#ifdef DEBUG_PQ
  pq_peak = 0;
#endif
}

bool pq_empty(void) { return pq_size == 0; }

void pq_push(uint8_t y, uint8_t x, uint8_t dir, uint16_t cost) {
  uint16_t idx = IDX(y, x, dir);

  // visited 済みノードは積まない（重複push の最大の発生源を除去）
  if (st[idx].visited)
    return;

  // 既に記録されているコスト以上なら積まない
  if (cost > st[idx].dist)
    return;

  // キューが満杯なら諦める（visited チェックにより実際にはほぼ発生しない）
  if (pq_size >= PQ_CAPACITY) {
#ifdef DEBUG_PQ
    printf("PQ FULL: dropped (%d,%d,d=%d) cost=%d\r\n", x, y, dir, cost);
#endif
    return;
  }

  int i = pq_size;
  pq_data[i] = (PQNode){.cost = cost, .idx = idx};
  pq_size++;

#ifdef DEBUG_PQ
  if (pq_size > pq_peak)
    pq_peak = pq_size;
#endif

  // Up-Heap（最小ヒープ）
  while (i > 0) {
    int p = (i - 1) / 2;
    if (pq_data[p].cost <= pq_data[i].cost)
      break;
    pq_swap(p, i);
    i = p;
  }
}

PQNode pq_pop(void) {
  if (pq_size <= 0) {
    return (PQNode){.cost = 0xFFFF, .idx = 0xFFFF};
  }

  PQNode result = pq_data[0];
  pq_size--;
  pq_data[0] = pq_data[pq_size];

  // Down-Heap（最小ヒープ修復）
  int i = 0;
  while (1) {
    int l = 2 * i + 1;
    int r = 2 * i + 2;
    int s = i;
    if (l < pq_size && pq_data[l].cost < pq_data[s].cost)
      s = l;
    if (r < pq_size && pq_data[r].cost < pq_data[s].cost)
      s = r;
    if (s == i)
      break;
    pq_swap(i, s);
    i = s;
  }

  return result;
}

#ifdef DEBUG_PQ
void pq_print_peak(void) {
  printf("PQ peak: %d / %d\r\n", pq_peak, PQ_CAPACITY);
}
#endif

// =====================================================================
//  その他（変更なし）
// =====================================================================

MazePosition rt_goals[1] = {{0, 0}};
MazePosition fw_goals[GOAL_NUM] = {{GOAL_X, GOAL_Y}};
// MazePosition fw_goals[GOAL_NUM] = {{GOAL_X, GOAL_Y},
//                                 {GOAL_X + 1, GOAL_Y},
//                                 {GOAL_X, GOAL_Y + 1},
//                                 {GOAL_X + 1, GOAL_Y + 1}};

static const uint8_t turn_cost[4][4] = {
    {0, 7, 50, 7},
    {7, 0, 7, 50},
    {50, 7, 0, 7},
    {7, 50, 7, 0},
};

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

// =====================================================================
//  dijkstra_multi_goal
// =====================================================================
void dijkstra_multi_goal(MazePosition goals[], uint8_t goal_count) {
  static const uint8_t wall_mask[4] = {0x08, 0x04, 0x02, 0x01};

  // --- 初期化 ---
  for (int i = 0; i < 1024; i++) {
    st[i].dist = MAX_COST;
    st[i].visited = 0;
    st[i].parent = NO_PARENT;
  }

  pq_init();

  // --- ゴールをコスト 0 でエンキュー ---
  for (int i = 0; i < goal_count; i++) {
    uint8_t gx = goals[i].x;
    uint8_t gy = goals[i].y;
    for (int d = 0; d < 4; d++) {
      uint16_t idx = IDX(gy, gx, d);
      st[idx].dist = 0;
      st[idx].parent = NO_PARENT; // ゴールは親なし
      pq_push(gy, gx, d, 0);
    }
#ifdef DEBUG_PQ
    printf("GOAL[%d]: x=%d y=%d, dist=%d\r\n", i, gx, gy,
           st[IDX(gy, gx, 0)].dist);
#endif
  }

  // --- メインループ ---
  while (!pq_empty()) {
    PQNode u = pq_pop();

    uint8_t uy = IDX_Y(u.idx);
    uint8_t ux = IDX_X(u.idx);
    uint8_t ud = IDX_D(u.idx);

    if (st[u.idx].visited)
      continue;
    st[u.idx].visited = 1;

    uint16_t cd = st[u.idx].dist;

    // predecessor（逆方向に1マス戻る）
    int8_t px = (int8_t)ux - dx[ud];
    int8_t py = (int8_t)uy - dy[ud];

    // デバッグ（元コードのまま保持）
    if (uy == 8 && ux == 7 && ud == 1) {
      uint8_t w = maze_wall[py][px];
      if (MF.FLAG.SCND)
        w >>= 4;
#ifdef DEBUG_PQ
      printf("DEBUG: u=(8,7,E) cd=%d, px=%d py=%d, "
             "wall=0x%02X, mask=0x%02X, blocked=%d\r\n",
             cd, px, py, w, wall_mask[ud], (w & wall_mask[ud]) != 0);
#endif
    }

    if (px < 0 || px >= 16 || py < 0 || py >= 16)
      continue;

    // 壁判定
    uint8_t w = maze_wall[py][px];
    if (MF.FLAG.SCND)
      w >>= 4;
    if (w & wall_mask[ud])
      continue;

    // predecessor の向きを全探索
    for (uint8_t pdir = 0; pdir < 4; pdir++) {
      uint16_t tc = turn_cost[pdir][ud];
      if (cd + 1u + tc >= MAX_COST)
        continue;

      uint16_t nd = cd + 1 + tc;
      uint16_t pid = IDX(py, px, pdir);

      if (nd < st[pid].dist) {
        st[pid].dist = nd;
        st[pid].parent = u.idx; // 10bit インデックスで親を記録
        pq_push((uint8_t)py, (uint8_t)px, pdir, nd);
      }
    }
  }

#ifdef DEBUG_PQ
  pq_print_peak();
#endif
}

// =====================================================================
//  make_route_dijkstra
//  parent インデックスを辿ってルートを生成
// =====================================================================
void make_route_dijkstra(uint8_t start_y, uint8_t start_x, uint8_t start_dir) {
  uint16_t cur = IDX(start_y, start_x, start_dir);
  int r_idx = 0;

  while (st[cur].dist > 0 && r_idx < 500) {
    uint16_t par = st[cur].parent;

    // ゴール到達 or 親なし
    if (par == NO_PARENT)
      break;

    uint8_t cur_dir = IDX_D(cur);
    uint8_t par_dir = IDX_D(par);

    if (cur_dir == par_dir) {
      route[r_idx++] = 0x88; // 直進
    } else {
      route[r_idx++] = determine_turn_op(cur_dir, par_dir);
    }

    cur = par;
  }

  route[r_idx] = 0xFF;
}

// =====================================================================
//  dump_dijkstra_map（st アクセスをフラットインデックスに変更）
// =====================================================================
void dump_dijkstra_map(uint8_t my, uint8_t mx, uint8_t md, MazePosition *goals, uint8_t goal_count) {
  static const uint8_t wall_mask[4] = {0x08, 0x04, 0x02, 0x01};

  printf("\r\n=== DOUBLE WALL MAP (FIXED LAYOUT) ===\r\n");

  for (int y = 15; y >= 0; y--) {
    // 隣セルの南壁
    for (int x = 0; x < 16; x++) {
      printf("+");
      if (y < 15) {
        uint8_t w_above = maze_wall[y + 1][x];
        if (MF.FLAG.SCND)
          w_above >>= 4;
        printf((w_above & wall_mask[2]) ? "--- " : "    ");
      } else {
        printf("    ");
      }
    }
    printf("+\r\n");

    // 自セルの北壁
    for (int x = 0; x < 16; x++) {
      printf("+");
      uint8_t w = maze_wall[y][x];
      if (MF.FLAG.SCND)
        w >>= 4;
      printf((w & wall_mask[0]) ? "--- " : "    ");
    }
    printf("+\r\n");

    // セル内部と東西の壁
    for (int x = 0; x < 16; x++) {
      uint8_t w = maze_wall[y][x];
      if (MF.FLAG.SCND)
        w >>= 4;

      if (x > 0) {
        uint8_t w_left = maze_wall[y][x - 1];
        if (MF.FLAG.SCND)
          w_left >>= 4;
        printf((w_left & wall_mask[1]) ? "|" : " ");
        printf((w & wall_mask[3]) ? "|" : " ");
      } else {
        printf(" ");
        printf((w & wall_mask[3]) ? "|" : " ");
      }

      bool is_goal = false;
      for (int i = 0; i < goal_count; i++) {
        if (x == goals[i].x && y == goals[i].y) {
          is_goal = true;
          break;
        }
      }

      if (is_goal) {
        printf(" G ");
      } else if (x == mx && y == my) {
        const char *arrows[4] = {" ^ ", " > ", " v ", " < "};
        printf("%s", arrows[md]);
      } else {
        uint16_t best = MAX_COST;
        for (int d = 0; d < 4; d++) {
          uint16_t v = st[IDX(y, x, d)].dist;
          if (v < best)
            best = v;
        }
        printf((best == MAX_COST) ? " * " : "%3d", best);
      }
    }

    uint8_t w_east = maze_wall[y][15];
    if (MF.FLAG.SCND)
      w_east >>= 4;
    printf((w_east & wall_mask[1]) ? "|" : " ");
    printf("\r\n");
  }

  // 最下段（南端）
  for (int x = 0; x < 16; x++) {
    uint8_t w = maze_wall[0][x];
    if (MF.FLAG.SCND)
      w >>= 4;
    printf("+%s ", (w & wall_mask[2]) ? "---" : "   ");
  }
  printf("+\r\n");
}

// =====================================================================
//  dump_route_dijkstra（変更なし）
// =====================================================================
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
