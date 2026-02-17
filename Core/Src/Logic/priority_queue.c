#include "logic.h"

// PQ_SIZEを調整（256程度でも16x16なら十分なことが多いです）
#define PQ_SIZE 980

static PQNode pq[PQ_SIZE];
static int pq_count = 0;
extern State st[16][16][4];

void pq_init(void) { pq_count = 0; }

void pq_push(uint8_t y, uint8_t x, uint8_t dir, uint16_t dist) {
  if (pq_count >= PQ_SIZE) {
    printf("PQ OVERFLOW! count=%d\r\n", pq_count); // カウント表示に変更

    return;
  }

  // 既に確定済みのノードは push しない ← これを追加
  if (st[y][x][dir].visited)
    return;

  // 現在の最良値より悪い dist は push しない ← これも追加
  if (dist > st[y][x][dir].dist)
    return;

  int i = pq_count++;
  // 穴埋め法：正しい位置が見つかるまで親を下にずらす
  while (i > 0) {
    int p = (i - 1) / 2;
    if (pq[p].dist <= dist)
      break;
    pq[i] = pq[p];
    i = p;
  }
  pq[i].y = y;
  pq[i].x = x;
  pq[i].dir = dir;
  pq[i].dist = dist;
}

PQNode pq_pop(void) {
  if (pq_count == 0)
    return (PQNode){0, 0, 0, 0xFF};

  PQNode res = pq[0];
  PQNode last = pq[--pq_count];

  if (pq_count > 0) {
    int i = 0;
    // 穴埋め法：正しい位置が見つかるまで子を上に引き上げる
    while (i * 2 + 1 < pq_count) {
      int a = i * 2 + 1;
      int b = i * 2 + 2;
      if (b < pq_count && pq[b].dist < pq[a].dist)
        a = b;
      if (pq[a].dist >= last.dist)
        break;
      pq[i] = pq[a];
      i = a;
    }
    pq[i] = last;
  }
  return res;
}

bool pq_empty(void) { return (pq_count == 0); }
