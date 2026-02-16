#include "logic.h"

static PQNode pq[PQ_SIZE];
static int pq_count = 0;

void pq_init(void) { pq_count = 0; }

void pq_push(uint8_t y, uint8_t x, uint8_t dir, uint16_t dist) {
  if (pq_count >= PQ_SIZE) {
    // エラー処理（キュー溢れ）
    return;
  }

  // 末尾に追加
  int idx = pq_count;
  pq[idx].x = x;
  pq[idx].y = y;
  pq[idx].dir = dir;
  pq[idx].dist = dist;
  pq_count++;

  // ヒープアップ（親より小さければ交換）
  while (idx > 0) {
    int parent = (idx - 1) / 2;
    if (pq[parent].dist <= pq[idx].dist)
      break;

    // 交換
    PQNode temp = pq[parent];
    pq[parent] = pq[idx];
    pq[idx] = temp;
    idx = parent;
  }
}

PQNode pq_pop(void) {
  if (pq_count == 0) {
    return (PQNode){0, 0, 0, 0xFFFF};
  }

  PQNode ret = pq[0];

  // 末尾を先頭に移動
  pq_count--;
  if (pq_count > 0) {
    pq[0] = pq[pq_count];

    // ヒープダウン（子より大きければ交換）
    int idx = 0;
    while (1) {
      int left = 2 * idx + 1;
      int right = 2 * idx + 2;
      int smallest = idx;

      if (left < pq_count && pq[left].dist < pq[smallest].dist)
        smallest = left;
      if (right < pq_count && pq[right].dist < pq[smallest].dist)
        smallest = right;

      if (smallest == idx)
        break;

      // 交換
      PQNode temp = pq[idx];
      pq[idx] = pq[smallest];
      pq[smallest] = temp;
      idx = smallest;
    }
  }

  return ret;
}

bool pq_empty(void) { return (pq_count == 0); }
