#include "logic.h"

static PQNode pq[PQ_SIZE];
static int pq_count = 0;

void pq_init(void) { pq_count = 0; }

void pq_push(uint8_t y, uint8_t x, uint8_t dir, uint16_t dist) {
  if (pq_count >= PQ_SIZE) {
    return;
  }

  int idx = pq_count;
  pq[idx].y = y;
  pq[idx].x = x;
  pq[idx].dir = dir;
  pq[idx].dist = dist;
  pq_count++;

  while (idx > 0) {
    int parent = (idx - 1) / 2;
    if (pq[parent].dist <= pq[idx].dist)
      break;
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

  pq_count--;
  if (pq_count > 0) {
    pq[0] = pq[pq_count];
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

      PQNode temp = pq[idx];
      pq[idx] = pq[smallest];
      pq[smallest] = temp;
      idx = smallest;
    }
  }

  return ret;
}

bool pq_empty(void) { return (pq_count == 0); }
