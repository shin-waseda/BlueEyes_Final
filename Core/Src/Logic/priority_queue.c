// #include "logic.h"

// // PQ_SIZEを調整（256程度でも16x16なら十分なことが多いです）
// #define PQ_SIZE 980

// static PQNode pq[PQ_SIZE];
// static int pq_count = 0;
// extern State st[16][16][4];

// void pq_init(void) { pq_count = 0; }

// void pq_push(uint8_t y, uint8_t x, uint8_t dir, uint16_t dist) {
//   if (pq_count >= PQ_SIZE) {
//     printf("PQ OVERFLOW! count=%d\r\n", pq_count); // カウント表示に変更

//     return;
//   }

//   // 既に確定済みのノードは push しない ← これを追加
//   if (st[y][x][dir].visited)
//     return;

//   // 現在の最良値より悪い dist は push しない ← これも追加
//   if (dist > st[y][x][dir].dist)
//     return;

//   int i = pq_count++;
//   // 穴埋め法：正しい位置が見つかるまで親を下にずらす
//   while (i > 0) {
//     int p = (i - 1) / 2;
//     if (pq[p].dist <= dist)
//       break;
//     pq[i] = pq[p];
//     i = p;
//   }
//   pq[i].y = y;
//   pq[i].x = x;
//   pq[i].dir = dir;
//   pq[i].dist = dist;
// }

// PQNode pq_pop(void) {
//   if (pq_count == 0)
//     return (PQNode){0, 0, 0, 0xFF};

//   PQNode res = pq[0];
//   PQNode last = pq[--pq_count];

//   if (pq_count > 0) {
//     int i = 0;
//     // 穴埋め法：正しい位置が見つかるまで子を上に引き上げる
//     while (i * 2 + 1 < pq_count) {
//       int a = i * 2 + 1;
//       int b = i * 2 + 2;
//       if (b < pq_count && pq[b].dist < pq[a].dist)
//         a = b;
//       if (pq[a].dist >= last.dist)
//         break;
//       pq[i] = pq[a];
//       i = a;
//     }
//     pq[i] = last;
//   }
//   return res;
// }

// bool pq_empty(void) { return (pq_count == 0); }

/*







*/

#include "logic.h"
#include <stdio.h>
#include <stdlib.h>

// 優先度付きキューの構造体
typedef struct {
  int *data;    // 動的配列へのポインタ
  int size;     // 現在の要素数
  int capacity; // 現在確保されているメモリの容量
} PriorityQueue;

// 初期化（初期容量を指定）
void initQueue(PriorityQueue *pq, int initial_capacity) {
  pq->data = (int *)malloc(initial_capacity * sizeof(int));
  if (pq->data == NULL) {
    fprintf(stderr, "Memory allocation failed\n");
    exit(1);
  }
  pq->size = 0;
  pq->capacity = initial_capacity;
}

// メモリ解放
void freeQueue(PriorityQueue *pq) {
  free(pq->data);
  pq->data = NULL;
  pq->size = 0;
  pq->capacity = 0;
}

void swap(int *a, int *b) {
  int temp = *a;
  *a = *b;
  *b = temp;
}

// エンキュー（自動リサイズ機能付き）
void push(PriorityQueue *pq, int value) {
  // 容量がいっぱいになったら拡張する
  if (pq->size == pq->capacity) {
    // 容量を2倍にする
    int new_capacity = pq->capacity * 2;
    int *new_data = (int *)realloc(pq->data, new_capacity * sizeof(int));

    if (new_data == NULL) {
      fprintf(stderr, "Memory reallocation failed\n");
      return;
    }

    pq->data = new_data;
    pq->capacity = new_capacity;
    printf("[Info] Queue resized: %d -> %d\n", pq->capacity / 2, pq->capacity);
  }

  // ここから下は固定長版と同じロジック（Up-Heap）
  int i = pq->size;
  pq->data[i] = value;
  pq->size++;

  while (i > 0) {
    int parent = (i - 1) / 2;
    if (pq->data[parent] >= pq->data[i]) {
      break;
    }
    swap(&pq->data[parent], &pq->data[i]);
    i = parent;
  }
}

// デキュー（ロジックは固定長版と同じ）
int pop(PriorityQueue *pq) {
  if (pq->size <= 0) {
    printf("Queue underflow\n");
    return -1;
  }

  int max_val = pq->data[0];
  pq->size--;
  pq->data[0] = pq->data[pq->size];

  int i = 0;
  while (1) {
    int left = 2 * i + 1;
    int right = 2 * i + 2;
    int largest = i;

    if (left < pq->size && pq->data[left] > pq->data[largest]) {
      largest = left;
    }
    if (right < pq->size && pq->data[right] > pq->data[largest]) {
      largest = right;
    }

    if (largest == i) {
      break;
    }

    swap(&pq->data[i], &pq->data[largest]);
    i = largest;
  }

  return max_val;
}