#include "global.h"
#include "interface.h"
#include "logic.h"
#include "params.h"

void adv_pos(void) {
  switch (mouse.dir) {
  case 0x00:
    mouse.y++;
    break;
  case 0x01:
    mouse.x++;
    break;
  case 0x02:
    mouse.y--;
    break;
  case 0x03:
    mouse.x--;
    break;
  }
  // tmap[mouse.y][mouse.x] = true;
}

void make_smap_adachi(void) {
  uint8_t x, y;
  for (y = 0; y <= 15; y++) {
    for (x = 0; x <= 15; x++) {
      smap[y][x] = 0xff;
    }
  }

  uint8_t m_step = 0;
  smap[goal_y][goal_x] = 0;

  do {
    for (y = 0; y <= 15; y++) {
      for (x = 0; x <= 15; x++) {
        if (smap[y][x] == m_step) {
          uint16_t m_temp = maze_wall[y][x];
          if (MF.FLAG.SCND) {
            m_temp >>= 4;
          }
          if (!(m_temp & 0x08) && y != 15) { // 北
            if (smap[y + 1][x] == 0xff) {
              smap[y + 1][x] = m_step + 1;
            }
          }
          if (!(m_temp & 0x04) && x != 15) { // 東
            if (smap[y][x + 1] == 0xff) {
              smap[y][x + 1] = m_step + 1;
            }
          }
          if (!(m_temp & 0x02) && y != 0) { // 南
            if (smap[y - 1][x] == 0xff) {
              smap[y - 1][x] = m_step + 1;
            }
          }
          if (!(m_temp & 0x01) && x != 0) { // 西
            if (smap[y][x - 1] == 0xff) {
              smap[y][x - 1] = m_step + 1;
            }
          }
        }
      }
    }
    m_step++;
  } while (smap[mouse.y][mouse.x] == 0xff);
}

void make_route() {
  uint8_t x, y;
  uint8_t dir_temp = mouse.dir;

  uint16_t i;
  for (i = 0; i < 256; i++) {
    route[i] = 0xff;
  }

  uint8_t m_step = smap[mouse.y][mouse.x];

  x = mouse.x;
  y = mouse.y;

  i = 0;
  do {
    uint8_t m_temp = maze_wall[y][x];
    if (MF.FLAG.SCND) {
      m_temp >>= 4;
    }

    if (!(m_temp & 0x08) && (smap[y + 1][x] < m_step)) { // 北
      route[i] = (0x00 - mouse.dir) & 0x03;
      m_step = smap[y + 1][x];
      y++;
    } else if (!(m_temp & 0x04) && (smap[y][x + 1] < m_step)) { // 東
      route[i] = (0x01 - mouse.dir) & 0x03;
      m_step = smap[y][x + 1];
      x++;
    } else if (!(m_temp & 0x02) && (smap[y - 1][x] < m_step)) { // 南
      route[i] = (0x02 - mouse.dir) & 0x03;
      m_step = smap[y - 1][x];
      y--;
    } else if (!(m_temp & 0x01) && (smap[y][x - 1] < m_step)) { // 西
      route[i] = (0x03 - mouse.dir) & 0x03;
      m_step = smap[y][x - 1];
      x--;
    }

    switch (route[i]) {
    case 0x00:
      route[i] = 0x88;
      break;
    case 0x01:
      turn_dir(DIR_TURN_R90);
      route[i] = 0x44;
      break;
    case 0x02:
      turn_dir(DIR_TURN_180);
      route[i] = 0x22;
      break;
    case 0x03:
      turn_dir(DIR_TURN_L90);
      route[i] = 0x11;
      break;
    default:
      route[i] = 0x00;
      break;
    }
    i++;
  } while (smap[y][x] != 0);
  mouse.dir = dir_temp;
}
