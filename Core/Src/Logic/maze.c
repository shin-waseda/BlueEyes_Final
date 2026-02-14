#include "global.h"
#include "logic.h"
#include <stdint.h>

void map_init(void) {
  uint8_t x, y;

  for (y = 0; y < 16; y++) {
    for (x = 0; x < 16; x++) {
      maze_wall[y][x] = 0xf0;
      tmap[y][x] = false;
    }
  }

  for (y = 0; y < 16; y++) {
    maze_wall[y][0] |= 0xf1;
    maze_wall[y][15] |= 0xf4;
  }
  for (x = 0; x < 16; x++) {
    maze_wall[0][x] |= 0xf2;
    maze_wall[15][x] |= 0xf8;
  }
}

void update_map_info(MazePosition s, uint8_t wall_info) {
  uint8_t wall = (wall_info >> s.dir) & 0x0f;

  wall |= (wall << 4);
  wall |= (wall >> 4);

  maze_wall[s.y][s.x] = wall;

  if (s.y != 15) {
    if (wall & 0x88) {
      maze_wall[s.y + 1][s.x] |= 0x22;
    } else {
      maze_wall[s.y + 1][s.x] &= 0xDD;
    }
  }

  if (s.x != 15) {
    if (wall & 0x44) {
      maze_wall[s.y][s.x + 1] |= 0x11;
    } else {
      maze_wall[s.y][s.x + 1] &= 0xEE;
    }
  }

  if (s.y != 0) {
    if (wall & 0x22) {
      maze_wall[s.y - 1][s.x] |= 0x88;
    } else {
      maze_wall[s.y - 1][s.x] &= 0x77;
    }
  }

  if (s.x != 0) {
    if (wall & 0x11) {
      maze_wall[s.y][s.x - 1] |= 0x44;
    } else {
      maze_wall[s.y][s.x - 1] &= 0xBB;
    }
  }
}

void turn_dir(uint8_t t_pat) { mouse.dir = (mouse.dir + t_pat) & 0x03; }

void store_map_in_flash(void) {
  flash_unlock_erase();
  int i;
  for (i = 0; i < 16; i++) {
    int j;
    for (j = 0; j < 16; j++) {
      flash_write_2byte(i * 16 + j, (uint16_t)maze_wall[i][j]);
    }
  }
  flash_lock();
}

void load_map_from_flash(void) {
  int i;
  for (i = 0; i < 16; i++) {
    int j;
    for (j = 0; j < 16; j++) {
      maze_wall[i][j] = (uint8_t)flash_read_2byte(i * 16 + j);
    }
  }
}
