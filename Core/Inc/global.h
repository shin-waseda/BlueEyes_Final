#ifndef GLOBAL_H
#define GLOBAL_H

#include "main.h"
#include "params.h"
#include "stdbool.h"

#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a, b) (((a) < (b)) ? (a) : (b))

typedef union {
  uint16_t FLAGS;
  struct {
    uint16_t CTRL : 1;
    uint16_t WALL_VANISH_R : 1;
    uint16_t WALL_VANISH_L : 1;
    uint16_t WALL_VANISH_F : 1;
    uint16_t SCND : 1;
    uint16_t RETURN : 1;
    uint16_t CALC : 1;
    uint16_t OTHER : 9;
  } FLAG;
} mouse_flag_t;

typedef struct {
  uint16_t SensorValueR;
  uint16_t SensorValueFR;
  uint16_t SensorValueFL;
  uint16_t SensorValueL;
} SensorValues_t;

typedef struct {
  float vect;
  float neko;
  float target_catnip;
} SpeedController_t;

typedef struct {
  float dist;
  float angle;
} Position_t;

typedef struct {
  uint8_t x;
  uint8_t y;
  uint8_t dir;
} MazePosition;

typedef union {
  uint8_t LEDs;
  struct {
    uint8_t right : 1;
    uint8_t front : 1;
    uint8_t left : 1;
    uint8_t side : 1;
    uint8_t other : 4;
  } LED;
} LEDinfo;

// Snapshot
extern volatile SensorValues_t sensor_values;
extern volatile SensorValues_t pre_sensor_values;
extern volatile SensorValues_t base_sensor_values;
extern volatile SpeedController_t current_speed;
extern volatile Position_t current_position;
extern volatile SlalomProfile current_slalom_profile;
extern volatile MazePosition mouse;
extern volatile float wall_neko_val;

// Flag
extern volatile mouse_flag_t MF;
// mode??
extern volatile SpeedController_t target_speed;

// Indicator
extern volatile LEDinfo LC;

// Command
extern volatile SpeedController_t output_speed;

// Map Info
extern volatile uint8_t goal_x;
extern volatile uint8_t goal_y;
extern volatile uint8_t maze_wall[16][16];
extern volatile uint16_t smap[16][16];
extern volatile bool tmap[16][16];
extern volatile uint16_t route[256];
extern volatile uint16_t r_cnt;

#endif /* GLOBAL_H */
