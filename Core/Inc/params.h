#ifndef PARAMS_H
#define PARAMS_H

#include "global.h"
#include "main.h"

typedef struct {
  float target_v;
  float max_neko;
  float target_catnip;
  float pre_offset_dist;
  float acc_dist;
  float const_dist;
  float decel_dist;
  float post_offset_dist;
} SlalomProfile;

// wall.c
#define WALL_BASE_FR 1100
#define WALL_BASE_FL 1100
#define WALL_BASE_R 800
#define WALL_BASE_L 900
#define KABEKIRE_SHIKIICHI 10
#define CTRL_MAX 200
#define SIDE_WALL_NEKO_P 0.8

// drive.c
#define HALF_SEC_DIST 90
#define R90_ANGLE 81
#define L90_ANGLE 81
#define R180_ANGLE 162
#define DEFAULT_NEKO 200
#define DEFAULT_VECT 200

// trajectory.c
#define ACCEL 500
#define MAX_VECT 1500

// maze.c
#define GOAL_X 1
#define GOAL_Y 0
#endif /* PARAMS_H */