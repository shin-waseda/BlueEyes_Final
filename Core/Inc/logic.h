#ifndef LOGIC_H
#define LOGIC_H

#include "global.h"
#include "main.h" // これがHALヘッダーを含んでいるはず
#include "params.h"
#include <stdbool.h>
#include <stdint.h>

// wall.c
extern volatile int16_t delta_l_cnt;
extern volatile int16_t delta_r_cnt;
uint8_t get_wall_info(SensorValues_t s);
void get_base_sensor_values(void);
void get_base_sensor_value(void);
uint16_t wall_check(uint16_t sensor_val, uint16_t pre_val, float v,
                    uint16_t current_cnt, uint16_t limit);
void update_wall_state(SensorValues_t s, SensorValues_t pre_s, float v);
float side_wall_control(SensorValues_t s, SensorValues_t base_s,
                        float current_v, bool is_ctrl_on, bool vanish_l,
                        bool vanish_r);
bool is_wall_vanishing(int16_t cnt);

// trajectory.c
typedef enum {
  MOTION_TYPE_STRAIGHT,
  MOTION_TYPE_ROTATE,
  MOTION_TYPE_SLALOM
} MotionType;

typedef struct {
  float acc_dist;
  float decel_dist;
  float const_dist;
  float pri_offset_dist;
  float post_offset_dist;
  double target_neko_catnip;
  double max_neko;
  double target_v;
} SlalomParam;

typedef struct {
  MotionType type;
  float target_limit;
  float start_v, end_v, max_v, accel;
  float start_neko, end_neko, max_neko, catnip;
  bool use_wall_ctrl;
} MotionConfig;

SpeedController_t calculate_trajectory(MotionConfig config, float current_p);

// dirve.c
void hitting_step(void (*rotate_func)(void));
void drive_wait(void);
void drive_R(float angle, float target_neko);
void drive_R_90R(void);
void drive_R_90L(void);
void drive_R_180(void);
void drive_c(float dist);
void drive_trapezoid(float dist, float target_v, float end_v, float max_v);
void drive_slalom(SlalomProfile p, bool direction);
void drive_S_R90(void);
void drive_straight(float dist, float target_v, float catnip);
void enable_motor(void);
void disable_motor(void);
void reset_current_position(void);
void set_position(bool sw);
void one_sectionU(void);
void half_sectionA(void);
void half_sectionD(void);
void turn_right(bool is_slalom);
void turn_left(bool is_slalom);
void turn_back(bool is_last);
void drive_A(float dist);
void drive_D(float dist);
void drive_U(float dist);
void drive_S_L90(void);
float get_target_v(float current_dist, float target_dist, float accel,
                   float max_v, float current_v, float target_v);
void start_sequence(void);

// route.c / maze.c
void adv_pos(void);
void make_smap_adachi(void);
void make_route(void);
HAL_StatusTypeDef flash_unlock_erase(void);
HAL_StatusTypeDef flash_write_2byte(uint32_t address, uint16_t data);
uint16_t flash_read_2byte(uint32_t address);
HAL_StatusTypeDef flash_lock(void);
void store_map_in_flash(void);
void load_map_from_flash(void);

// App層の補助
void test_drive(void);

// search.c
#define DIR_TURN_R90 0x01
#define DIR_TURN_L90 0xff
#define DIR_TURN_180 0x02
extern volatile uint16_t r_cnt;
void searchB_adachi(bool is_slalom);
void get_wall(void);
void write_map(void);
void conf_route(void);
void searchB_dijkstra(bool is_slalom);
void last_run(void);
void led_pattern_goal(void);
void dump_dijkstra_map(uint8_t my, uint8_t mx, uint8_t md);
void dump_route_dijkstra(void);
void drive_calc_offset(float dist);

// maze.c
void map_init(void);
void update_map_info(MazePosition s, uint8_t wall_info);
void turn_dir(uint8_t t_pat);

// priority_queue.c
// typedef struct {
//   uint8_t y : 4;
//   uint8_t x : 4;
//   uint8_t dir : 2;
//   uint16_t dist;
// } PQNode;
// void pq_init(void);
// void pq_push(uint8_t y, uint8_t x, uint8_t dir, uint16_t dist);
// PQNode pq_pop(void);
// bool pq_empty(void);

// dijkstra.c
typedef struct {
  uint16_t dist;
  uint16_t parent : 10; // 0~1023, 0x3FF = NO_PARENT
  uint16_t visited : 1;
  // 残り 5bit はパディング（将来の拡張用）
} State; // sizeof == 4バイト             // これで 4バイト になり、合計 4 KB
         // まで半減します
extern MazePosition goals_quad[4];
extern bool use_quad_goal;
extern State st[1024];
extern MazePosition goals[GOAL_NUM];
void conf_route_dijkstra(void);

#define MAX_COST 10000u
void dijkstra_multi_goal(MazePosition goals[], uint8_t goal_count);
void make_route_dijkstra(uint8_t start_y, uint8_t start_x, uint8_t start_dir);

// logic.h

#endif /* LOGIC_H */
