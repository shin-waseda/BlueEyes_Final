#include "global.h"

#ifdef DEBUG_PQ
uint16_t calc_cnt = 0;
#endif

// Snapshot
volatile mouse_flag_t MF;

// Sensor data
volatile SensorValues_t sensor_values;
volatile SensorValues_t pre_sensor_values;
volatile SensorValues_t base_sensor_values;
volatile SpeedController_t current_speed;
volatile Position_t current_position;
volatile float wall_neko_val = 0.0f;
volatile MazePosition mouse;
volatile uint8_t goal_x;
volatile uint8_t goal_y;
volatile uint16_t r_cnt;
// volatile bool tmap[16][16];
volatile uint32_t tmap_bits[8];
volatile SlalomProfile current_slalom_profile;

volatile SpeedController_t output_speed;
volatile SpeedController_t target_speed;

volatile LEDinfo LC;

volatile uint8_t route[512];
volatile uint16_t smap[16][16];
volatile uint8_t maze_wall[16][16];
/* USER CODE BEGIN 0 */
