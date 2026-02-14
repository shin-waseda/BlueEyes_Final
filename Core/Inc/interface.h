#ifndef INTERFACE_H
#define INTERFACE_H

#include "global.h"
#include "main.h"

// hard info
#define TIMER_INTERVAL 1000000.0f
#define TIMER_CLOCK 0.001f // [s]
#define WHEEL_DIA 52.5f    // [mm]
#define PULSE_PRE_ROTATION 400
#define WHEEL_DISTANCE 86.0f // [mm]
#define PULSE_DISTANCE (WHEEL_DIA * M_PI / PULSE_PRE_ROTATION)

// sensor.c
void sensor_init(void);
SensorValues_t get_sensor_value();

// motor.c
#define MT_FWD_L GPIO_PIN_SET
#define MT_BACK_L GPIO_PIN_RESET
#define MT_FWD_R GPIO_PIN_RESET
#define MT_BACK_R GPIO_PIN_SET
uint16_t motor_control(TIM_HandleTypeDef *htim);
void motor_init(void);
void drive_start(void);
void drive_stop(void);
void enable_motor(void);
void disable_motor(void);

// flash.c
#define EEPROM_START_ADDRESS (uint32_t)0x0800F800
HAL_StatusTypeDef flash_lock();
HAL_StatusTypeDef flash_unlock_erase();
HAL_StatusTypeDef flash_write_2byte(uint32_t address, uint16_t data);
HAL_StatusTypeDef flash_write_4byte(uint32_t address, uint32_t data);
uint16_t flash_read_2byte(uint32_t address);
uint32_t flash_read_4byte(uint32_t address);

// odometry.c
SpeedController_t get_current_speed(int16_t pulse_l, int16_t pulse_r);
Position_t get_current_position(float dist, float angle, int16_t pulse_l,
                                int16_t pulse_r);
void reset_current_position(void);
void reset_odometry_pulses(void);

extern volatile uint16_t OMRarr_l;  // 仮想オドメトリ用
extern volatile uint16_t OMRarr_r;  // 仮想オドメトリ用
extern volatile int16_t OMRpulse_l; // 仮想オドメトリ用 (符号付きに変更)
extern volatile int16_t OMRpulse_r; // 仮想オドメトリ用 (符号付きに変更)

// led.c
void led_write(LEDinfo s);

// intterupt.c
void tim6_wait_us(uint16_t us);
#define IR_WAIT_US 15
#define VECTOR_TO_ARR ((TIMER_INTERVAL * M_PI * WHEEL_DIA) / PULSE_PRE_ROTATION)

#endif /* INTERFACE_H */
