#include "global.h"
#include "interface.h"
#include "logic.h"

volatile uint16_t arr_l;
volatile uint16_t arr_r;

// タイマー割り込み（1kHz 1ms周期)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == htim6.Instance) {
    pre_sensor_values = sensor_values;
    sensor_values = get_sensor_value();

    // __disable_irq();
    int16_t snap_l = OMRpulse_l;
    int16_t snap_r = OMRpulse_r;
    OMRpulse_l = 0;
    OMRpulse_r = 0;
    // __enable_irq();

    current_speed = get_current_speed(snap_l, snap_r);
    current_position = get_current_position(
        current_position.dist, current_position.angle, snap_l, snap_r);

    update_wall_state(sensor_values, pre_sensor_values, current_speed.vect);

    wall_neko_val = side_wall_control(
        sensor_values, base_sensor_values, current_speed.vect, MF.FLAG.CTRL,
        is_wall_vanishing(delta_l_cnt), is_wall_vanishing(delta_r_cnt));

    output_speed.neko += output_speed.target_catnip * TIMER_CLOCK;

    arr_l = motor_control(&htim16);
    arr_r = motor_control(&htim17);
  }

  if (htim->Instance == htim16.Instance) {
#ifdef DEBUG_PQ
    if (MF.FLAG.DEBUG_MODE) {
      calc_cnt++;
    }
#endif
    if (HAL_GPIO_ReadPin(CW_CCW_L_GPIO_Port, CW_CCW_L_Pin) == MT_FWD_L) {
      OMRpulse_l++;
    } else {
      OMRpulse_l--;
    }

    if (arr_l > 0)
      __HAL_TIM_SET_AUTORELOAD(&htim16, arr_l);
  }

  if (htim->Instance == htim17.Instance) {
    if (HAL_GPIO_ReadPin(CW_CCW_R_GPIO_Port, CW_CCW_R_Pin) == MT_FWD_R) {
      OMRpulse_r++;
    } else {
      OMRpulse_r--;
    }

    if (arr_r > 0)
      __HAL_TIM_SET_AUTORELOAD(&htim17, arr_r);
  }
}

void tim6_wait_us(uint16_t us) {
  uint16_t start = __HAL_TIM_GET_COUNTER(&htim6);
  while (1) {
    uint16_t now = __HAL_TIM_GET_COUNTER(&htim6);
    // 1000周期(1ms)でのロールオーバーを考慮した経過時間計算
    uint16_t elapsed = (now >= start) ? (now - start) : (1000 + now - start);
    if (elapsed >= us)
      break;
  }
}
