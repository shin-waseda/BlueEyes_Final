#include "global.h"
#include "interface.h"
#include "logic.h"
#include <math.h>

uint16_t motor_control(TIM_HandleTypeDef *htim) {
  uint16_t tmp_arr;

  if (htim->Instance == htim16.Instance) {
    float tmp_vect_l = output_speed.vect + output_speed.neko - wall_neko_val;

    HAL_GPIO_WritePin(CW_CCW_L_GPIO_Port, CW_CCW_L_Pin,
                      (tmp_vect_l >= 0) ? MT_FWD_L : MT_BACK_L);
    float v_abs = fabsf(tmp_vect_l);
    if (v_abs < 1.0f) {
      v_abs = 1.0f;
    }
    tmp_arr = (uint16_t)(VECTOR_TO_ARR / v_abs);
    OMRarr_l = tmp_arr;
  }
  // tim17割り込み
  if (htim->Instance == htim17.Instance) {
    float tmp_vect_r = output_speed.vect - output_speed.neko + wall_neko_val;

    HAL_GPIO_WritePin(CW_CCW_R_GPIO_Port, CW_CCW_R_Pin,
                      (tmp_vect_r >= 0) ? MT_FWD_R : MT_BACK_R);
    HAL_GPIO_WritePin(CW_CCW_R_2_GPIO_Port, CW_CCW_R_2_Pin,
                      (tmp_vect_r >= 0) ? MT_FWD_R : MT_BACK_R);

    float v_abs = fabsf(tmp_vect_r);
    if (v_abs < 1.0f) {
      v_abs = 1.0f;
    }
    tmp_arr = (uint16_t)(VECTOR_TO_ARR / v_abs);
    OMRarr_r = tmp_arr;
  }

  return tmp_arr;
}

void motor_init(void) {
  OMRpulse_l = OMRpulse_r = 0;
  output_speed = (SpeedController_t){0.0f, 0.0f, 0.0f};
  current_position = (Position_t){0.0f, 0.0f};

  MF.FLAGS = 0;

  disable_motor();
  HAL_GPIO_WritePin(CW_CCW_L_GPIO_Port, CW_CCW_L_Pin, MT_FWD_L);
  HAL_GPIO_WritePin(CW_CCW_R_GPIO_Port, CW_CCW_R_Pin, MT_FWD_R);
  HAL_GPIO_WritePin(CW_CCW_R_2_GPIO_Port, CW_CCW_R_2_Pin, MT_FWD_R);

  __HAL_TIM_SET_AUTORELOAD(&htim16, (VECTOR_TO_ARR / 300));
  __HAL_TIM_SET_AUTORELOAD(&htim17, (VECTOR_TO_ARR / 300));
}

void enable_motor(void) {
  HAL_GPIO_WritePin(M3_GPIO_Port, M3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(M3_2_GPIO_Port, M3_2_Pin, GPIO_PIN_RESET);
}

void disable_motor(void) {
  HAL_GPIO_WritePin(M3_GPIO_Port, M3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(M3_2_GPIO_Port, M3_2_Pin, GPIO_PIN_SET);
}

void drive_start(void) {

  OMRpulse_l = OMRpulse_r = 0;
  reset_current_position();

  __HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

  __HAL_TIM_CLEAR_FLAG(&htim17, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim17, TIM_IT_UPDATE);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
}

void drive_stop(void) {

  HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);

  __HAL_TIM_SET_COUNTER(&htim16, 0);
  __HAL_TIM_SET_COUNTER(&htim17, 0);

  // 速度成分を完全にリセット
  // output_speed.vect = 0;
  // output_speed.neko = 0;
  // output_speed.target_catnip = 0;
}
