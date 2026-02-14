#include "global.h"
#include "interface.h"

void sensor_init(void) {

  sensor_values.SensorValueFL = 0;
  sensor_values.SensorValueFR = 0;
  sensor_values.SensorValueL = 0;
  sensor_values.SensorValueR = 0;

  pre_sensor_values.SensorValueFL = 0;
  pre_sensor_values.SensorValueFR = 0;
  pre_sensor_values.SensorValueL = 0;
  pre_sensor_values.SensorValueR = 0;

  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
  HAL_TIM_Base_Start(&htim6);
}

SensorValues_t get_sensor_value() {
  SensorValues_t result;

  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc1);
  HAL_GPIO_WritePin(IR_SIDE_GPIO_Port, IR_SIDE_Pin, GPIO_PIN_RESET);
  HAL_ADC_PollForConversion(&hadc1, 1);
  uint16_t r_off = HAL_ADC_GetValue(&hadc2);
  uint16_t l_off = HAL_ADC_GetValue(&hadc1);

  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1);
  uint16_t fl_off = HAL_ADC_GetValue(&hadc2);
  uint16_t fr_off = HAL_ADC_GetValue(&hadc1);

  HAL_GPIO_WritePin(IR_SIDE_GPIO_Port, IR_SIDE_Pin, GPIO_PIN_SET);
  tim6_wait_us(IR_WAIT_US);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 1);
  uint16_t r_on = HAL_ADC_GetValue(&hadc2);
  uint16_t l_on = HAL_ADC_GetValue(&hadc1);
  HAL_GPIO_WritePin(IR_SIDE_GPIO_Port, IR_SIDE_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(IR_FRONT_GPIO_Port, IR_FRONT_Pin, GPIO_PIN_SET);
  tim6_wait_us(IR_WAIT_US);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc2, 1);
  uint16_t fl_on = HAL_ADC_GetValue(&hadc2);
  uint16_t fr_on = HAL_ADC_GetValue(&hadc1);
  HAL_GPIO_WritePin(IR_FRONT_GPIO_Port, IR_FRONT_Pin, GPIO_PIN_RESET);

  result.SensorValueR = r_on - r_off;
  result.SensorValueFR = fr_on - fr_off;
  result.SensorValueFL = fl_on - fl_off;
  result.SensorValueL = l_on - l_off;

  return result;
}