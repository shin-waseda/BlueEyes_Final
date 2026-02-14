#include "global.h"
#include "interface.h"

void led_write(LEDinfo s) {
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, s.LED.right);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, s.LED.front);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, s.LED.left);
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, s.LED.side);
}
