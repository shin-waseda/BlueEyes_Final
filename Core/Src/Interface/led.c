#include "global.h"
#include "interface.h"

void led_write(LEDinfo s) {
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, s.LED.right);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, s.LED.front);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, s.LED.left);
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, s.LED.side);
}

void led_pattern_goal(void) {
  for (int i = 0; i < 3; i++) {
    LEDinfo led_p;
    led_p.LEDs = 0;
    led_p.LED.right = 1;
    led_write(led_p);
    HAL_Delay(100);
    led_p.LEDs = 0;
    led_p.LED.front = 1;
    led_write(led_p);
    HAL_Delay(100);
    led_p.LEDs = 0;
    led_p.LED.left = 1;
    led_write(led_p);
    HAL_Delay(100);
    led_p.LEDs = 0;
    led_write(led_p);
    HAL_Delay(100);
  }
}

void led_pattern_search(void) {
  for (int i = 0; i < 3; i++) {
    LEDinfo led_p;
    led_p.LEDs = 0;
    led_p.LED.right = 1;
    led_write(led_p);
    HAL_Delay(100);
    led_p.LEDs = 0;
    led_p.LED.front = 1;
    led_write(led_p);
    HAL_Delay(100);
    led_p.LEDs = 0;
    led_p.LED.left = 1;
    led_write(led_p);
    HAL_Delay(100);
    led_p.LEDs = 0;
    led_write(led_p);
    HAL_Delay(100);
  }
}