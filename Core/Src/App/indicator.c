#include "app.h"
#include "global.h"
#include "interface.h"

int select_mode(int mode) {
  mode = 0;
  printf("Mode : %d\n", mode);

  while (1) {
    LC.LED.right = (mode & 0b001) != 0;
    LC.LED.front = (mode & 0b010) != 0;
    LC.LED.left = (mode & 0b100) != 0;
    led_write(LC);

    if (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET) {
      HAL_Delay(100);
      while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET)
        ;
      mode++;
      if (mode > 7) {
        mode = 0;
      }
      printf("Mode : %d\n", mode);
    }

    if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET) {
      HAL_Delay(100);
      while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET)
        ;
      mode--;
      if (mode < 0) {
        mode = 7;
      }
      printf("Mode : %d\n", mode);
    }

    if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_RESET) {
      HAL_Delay(100);
      while (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_RESET)
        ;
      return mode;
    }
  }
}