#ifndef APP_H
#define APP_H

#include "global.h"
#include "main.h"

// mode.c
uint8_t execute_mode(uint8_t mode);

// indicator.c
int select_mode(int mode);

// search.c
extern volatile uint16_t r_cnt;
void searchB_adachi(bool is_slalom);

#endif /* APP_H */
