#ifndef APP_H
#define APP_H

#include "global.h"
#include "main.h"

// mode.c
uint8_t execute_mode(uint8_t mode);
void select_speed(uint8_t mode);
SlalomProfile select_S90_param(uint8_t mode);
void test_drive(void);
void test_straight(void);
void test_rotate(void);
void test_slalom(void);


// indicator.c
int select_mode(int mode);

// search.c
void search_init(void);
void searchB_adachi(bool is_slalom);
void searchB_dijkstra(bool is_slalom);
void get_wall(void);
void write_map(void);
void conf_route(void);
void conf_route_dijkstra(void);
void dump_adachi_map(void);
void dump_dijkstra_map(uint8_t my, uint8_t mx, uint8_t md);

void conf_route_dijkstra(void);

#endif /* APP_H */
