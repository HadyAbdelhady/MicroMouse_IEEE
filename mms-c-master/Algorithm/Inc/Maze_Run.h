#ifndef __MAZE_RUN__
#define __MAZE_RUN__
#include "Algorithm/Inc/G_init.h"
#include "Algorithm/Inc/Move.h"
#include "Algorithm/Inc/Navigate.h"
// #include "Inc/MCAL/MCAL_eeprom.h"

extern uint8_t PATH[ROWS][COLS];

extern uint8_t Blocking_V_Walls[ROWS + 1][COLS]; // Vertical walls Readings
extern uint8_t Blocking_H_Walls[ROWS][COLS + 1]; // Horizontal walls Readings

#define v_Walls_BASE ((ROWS - 1) * 10 + (COLS))     // last address of the eeprom to Save the Cells values
#define h_Walls_BASE 2 * ((ROWS - 1) * 10 + (COLS)) // last address of the eeprom to Save the V_Walls values


#define north_west 0
#define north_east 1
#define south_west 2
#define south_east 3

void Find_Lowest_Path();
uint8_t getLowestPotential(uint8_t x, uint8_t y);
void setHorizontalWall(uint8_t x, uint8_t y);
void setVerticalWall(uint8_t x, uint8_t y);
void readMazeFromEEPROM();
uint8_t max_value(uint8_t n, uint8_t s, uint8_t e, uint8_t w);

#endif