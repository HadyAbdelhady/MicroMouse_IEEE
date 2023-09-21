#ifndef __Visualize__
#define __Visualize__

#include <stdint.h>
#include "MCAL_timer.h"
#include <math.h>


#define NAV_switch 0
#define RUN_switch 1
#define CLR_EEPROM_switch 2

// Define constants for maze dimensions
#define ROWS 16
#define COLS 16
#define Maze_Border 255 // A large number to prevent out-of-range
#define x_fin 8
#define y_fin 8

// Define Ultrasonic pins
#define lecho 13
#define ltrigger 12
#define fecho 11
#define ftrigger 8
#define recho 7
#define rtrigger 4


// Declare external timer configuration variables
extern Timer_Config_t *configuartion;
extern TIMER1_Config_t *configuartion_1;

// Declare a 2D array to represent maze cells
extern uint8_t Cell[ROWS][COLS];
extern uint8_t PATH[ROWS][COLS];

extern uint8_t isNavigating ;


// Function to initialize maze cells for flood-fill algorithm
void Cells_init_FloodFill(void);

// Function to initialize the application
void G_Init();

// Function to initialize maze cells for flood-fill algorithm (duplicate declaration)
void Cells_init_FloodFill(void);

// Functions to add left and right turns (provide descriptions)
void add_left();
void add_right();
void Reset_eeprom();

#endif // __Visualize__
