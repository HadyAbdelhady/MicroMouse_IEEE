#ifndef __NAVIGATE__
#define __NAVIGATE__

#include "G_init.h"
#include "MCAL_gpio.h"
#include "Move.h"

// Define compass directions
#define north 0
#define east 1
#define south 2
#define west 3
#define THRESHOLD_DISTANCE 100
#define LARGEST_DISTANCE 2880
#define SMALLEST_DISTANCE 0
#define DONE_Navigating 3
// Declare external variables and arrays
extern uint8_t FirtPath;
extern uint8_t x_cor;
extern uint8_t y_cor;
extern uint8_t face;
extern uint8_t trail[ROWS][COLS];
extern uint8_t deadend[ROWS][COLS];
extern uint8_t pot_field[ROWS][COLS];
extern uint8_t decision;
extern uint8_t Navegating_times;
extern uint8_t way_left;
extern uint8_t way_front;
extern uint8_t way_right;
// For every position of the robot, the potential field of every cell surrounding the robot
// will be summoned. It'll be easier for us to determined where to go with this variable.
extern uint8_t pot_north;
extern uint8_t pot_east;
extern uint8_t pot_south;
extern uint8_t pot_west;

extern uint8_t trail_north;
extern uint8_t trail_east;
extern uint8_t trail_south;
extern uint8_t trail_west;

// Function to check if the maze is finished
void finishcheck();

// Function to check for walls and update maze information
void wallcheck();

// Function for making decisions on how to navigate
void decisions();

// Function to handle three-way intersections
void three_way();

// Function to handle two-way intersections
void two_way();

// Function to handle dead ends
void dead_end();

// Closes a cell after reaching a dead end.
void Close_Cell_After_Deadend();

// Function to perform a half-turn
void half_turn();

// Function to handle one-way paths
void oneway();

// Function to check if there's a wall using ultrasonic sensors
uint8_t wall_present(S_GPIO_t *portName, uint8_t trigPin, uint8_t echoPin);

void DetermineWallTypes(uint8_t x, uint8_t y);

#endif // __NAVIGATE__
