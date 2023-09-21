#ifndef __NAVIGATE__
#define __NAVIGATE__

// #include "Algorithm/Inc/G_init.h"
// #include "Inc/MCAL/MCAL_gpio.h"
// #include "Algorithm/Inc/Move.h"
#include <stdint.h>
// Define compass directions


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



#define north 0
#define east 1
#define south 2
#define west 3
#define M_distance 100
#define DONE_Navigating 3
// Declare external variables and arrays
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

extern uint8_t trail_north ;
extern uint8_t trail_east ;
extern uint8_t trail_south ;
extern uint8_t trail_west ;

// // Function to check if the maze is finished
// void finishcheck();

// // Function to check for walls and update maze information
// void wallcheck();

// // Function for making decisions on how to navigate
// void decisions();

// // Function to handle three-way intersections
// void three_way();

// // Function to handle two-way intersections
// void two_way();

// // Function to handle dead ends
// void dead_end();

// // Function to perform a half-turn
// void half_turn();

// // Function to handle one-way paths
// void oneway();

// void Go_Back();
// // Function to check if there's a wall using ultrasonic sensors
// // uint8_t wall_present(S_GPIO_t *portName, uint8_t trigPin, uint8_t echoPin);

// void DetermineWallTypes(uint8_t x, uint8_t y);

#include "Algorithm/Inc/Navigate.h"
// #include "Inc/HAL/HAL_Ultrasonic.h"
#include "Algorithm/Inc/Move.h"
// #include "Algorithm/Inc/PID.h"
// #include "Inc/MCAL/MCAL_eeprom.h"
// #include "Algorithm/Inc/Maze_Run.h"
// #include "Inc/MCAL/MCAL_interrupt.h"
#include "Algorithm/Inc/G_init.h"

// For every position of the robot, the potential field of every cell surrounding the robot
// will be summoned. It'll be easier for us to determined where to go with this variable.
uint8_t pot_north;
uint8_t pot_east;
uint8_t pot_south;
uint8_t pot_west;
uint8_t trail_north;
uint8_t trail_east;
uint8_t trail_south;
uint8_t trail_west;
uint8_t Navegating_times = 0;

volatile unsigned long ticks_r = 0;
volatile unsigned long ticks_l = 0;
uint8_t x_cor, y_cor;
uint8_t face;
uint8_t trail[ROWS][COLS];
uint8_t deadend[ROWS][COLS];
uint8_t pot_field[ROWS][COLS];
uint8_t decision;
uint8_t way_left, way_front, way_right;
// Check if a wall is present using ultrasonic sensors

// uint8_t wall_present(S_GPIO_t *portName, uint8_t trigPin, uint8_t echoPin)
// {
//     // Measure the distance from the sensor to the wall
//     int distance = GetUltrasonicDistance(portName, trigPin, echoPin);
//     // If the distance is greater than or equal to 100mm, assume no wall is present
//     if (distance >= M_distance)
//         return 0; // No wall
//     else
//         return 1; // Wall detected
// }

// Check for walls using ultrasonic sensors and update variables
void wallcheck()
{
    // Check if there are walls in different directions
    way_left = API_wallLeft();
    way_front = API_wallFront();
    way_right = API_wallRight();
    if (isNavigating && Navegating_times != DONE_Navigating)
    {
        API_setWall(x_cor, y_cor, face);
        // DetermineWallTypes(x_cor, y_cor);
    }
}

// Function to determine wall types and update maze values
void DetermineWallTypes(uint8_t x, uint8_t y)
{
    // Determine wall types and update maze values based on the current facing direction
    if (face == north)
    {
        if (way_left)
            setVerticalWall(x, y);
        if (way_front)
            setHorizontalWall(x, y + 1);
        if (way_right)
            setVerticalWall(x + 1, y);
    }
    else if (face == east)
    {
        if (way_left)
            setHorizontalWall(x, y + 1);
        if (way_front)
            setVerticalWall(x + 1, y);
        if (way_right)
            setHorizontalWall(x, y);
    }
    else if (face == south)
    {
        if (way_left)
            setVerticalWall(x + 1, y);
        if (way_front)
            setHorizontalWall(x, y);
        if (way_right)
            setVerticalWall(x, y);
    }
    else if (face == west)
    {
        if (way_left)
            setHorizontalWall(x, y);
        if (way_front)
            setVerticalWall(x, y);
        if (way_right)
            setHorizontalWall(x, y + 1);
    }
}

void Go_Back()
{
    half_turn();
    while (x_cor != 0 && y_cor != 0)
    {
        wallcheck();
        Find_Lowest_Path();
    }
}
// Check if the micromouse has reached the finish cell
void finishcheck()
{
    if (x_cor == x_fin && y_cor == y_fin)
    {
        // Perform actions when the finish cell is reached
        // Example: Serial.println("FINISH");

        if (Navegating_times < DONE_Navigating)
        {
            Navegating_times++;
            delay_ms(30 * 1000);
            x_cor = 0;
            y_cor = 0;
            face = north;
        }
    }
}

// Handle one-way paths
void oneway()
{
    if (way_left == 0)
    {
        // If the left path is open, turn left and move API_moveForward
        API_turnLeft();
        API_moveForward();
    }
    else if (way_right == 0)
    {
        // If the right path is open, turn right and move API_moveForward
        API_turnRight();
        API_moveForward();
    }
    else if (way_front == 0)
    {
        // If the front path is open, move API_moveForward
        API_moveForward();
    }
}

// Perform a half-turn (rotate 180 degrees clockwise)
void half_turn()
{
    // Reset the tick counters for motor rotation
    // GIE_disable();
    // ticks_l = 0;
    // ticks_r = 0;
    // GIE_enable();

    /////////////////////////////////////////// ARDUINO////////////////////////////////////////////////////
    // cli(); // Disable global interrupts
    ticks_l = 0;
    ticks_r = 0;
    // sei(); // Enable global interrupts

    // Rotate 180 degrees clockwise (40 ticks per 180 degrees)
    while ((ticks_r < (Full_cycle / 2)) || (ticks_l < (Full_cycle / 2)))
    {
        // Use the PID controller to control the wheels for a half-turn
        PID_Controller(HALF_TURN);
    }

    // Update the facing direction after the half-turn
    if (face == north)
        face = south;
    else if (face == west)
        face = east;
    else if (face == south)
        face = north;
    else if (face == east)
        face = west;
}

// Handle dead ends and find a way out
void dead_end()
{
    // Execute a half-turn to retrace the path
    API_turnLeft();
    API_turnLeft();
    deadend[x_cor][y_cor] = 1;
    trail[x_cor][y_cor]++;
    EEPROM_write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
    if (face == north)
        setHorizontalWall(x_cor, y_cor);
    else if (face == south)
        setHorizontalWall(x_cor, y_cor + 1);
    else if (face == east)
        setVerticalWall(x_cor, y_cor);
    else if (face == west)
        setVerticalWall(x_cor + 1, y_cor);
}

// ==========================================================================================
//                                         2-WAY
// ==========================================================================================

void two_way()
{
    // This subroutine will triggered when there are only two possible ways oppened.
    // Potential field will of every cell is important here in this case.

    // Access the array elements if within bounds
    pot_north = (y_cor + 1 < ROWS) ? pot_field[x_cor][y_cor + 1] : Maze_Border;
    pot_east = (x_cor + 1 < COLS) ? pot_field[x_cor + 1][y_cor] : Maze_Border;
    pot_south = (y_cor - 1 >= 0) ? pot_field[x_cor][y_cor - 1] : Maze_Border;
    pot_west = (x_cor - 1 >= 0) ? pot_field[x_cor - 1][y_cor] : Maze_Border;

    // Every possible way in the junction need to be listed so the program will know exactly
    // where to move.
    // Here are some possible way:

    // ------------------
    // Left way BLOCKED
    // ------------------
    if (way_left == 1)
    {
        // Facing north
        if (face == north)
        {
            // Deadend: Front
            if (deadend[x_cor][y_cor + 1] == 1)
            {
                API_turnRight();
                API_moveForward();
            }
            // Deadend: Right
            else if (deadend[x_cor + 1][y_cor] == 1)
            {
                API_moveForward();
            }
            // if you didnt move there before
            else if (trail[x_cor + 1][y_cor] == trail[x_cor][y_cor + 1])
            {
                if (pot_north < pot_east)
                {
                    API_moveForward();
                }
                else if (pot_north >= pot_east)
                {
                    API_turnRight();
                    API_moveForward();
                }
            }
            // if not entered the prev if then you must enter 1 of the following 2
            else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor + 1]) // moved in the right cell before?
            {
                //    if not
                API_turnRight();
                API_moveForward();
            }
            else if (trail[x_cor][y_cor + 1] < trail[x_cor + 1][y_cor]) // moved in the front cell before?
            {
                //    if not
                API_moveForward();
            }
        }

        // Facing east
        else if (face == east)
        {
            // Deadend: Front
            if (deadend[x_cor + 1][y_cor] == 1)
            {
                API_turnRight();
                API_moveForward();
            }
            // Deadend: Right
            else if (deadend[x_cor][y_cor - 1] == 1)
            {
                API_moveForward();
            }
            else if (trail[x_cor + 1][y_cor] == trail[x_cor][y_cor - 1])
            {
                if (pot_east < pot_south)
                {
                    API_moveForward();
                }
                else if (pot_east >= pot_south)
                {
                    API_turnRight();
                    API_moveForward();
                }
            }
            // if not entered the prev if then you must enter 1 of the following 2
            else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the leading cell before?
            {
                API_moveForward();
            }
            else if (trail[x_cor][y_cor - 1] < trail[x_cor + 1][y_cor]) // moved in the benith cell before?
            {
                API_turnRight();
                API_moveForward();
            }
        }
        // Facing South
        else if (face == south)
        {
            // Deadend: Front
            if (deadend[x_cor][y_cor - 1] == 1)
            {
                API_turnRight();
                API_moveForward();
            }
            // Deadend: Right
            else if (deadend[x_cor - 1][y_cor] == 1)
            {
                API_moveForward();
            }
            else if (trail[x_cor - 1][y_cor] == trail[x_cor][y_cor - 1])
            {
                if (pot_south <= pot_east)
                {
                    API_moveForward();
                }
                else if (pot_south > pot_east)
                {
                    API_turnRight();
                    API_moveForward();
                }
            }
            else if (trail[x_cor - 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
            {
                API_turnRight();
                API_moveForward();
            }
            else if (trail[x_cor][y_cor - 1] < trail[x_cor - 1][y_cor]) // moved in the leading cell before?
            {
                API_moveForward();
            }
        }

        // Facing west
        else if (face == west)
        {
            // Deadend: Front
            if (deadend[x_cor - 1][y_cor] == 1)
            {
                API_turnRight();
                API_moveForward();
            }
            // Deadend: Right
            else if (deadend[x_cor][y_cor + 1] == 1)
            {
                API_moveForward();
            }
            else if (trail[x_cor - 1][y_cor] == trail[x_cor][y_cor + 1])
            {
                if (pot_north < pot_west)
                {
                    API_turnRight();
                    API_moveForward();
                }
                else if (pot_north >= pot_west)
                {
                    API_moveForward();
                }
            }
            else if (trail[x_cor - 1][y_cor] < trail[x_cor][y_cor + 1]) // moved in the right cell before?
            {
                API_moveForward();
            }
            else if (trail[x_cor][y_cor + 1] < trail[x_cor - 1][y_cor]) // moved in the leading cell before?
            {
                API_turnRight();
                API_moveForward();
            }
        }
    }
    // ------------------
    // front way BLOCKED
    // ------------------
    else if (way_front == 1)
    {
        // Facing north
        if (face == north)
        {
            // Deadend: Left
            if (deadend[x_cor - 1][y_cor] == 1)
            {
                API_turnRight();
                API_moveForward();
            }
            // Deadend: Right
            else if (deadend[x_cor + 1][y_cor] == 1)
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (trail[x_cor + 1][y_cor] == trail[x_cor - 1][y_cor])
            {
                if (pot_west < pot_east)
                {
                    API_turnLeft();
                    API_moveForward();
                }
                else if (pot_west >= pot_east)
                {
                    API_turnRight();
                    API_moveForward();
                }
            }
            else if (trail[x_cor - 1][y_cor] < trail[x_cor + 1][y_cor]) // moved in the right cell before?
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (trail[x_cor + 1][y_cor] < trail[x_cor - 1][y_cor]) // moved in the leading cell before?
            {
                API_turnRight();
                API_moveForward();
            }
        }

        // Facing east
        else if (face == east)
        {
            // Deadend: Left
            if (deadend[x_cor][y_cor + 1] == 1)
            {
                API_turnRight();
                API_moveForward();
            }
            // Deadend: Right
            else if (deadend[x_cor][y_cor - 1] == 1)
            {
                API_turnLeft();
                API_moveForward();
            }

            else if (trail[x_cor][y_cor + 1] == trail[x_cor][y_cor - 1])
            {
                if (pot_north < pot_south)
                {
                    API_turnLeft();
                    API_moveForward();
                }
                else if (pot_north >= pot_south)
                {
                    API_turnRight();
                    API_moveForward();
                }
            }
            else if (trail[x_cor][y_cor - 1] < trail[x_cor][y_cor + 1]) // moved in the right cell before?
            {
                API_turnRight();
                API_moveForward();
            }
            else if (trail[x_cor][y_cor + 1] < trail[x_cor][y_cor - 1]) // moved in the leading cell before?
            {
                API_turnLeft();
                API_moveForward();
            }
        }

        // Facing South
        else if (face == south)
        {
            // Deadend: Left
            if (deadend[x_cor + 1][y_cor] == 1)
            {
                API_turnRight();
                API_moveForward();
            }
            // Deadend: Right
            else if (deadend[x_cor - 1][y_cor] == 1)
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (trail[x_cor + 1][y_cor] == trail[x_cor - 1][y_cor])
            {
                if (pot_west < pot_east)
                {
                    API_turnRight();
                    API_moveForward();
                }
                else if (pot_west >= pot_east)
                {
                    API_turnLeft();
                    API_moveForward();
                }
            }
            else if (trail[x_cor - 1][y_cor] < trail[x_cor + 1][y_cor]) // moved in the right cell before?
            {
                API_turnRight();
                API_moveForward();
            }
            else if (trail[x_cor + 1][y_cor] < trail[x_cor - 1][y_cor]) // moved in the leading cell before?
            {
                API_turnLeft();
                API_moveForward();
            }
        }

        // Facing west
        else if (face == west)
        {
            // Deadend: Left
            if (deadend[x_cor][y_cor - 1] == 1)
            {
                API_turnRight();
                API_moveForward();
            }
            // Deadend: Right
            else if (deadend[x_cor][y_cor + 1] == 1)
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (trail[x_cor][y_cor - 1] == trail[x_cor][y_cor + 1])
            {
                if (pot_north <= pot_south)
                {
                    API_turnRight();
                    API_moveForward();
                }
                else if (pot_north > pot_south)
                {
                    API_turnLeft();
                    API_moveForward();
                }
            }
            else if (trail[x_cor][y_cor + 1] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
            {
                API_turnRight();
                API_moveForward();
            }
            else if (trail[x_cor][y_cor - 1] < trail[x_cor][y_cor + 1]) // moved in the leading cell before?
            {
                API_turnLeft();
                API_moveForward();
            }
        }
    }
    // ------------------
    // right way BLOCKED
    // ------------------
    else if (way_right == 1)
    {
        // Facing North
        if (face == north)
        {
            // Deadend: Left
            if (deadend[x_cor - 1][y_cor] == 1)
            {
                API_moveForward();
            }
            // Deadend: Front
            else if (deadend[x_cor][y_cor + 1] == 1)
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (trail[x_cor][y_cor + 1] == trail[x_cor - 1][y_cor])
            {
                if (pot_west < pot_north)
                {
                    API_turnLeft();
                    API_moveForward();
                }
                else if (pot_west >= pot_north)
                {
                    API_moveForward();
                }
            }
            else if (trail[x_cor][y_cor + 1] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
            {
                // API_turnRight();
                API_moveForward();
            }
            else if (trail[x_cor - 1][y_cor] < trail[x_cor][y_cor + 1]) // moved in the leading cell before?
            {
                API_turnLeft();
                API_moveForward();
            }
        }
        // Facing East
        else if (face == east)
        {
            // Deadend: Left
            if (deadend[x_cor][y_cor + 1] == 1)
            {
                API_moveForward();
            }
            // Deadend: Front
            else if (deadend[x_cor + 1][y_cor] == 1)
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (trail[x_cor][y_cor + 1] == trail[x_cor + 1][y_cor])
            {
                if (pot_north < pot_east)
                {
                    API_turnLeft();
                    API_moveForward();
                }
                else if (pot_north >= pot_east)
                {
                    API_moveForward();
                }
            }
            else if (trail[x_cor][y_cor + 1] < trail[x_cor + 1][y_cor]) // moved in the right cell before?
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor + 1]) // moved in the leading cell before?
            {
                API_moveForward();
            }
        }

        // Facing South
        else if (face == south)
        {
            // Deadend: Left
            if (deadend[x_cor + 1][y_cor] == 1)
            {
                API_moveForward();
            }
            // Deadend: Front
            else if (deadend[x_cor][y_cor - 1] == 1)
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (trail[x_cor][y_cor - 1] == trail[x_cor + 1][y_cor])
            {
                if (pot_east < pot_south)
                {
                    API_turnLeft();
                    API_moveForward();
                }
                else if (pot_east >= pot_south)
                {
                    API_moveForward();
                }
            }
            else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (trail[x_cor][y_cor - 1] < trail[x_cor + 1][y_cor]) // moved in the leading cell before?
            {
                API_moveForward();
            }
        }

        // Facing West
        else if (face == west)
        {
            // Deadend: Left
            if (deadend[x_cor][y_cor - 1] == 1)
            {
                API_moveForward();
            }
            // Deadend: Front
            else if (deadend[x_cor - 1][y_cor] == 1)
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (trail[x_cor][y_cor - 1] == trail[x_cor - 1][y_cor])
            {
                if (pot_south < pot_west)
                {
                    API_turnLeft();
                    API_moveForward();
                }
                else if (pot_south >= pot_west)
                {
                    API_moveForward();
                }
            }
            else if (trail[x_cor][y_cor - 1] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (trail[x_cor - 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the leading cell before?
            {
                API_moveForward();
            }
        }
    }
}
void three_way()
{
    // This subroutine is triggered when there are 3 possible ways open.

    // Access the array elements if within bounds
    pot_north = (y_cor + 1 < ROWS) ? pot_field[x_cor][y_cor + 1] : Maze_Border;
    pot_east = (x_cor + 1 < COLS) ? pot_field[x_cor + 1][y_cor] : Maze_Border;
    pot_south = (y_cor - 1 >= 0) ? pot_field[x_cor][y_cor - 1] : Maze_Border;
    pot_west = (x_cor - 1 >= 0) ? pot_field[x_cor - 1][y_cor] : Maze_Border;

    trail_north = (y_cor + 1 < ROWS) ? trail[x_cor][y_cor + 1] : Maze_Border;
    trail_east = (x_cor + 1 < COLS) ? trail[x_cor + 1][y_cor] : Maze_Border;
    trail_south = (y_cor - 1 >= 0) ? trail[x_cor][y_cor - 1] : Maze_Border;
    trail_west = (x_cor - 1 >= 0) ? trail[x_cor - 1][y_cor] : Maze_Border;

    if (face == north)
    {
        if (trail_east < trail_north && trail_east < trail_west)
        {
            API_turnRight();
            API_moveForward();
        }
        else if ((trail_north < trail_east && trail_north < trail_west))
        {
            API_moveForward();
        }
        else if (trail_west < trail_east && trail_west < trail_north)
        {
            API_turnLeft();
            API_moveForward();
        }
        else if (trail_east == trail_north && trail_east == trail_west)
        {
            if (pot_east < pot_north && pot_east < pot_west)
            {
                API_turnRight();
                API_moveForward();
            }
            else if (pot_north < pot_east && pot_north < pot_west)
            {
                API_moveForward();
            }
            else if (pot_west < pot_east && pot_west < pot_north)
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (pot_east == pot_north && pot_east == pot_west)
            {
                API_turnRight();
                API_moveForward();
            }
            else if (pot_east == pot_north || pot_east == pot_west)
            {
                API_turnRight();
                API_moveForward();
            }
            else if (pot_north == pot_west)
            {
                API_moveForward();
            }
        }
        else if (trail_east == trail_north)
        {
            if (pot_north < pot_east)
            {
                API_moveForward();
            }
            else if (pot_north >= pot_east)
            {
                API_turnRight();
                API_moveForward();
            }
        }
        else if (trail_east == trail_west)
        {
            if (pot_west < pot_east)
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (pot_west >= pot_east)
            {
                API_turnRight();
                API_moveForward();
            }
        }
        else if (trail_north == trail_west)
        {
            if (pot_north <= pot_west)
            {
                API_moveForward();
            }
            else if (pot_north > pot_west)
            {
                API_turnLeft();
                API_moveForward();
            }
        }
    }

    else if (face == east)
    {
        if (trail_east < trail_north && trail_east < trail_south)
        {
            API_moveForward();
        }
        else if (trail_north < trail_east && trail_north < trail_south)
        {
            API_turnLeft();
            API_moveForward();
        }
        else if (trail_south < trail_east && trail_south < trail_north)
        {
            API_turnRight();
            API_moveForward();
        }
        else if (trail_east == trail_north && trail_east == trail_south)
        {
            if (pot_east < pot_north && pot_east < pot_south)
            {
                API_moveForward();
            }
            else if (pot_north < pot_east && pot_north < pot_south)
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (pot_south < pot_east && pot_south < pot_north)
            {
                API_turnRight();
                API_moveForward();
            }
            else if (pot_east == pot_north && pot_east == pot_south)
            {
                API_turnRight();
                API_moveForward();
            }
            else if (pot_east == pot_north || pot_east == pot_south)
            {
                API_moveForward();
            }
            else if (pot_north == pot_south)
            {
                API_turnRight();
                API_moveForward();
            }
        }
        else if (trail_east == trail_north)
        {
            if (pot_north < pot_east)
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (pot_north >= pot_east)
            {
                API_moveForward();
            }
        }
        else if (trail_east == trail_south)
        {
            if (pot_south < pot_east)
            {
                API_turnRight();
                API_moveForward();
            }
            else if (pot_south >= pot_east)
            {
                API_moveForward();
            }
        }
        else if (trail_south == trail_north)
        {
            if (pot_north < pot_south)
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (pot_north >= pot_south)
            {
                API_turnRight();
                API_moveForward();
            }
        }
    }
    else if (face == south)
    {
        if (trail_east < trail_south && trail_east < trail_west)
        {
            API_turnLeft();
            API_moveForward();
        }
        else if (trail_south < trail_east && trail_south < trail_west)
        {
            API_moveForward();
        }
        else if (trail_west < trail_east && trail_west < trail_south)
        {
            API_turnRight();
            API_moveForward();
        }
        else if (trail_east == trail_south && trail_east == trail_west)
        {
            if (pot_east < pot_south && pot_east < pot_west)
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (pot_south < pot_east && pot_south < pot_west)
            {
                API_moveForward();
            }
            else if (pot_west < pot_east && pot_west < pot_south)
            {
                API_turnRight();
                API_moveForward();
            }
            else if (pot_east == pot_south && pot_south == pot_west)
            {
                API_turnRight();
                API_moveForward();
            }
            else if (pot_south == pot_west || pot_west == pot_east)
            {
                API_turnRight();
                API_moveForward();
            }
            else if (pot_east == pot_south)
            {
                API_moveForward();
            }
        }

        else if (trail_east == trail_south)
        {
            if (pot_south <= pot_east)
            {
                API_moveForward();
            }
            else if (pot_south > pot_east)
            {
                API_turnLeft();
                API_moveForward();
            }
        }
        else if (trail_east == trail_west)
        {
            if (pot_west <= pot_east)
            {
                API_turnRight();
                API_moveForward();
            }
            else if (pot_west > pot_east)
            {
                API_turnLeft();
                API_moveForward();
            }
        }
        else if (trail_south == trail_west)
        {
            if (pot_south < pot_west)
            {
                API_moveForward();
            }
            else if (pot_south >= pot_west)
            {
                API_turnRight();
                API_moveForward();
            }
        }
    }
    else if (face == west)
    {
        if (trail_west < trail_north && trail_west < trail_south)
        {
            API_moveForward();
        }
        else if ((trail_north < trail_west && trail_north < trail_west))
        {
            API_turnRight();
            API_moveForward();
        }
        else if (trail_south < trail_west && trail_south < trail_north)
        {
            API_turnLeft();
            API_moveForward();
        }
        else if (trail_west == trail_north && trail_south == trail_west)
        {
            if (pot_west < pot_north && pot_west < pot_south)
            {
                API_moveForward();
            }
            else if (pot_north < pot_south && pot_north < pot_west)
            {
                API_turnRight();
                API_moveForward();
            }
            else if (pot_south < pot_west && pot_south < pot_north)
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (pot_west == pot_north && pot_west == pot_south)
            {
                API_turnRight();
                API_moveForward();
            }
            else if (pot_west == pot_north || pot_north == pot_south)
            {
                API_turnRight();
                API_moveForward();
            }
            else if (pot_north == pot_west)
            {
                API_moveForward();
            }
        }

        else if (trail_west == trail_north)
        {
            if (pot_north <= pot_west)
            {
                API_turnRight();
                API_moveForward();
            }
            else if (pot_north > pot_west)
            {
                API_moveForward();
            }
        }
        else if (trail_west == trail_south)
        {
            if (pot_south < pot_west)
            {
                API_turnLeft();
                API_moveForward();
            }
            else if (pot_south >= pot_west)
            {
                API_moveForward();
            }
        }
        else if (trail_north == trail_west)
        {
            if (pot_north <= pot_west)
            {
                API_turnRight();
                API_moveForward();
            }
            else if (pot_north > pot_west)
            {
                API_moveForward();
            }
        }
    }
}

// Decision-making routine
void decisions()
{
    // Decision-making is a quite important subroutine. With information from the wall below,
    // the algorithm now can make decisions on whether to move API_moveForward, rotate, or make a turn.
    decision = way_left + way_front + way_right;
    if (decision == 0)
    {
        // When there are 3 possible ways open, three_way subroutine will be triggered.
        // It'll never happen in any 5x5 maze. So this three_way subroutine won't
        // needs to be developed.
        three_way();
    }
    else if (decision == 1)
    {
        // When there are more than one way to move, it'll execute two_way subroutine.
        two_way();
    }
    else if (decision == 2)
    {
        // When there is one way to move, it'll execute oneway subroutine.
        oneway();
    }
    else if (decision == 3)
    {
        // When there is no way to move, it'll execute dead_end subroutine.
        dead_end();
    }
}

#endif // __NAVIGATE__
