#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "API.h"
#define BUFFER_SIZE 32

// Define movement states
#define FORWARD 0
#define HALF_TURN 1
#define RIGHT 2
#define LEFT 3
#define STOP 4
uint8_t isNavigating = -1;

#define Full_cycle 92

#define north 0
#define east 1
#define south 2
#define west 3
#define M_distance 100
#define DONE_Navigating 3
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

// Define constants for maze dimensions
#define ROWS 16
#define COLS 16
#define Maze_Border 255 // A large number to prevent out-of-range
#define x_fin 8
#define y_fin 8
uint8_t x_cor, y_cor;
uint8_t face;
uint8_t trail[ROWS][COLS];
uint8_t deadend[ROWS][COLS];
uint8_t pot_field[ROWS][COLS];
uint8_t decision;
uint8_t way_left, way_front, way_right;
void log(char *text)
{
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}
// Rotate 90 degrees counterclockwise
void rotate_left()
{
    // Reset the tick counters for motor rotation

    // cli(); // Disable global interrupts
    // ticks_l = 0;
    // ticks_r = 0;
    // // sei(); // Enable global interrupts
    // // Rotate the micromouse 90 degrees (20 ticks per 90 degrees)
    // while ((ticks_r < (Full_cycle / 4)) || (ticks_l < (Full_cycle / 4)))
    // {
    //     // Use the PID controller to control the left wheel's motion
    //     PID_Controller(LEFT);
    // }

    // Update the facing direction based on the rotation
    switch (face)
    {
    case north:
        face = west;
        break;
    case west:
        face = south;
        break;
    case south:
        face = east;
        break;
    case east:
        face = north;
        break;
    }
}

// Rotate 90 degrees clockwise
void rotate_right()
{
    // Reset the tick counters for motor rotation
    // cli(); // Disable global interrupts
    // ticks_l = 0;
    // ticks_r = 0;
    // // sei(); // Enable global interrupts
    // // Rotate the micromouse 90 degrees (20 ticks per 90 degrees)
    // while ((ticks_r < (Full_cycle / 4)) || (ticks_l < (Full_cycle / 4)))
    // {
    //     // Use the PID controller to control the right wheel's motion
    //     PID_Controller(RIGHT);
    // }

    // Update the facing direction based on the rotation
    switch (face)
    {
    case north:
        face = east;
        break;
    case east:
        face = south;
        break;
    case south:
        face = west;
        break;
    case west:
        face = north;
        break;
    }
}

void forward()
{
    // Increment the trail marker for the current cell
    if (isNavigating)
    {
        if (Navegating_times == DONE_Navigating)
        {
            trail[x_cor][y_cor] == 0;
            log("zero");
            // EEPROM_write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
        }
        else
        {
            if (!(x_cor == 0 && y_cor == 0))
            {
                ++trail[x_cor][y_cor];
                API_setText(x_cor, y_cor, (trail[x_cor][y_cor]));

                // EEPROM_write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
            }
            else
            {
                trail[x_cor][y_cor] = 255;
                // EEPROM_write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
            }
        }
    }

    // Update the micromouse's coordinates based on the facing direction

    if (face == north)
        y_cor++;
    else if (face == east)
        x_cor++;
    else if (face == south)
        y_cor--;
    else if (face == west)
        x_cor--;
}

int getInteger(char *command)
{
    printf("%s\n", command);
    fflush(stdout);
    char response[BUFFER_SIZE];
    fgets(response, BUFFER_SIZE, stdin);
    int value = atoi(response);
    return value;
}
int getBoolean(char *command)
{
    printf("%s\n", command);
    fflush(stdout);
    char response[BUFFER_SIZE];
    fgets(response, BUFFER_SIZE, stdin);
    int value = (strcmp(response, "true\n") == 0);
    return value;
}
int getAck(char *command)
{
    printf("%s\n", command);
    fflush(stdout);
    char response[BUFFER_SIZE];
    fgets(response, BUFFER_SIZE, stdin);
    int success = (strcmp(response, "ack\n") == 0);
    return success;
}
int API_mazeWidth()
{
    return getInteger("mazeWidth");
}
int API_mazeHeight()
{
    return getInteger("mazeHeight");
}
int API_wallFront()
{
    return getBoolean("wallFront");
}
int API_wallRight()
{
    return getBoolean("wallRight");
}
int API_wallLeft()
{
    return getBoolean("wallLeft");
}
int API_moveForward()
{
    forward();
    // API_setColor(x_cor, y_cor, 'B');
    return getAck("moveForward");
}
void API_turnRight()
{
    rotate_right();
    getAck("turnRight");
}

void API_turnLeft()
{
    rotate_left();
    getAck("turnLeft");
}

void API_setWall(int x, int y, char direction)
{
    printf("setWall %d %d %c\n", x, y, direction);
    fflush(stdout);
}

void API_clearWall(int x, int y, char direction)
{
    printf("clearWall %d %d %c\n", x, y, direction);
    fflush(stdout);
}

void API_setColor(int x, int y, char color)
{
    printf("setColor %d %d %c\n", x, y, color);
    fflush(stdout);
}

void API_clearColor(int x, int y)
{
    printf("clearColor %d %d\n", x, y);
    fflush(stdout);
}

void API_clearAllColor()
{
    printf("clearAllColor\n");
    fflush(stdout);
}

void API_setText(int x, int y, uint8_t text)
{
    printf("setText %d %d %d\n", x, y, text);
    fflush(stdout);
}

void API_clearText(int x, int y)
{
    printf("clearText %d %d\n", x, y);
    fflush(stdout);
}

void API_clearAllText()
{
    printf("clearAllText\n");
    fflush(stdout);
}

int API_wasReset()
{
    return getBoolean("wasReset");
}

void API_ackReset()
{
    getAck("ackReset");
}
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

// // Function to determine wall types and update maze values
// void DetermineWallTypes(uint8_t x, uint8_t y)
// {
//     // Determine wall types and update maze values based on the current facing direction
//     if (face == north)
//     {
//         if (way_left)
//             setVerticalWall(x, y);
//         if (way_front)
//             setHorizontalWall(x, y + 1);
//         if (way_right)
//             setVerticalWall(x + 1, y);
//     }
//     else if (face == east)
//     {
//         if (way_left)
//             setHorizontalWall(x, y + 1);
//         if (way_front)
//             setVerticalWall(x + 1, y);
//         if (way_right)
//             setHorizontalWall(x, y);
//     }
//     else if (face == south)
//     {
//         if (way_left)
//             setVerticalWall(x + 1, y);
//         if (way_front)
//             setHorizontalWall(x, y);
//         if (way_right)
//             setVerticalWall(x, y);
//     }
//     else if (face == west)
//     {
//         if (way_left)
//             setHorizontalWall(x, y);
//         if (way_front)
//             setVerticalWall(x, y);
//         if (way_right)
//             setHorizontalWall(x, y + 1);
//     }
// }

// void Go_Back()
// {
//     half_turn();
//     while (x_cor != 0 && y_cor != 0)
//     {
//         wallcheck();
//         Find_Lowest_Path();
//     }
// }
// // Check if the micromouse has reached the finish cell
// void finishcheck()
// {
//     if (x_cor == x_fin && y_cor == y_fin)
//     {
//         // Perform actions when the finish cell is reached
//         // Example: Serial.println("FINISH");
//         if (Navegating_times < DONE_Navigating)
//         {
//             Navegating_times++;
//             // delay_ms(30 * 1000);
//             x_cor = 0;
//             y_cor = 0;
//             face = north;
//         }
//     }
// }

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
// void half_turn()
// {
//     // Reset the tick counters for motor rotation
//     // GIE_disable();
//     // ticks_l = 0;
//     // ticks_r = 0;
//     // GIE_enable();
//     /////////////////////////////////////////// ARDUINO////////////////////////////////////////////////////
//     // cli(); // Disable global interrupts
//     ticks_l = 0;
//     ticks_r = 0;
//     // sei(); // Enable global interrupts
//     // Rotate 180 degrees clockwise (40 ticks per 180 degrees)
//     // while ((ticks_r < (Full_cycle / 2)) || (ticks_l < (Full_cycle / 2)))
//     // {
//     //     // Use the PID controller to control the wheels for a half-turn
//     //     PID_Controller(HALF_TURN);
//     // }
//     // Update the facing direction after the half-turn
//     if (face == north)
//         face = south;
//     else if (face == west)
//         face = east;
//     else if (face == south)
//         face = north;
//     else if (face == east)
//         face = west;
// }
// Closes a cell after reaching a dead end.
void Close_Cell_After_Deadend()
{
    if (face == north)
        API_setWall(x_cor, y_cor, face);
    else if (face == south)
        API_setWall(x_cor, y_cor, face);
    else if (face == east)
        API_setWall(x_cor, y_cor, face);
    else if (face == west)
        API_setWall(x_cor, y_cor, face);
}

// Handle dead ends and find a way out
uint8_t FirtPath = 0;
void firstPath()
{
    wallcheck();
    decision = way_left + way_right + way_front;
    while (decision == 2)
    {
        log("DEAD");
        API_setColor(x_cor, y_cor, 'Y');
        deadend[x_cor][y_cor] = 1;
        trail[x_cor][y_cor] = 200;
        // EEPROM_write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);

        oneway();
        Close_Cell_After_Deadend();

        wallcheck();
        decision = way_left + way_right + way_front;
    }
    FirtPath = 1;
}
void dead_end()
{
    // Execute a half-turn to retrace the path
    // Execute a half-turn to retrace the path
    API_turnLeft();
    API_turnLeft();
    wallcheck();
    decision = way_left + way_right + way_front;
    while (decision == 2)
    {
        log("DEAD");
        API_setColor(x_cor, y_cor, 'R');
        deadend[x_cor][y_cor] = 1;
        trail[x_cor][y_cor] = 200;
        // EEPROM_write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);

        oneway();
        Close_Cell_After_Deadend();

        wallcheck();
        decision = way_left + way_right + way_front;
    }
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
            if ((deadend[x_cor][y_cor + 1] == 1) && (deadend[x_cor + 1][y_cor] == 1))
            {
                dead_end();
            }
            else if (deadend[x_cor][y_cor + 1] == 1)
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
            if ((deadend[x_cor][y_cor - 1] == 1) && (deadend[x_cor + 1][y_cor] == 1))
            {
                dead_end();
            }
            else if (deadend[x_cor + 1][y_cor] == 1)
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
            if ((deadend[x_cor][y_cor - 1] == 1) && (deadend[x_cor - 1][y_cor] == 1))
            {
                dead_end();
            }
            else if (deadend[x_cor][y_cor - 1] == 1)
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
            if ((deadend[x_cor][y_cor + 1] == 1) && (deadend[x_cor - 1][y_cor] == 1))
            {
                dead_end();
            }
            else if (deadend[x_cor - 1][y_cor] == 1)
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
            if ((deadend[x_cor - 1][y_cor] == 1) && (deadend[x_cor + 1][y_cor] == 1))
            {
                dead_end();
            }
            else if (deadend[x_cor - 1][y_cor] == 1)
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
            if ((deadend[x_cor][y_cor + 1] == 1) && (deadend[x_cor][y_cor - 1] == 1))
            {
                dead_end();
            }
            else if (deadend[x_cor][y_cor + 1] == 1)
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
            if ((deadend[x_cor - 1][y_cor] == 1) && (deadend[x_cor + 1][y_cor] == 1))
            {
                dead_end();
            }
            else if (deadend[x_cor + 1][y_cor] == 1)
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
            if ((deadend[x_cor][y_cor + 1] == 1) && (deadend[x_cor][y_cor - 1] == 1))
            {
                dead_end();
            }
            else if (deadend[x_cor][y_cor - 1] == 1)
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
            if ((deadend[x_cor][y_cor + 1] == 1) && (deadend[x_cor - 1][y_cor] == 1))
            {
                dead_end();
            }
            else if (deadend[x_cor - 1][y_cor] == 1)
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
            if ((deadend[x_cor][y_cor + 1] == 1) && (deadend[x_cor + 1][y_cor] == 1))
            {
                dead_end();
            }
            else if (deadend[x_cor][y_cor + 1] == 1)
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
            if ((deadend[x_cor][y_cor - 1] == 1) && (deadend[x_cor + 1][y_cor] == 1))
            {
                dead_end();
            }
            else if (deadend[x_cor + 1][y_cor] == 1)
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
            if ((deadend[x_cor][y_cor - 1] == 1) && (deadend[x_cor - 1][y_cor] == 1))
            {
                dead_end();
            }
            else if (deadend[x_cor][y_cor - 1] == 1)
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

void Maze_Navigate()
{
    // This function is responsible for navigating the maze.
    // Check if the maze solving is finished.
    // finishcheck();
    //   if (Navegating_times == DONE_Navigating)
    //     Go_Back();
    if (!FirtPath)
        firstPath();
    else
    {
        // Check for walls and update maze information.
        wallcheck();
        // Make navigation decisions based on the current state.
        decisions();
    } // Perform the next move based on the decisions.
    // Oop();
    // Delay for a second (500 milliseconds).
    // delay_ms(500);
}
int main(int argc, char *argv[])
{
    while (!(x_cor == x_fin && y_cor == y_fin))
    {
        Maze_Navigate();
    }
    return 0;
}
