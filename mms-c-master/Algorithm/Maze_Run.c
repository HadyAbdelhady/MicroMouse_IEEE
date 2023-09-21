#include "Algorithm/Inc/Maze_Run.h"
// Include the header file "Maze_Run.h" from the "Algorithm/Inc" directory.
// This header likely contains declarations and definitions needed for this code.

uint8_t PATH[ROWS][COLS];
// Declare a 2D array named 'PATH' of type uint8_t to represent the maze path.

uint8_t Blocking_V_Walls[ROWS + 1][COLS] = {0}; // Vertical walls Readings
uint8_t Blocking_H_Walls[ROWS][COLS + 1] = {0}; // Horizontal walls Readings
// Declare two 2D arrays 'Blocking_V_Walls' and 'Blocking_H_Walls' to represent vertical and horizontal walls in the maze.
// Adjusted the dimensions by adding 1 to accommodate the maze structure.
// Initialized them with zeros.

uint8_t isNavigating = -1;
// Declare a variable 'isNavigating' of type uint8_t and initialize it to -1.

// Function to read the maze data from EEPROM and populate the 'maze' array
void readMazeFromEEPROM()
{
    // Read data from EEPROM and populate the 'maze' array
    // Adjust this part to match your EEPROM data format
    for (int x = 0; x < ROWS; x++)
    {
        for (int y = 0; y < COLS; y++)
            PATH[x][y] = EEPROM_read(x * 10 + y);
    }
    // Read data from EEPROM and store it in the 'PATH' array based on the EEPROM data format.

    for (int x = 0; x <= ROWS; x++)
    {
        for (int y = 0; y < COLS; y++)
        {
            Blocking_V_Walls[x][y] = EEPROM_read(v_Walls_BASE + (x * ROWS + y));
        }
    }
    // Read data from EEPROM and store it in the 'Blocking_V_Walls' array based on the EEPROM data format.

    for (int x = 0; x < ROWS; x++)
    {
        for (int y = 0; y <= COLS; y++)
        {
            Blocking_H_Walls[x][y] = EEPROM_read(h_Walls_BASE + (x * ROWS + y));
        }
    }
    // Read data from EEPROM and store it in the 'Blocking_H_Walls' array based on the EEPROM data format.
}

// Function to set a vertical wall at a specific location
void setVerticalWall(uint8_t x, uint8_t y)
{
    if (x >= 0 && x <= ROWS && y >= 0 && y < COLS)
    {
        EEPROM_write(v_Walls_BASE + (x * ROWS + y), 1);
    }
}
// Set a vertical wall at the specified location (x, y) in EEPROM if the coordinates are within bounds.

// Function to set a horizontal wall at a specific location
void setHorizontalWall(uint8_t x, uint8_t y)
{
    if (x >= 0 && x < ROWS && y >= 0 && y <= COLS)
    {

        // // Write a value (e.g., 1) to the calculated EEPROM address
        // EEPROM.write(h_Walls_BASE + (x * ROWS + y), 1);

        // // Commit the changes to EEPROM
        // EEPROM.commit();

        // EEPROM_write(h_Walls_BASE + (x * ROWS + y), 1);
    }
}
// Set a horizontal wall at the specified location (x, y) in EEPROM if the coordinates are within bounds.
void Solve_2_ways()
{
    if (way_front)
    {
        if (face == north)
        {
            if (trail[x_cor + 1][y_cor] == trail[x_cor - 1][y_cor])
            {
                if (((x_cor + 1) + (y_cor)) < ((x_cor - 1) + (y_cor)))
                {
                    rotate_right();
                    forward();
                }
                else if (((x_cor + 1) + (y_cor)) > ((x_cor - 1) + (y_cor)))
                {
                    rotate_left();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]) < (Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]))
                    {
                        rotate_right();
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]) > (Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]))
                    {
                        rotate_left();
                        forward();
                    }
                }
            }
            else if (trail[x_cor + 1][y_cor] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
            {
                rotate_right();
                forward();
            }
            else if (trail[x_cor + 1][y_cor] > trail[x_cor - 1][y_cor]) // moved in the leading cell before?
            {
                rotate_left();
                forward();
            }
        }
        else if (face == south)
        {
            if (trail[x_cor + 1][y_cor] == trail[x_cor - 1][y_cor])
            {
                if (((x_cor + 1) + (y_cor)) < ((x_cor - 1) + (y_cor)))
                {
                    rotate_left();
                    forward();
                }
                else if (((x_cor + 1) + (y_cor)) > ((x_cor - 1) + (y_cor)))
                {
                    rotate_right();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]) < (Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]))
                    {
                        rotate_left();
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]) > (Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]))
                    {
                        rotate_right();
                        forward();
                    }
                }
            }
            else if (trail[x_cor + 1][y_cor] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
            {
                rotate_left();
                forward();
            }
            else if (trail[x_cor + 1][y_cor] > trail[x_cor - 1][y_cor]) // moved in the leading cell before?
            {
                rotate_right();
                forward();
            }
        }
        else if (face == east)
        {
            if (trail[x_cor][y_cor + 1] == trail[x_cor][y_cor - 1])
            {
                if (((x_cor) + (y_cor + 1)) < ((x_cor) + (y_cor - 1)))
                {
                    rotate_left();
                    forward();
                }
                if (((x_cor) + (y_cor + 1)) > ((x_cor) + (y_cor - 1)))
                {
                    rotate_right();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 1]) < (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        rotate_left();
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 1]) > (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        rotate_right();
                        forward();
                    }
                }
            }
            else if (trail[x_cor][y_cor + 1] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
            {
                rotate_left();
                forward();
            }
            else if (trail[x_cor][y_cor + 1] > trail[x_cor][y_cor - 1]) // moved in the leading cell before?
            {
                rotate_right();
                forward();
            }
        }
        else if (face == west)
        {
            if (trail[x_cor][y_cor + 1] == trail[x_cor][y_cor - 1])
            {
                if (((x_cor) + (y_cor + 1)) < ((x_cor) + (y_cor - 1)))
                {
                    rotate_right();
                    forward();
                }
                if (((x_cor) + (y_cor + 1)) > ((x_cor) + (y_cor - 1)))
                {
                    rotate_left();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 1]) < (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        rotate_right();
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 1]) > (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        rotate_left();
                        forward();
                    }
                }
            }
            else if (trail[x_cor][y_cor + 1] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
            {
                rotate_right();
                forward();
            }
            else if (trail[x_cor][y_cor + 1] > trail[x_cor][y_cor - 1]) // moved in the leading cell before?
            {
                rotate_left();
                forward();
            }
        }
    }
    else if (way_left)
    {
        if (face == north)
        {
            if (trail[x_cor][y_cor + 1] == trail[x_cor + 1][y_cor])
            {
                if (((x_cor) + (y_cor + 1)) < ((x_cor + 1) + (y_cor)))
                {
                    forward();
                }
                if (((x_cor) + (y_cor + 1)) > ((x_cor + 1) + (y_cor)))
                {
                    rotate_right();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) < (Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]))
                    {
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) > (Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]))
                    {
                        rotate_right();
                        forward();
                    }
                }
            }
            else if (trail[x_cor][y_cor + 1] < trail[x_cor + 1][y_cor]) // moved in the right cell before?
            {
                forward();
            }
            else if (trail[x_cor][y_cor + 1] > trail[x_cor + 1][y_cor]) // moved in the leading cell before?
            {
                rotate_right();
                forward();
            }
        }
        else if (face == south)
        {
            if (trail[x_cor - 1][y_cor] == trail[x_cor][y_cor - 1])
            {
                if (((x_cor - 1) + (y_cor)) < ((x_cor) + (y_cor - 1)))
                {
                    forward();
                }
                if (((x_cor - 1) + (y_cor)) > ((x_cor) + (y_cor - 1)))
                {
                    rotate_right();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]) < (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        rotate_right();
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]) > (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        forward();
                    }
                }
            }
            else if (trail[x_cor][y_cor - 1] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
            {
                forward();
            }
            else if (trail[x_cor - 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the leading cell before?
            {
                rotate_right();
                forward();
            }
        }
        else if (face == east)
        {
            if (trail[x_cor + 1][y_cor] == trail[x_cor][y_cor - 1])
            {
                if (((x_cor + 1) + (y_cor)) < ((x_cor) + (y_cor - 1)))
                {
                    forward();
                }
                if (((x_cor + 1) + (y_cor)) > ((x_cor) + (y_cor - 1)))
                {
                    rotate_right();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]) < (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]) > (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        rotate_right();
                        forward();
                    }
                }
            }
            else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
            {
                forward();
            }
            else if (trail[x_cor + 1][y_cor] > trail[x_cor][y_cor - 1]) // moved in the leading cell before?
            {
                rotate_right();
                forward();
            }
        }
        else if (face == west)
        {
            if (trail[x_cor][y_cor + 1] == trail[x_cor - 1][y_cor])
            {
                if (((x_cor) + (y_cor + 1)) < ((x_cor - 1) + (y_cor)))
                {
                    rotate_right();
                    forward();
                }
                if (((x_cor) + (y_cor + 1)) > ((x_cor - 1) + (y_cor)))
                {
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) < (Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]))
                    {
                        rotate_right();
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) > (Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]))
                    {
                        forward();
                    }
                }
            }
            else if (trail[x_cor][y_cor + 1] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
            {
                rotate_right();
                forward();
            }
            else if (trail[x_cor][y_cor + 1] > trail[x_cor - 1][y_cor]) // moved in the leading cell before?
            {
                forward();
            }
        }
    }
    else if (way_right)
    {
        if (face == north)
        {
            if (trail[x_cor][y_cor + 1] == trail[x_cor - 1][y_cor])
            {
                if (((x_cor) + (y_cor + 1)) < ((x_cor - 1) + (y_cor)))
                {
                    forward();
                }
                if (((x_cor) + (y_cor + 1)) > ((x_cor - 1) + (y_cor)))
                {
                    rotate_left();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) < (Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]))
                    {
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) > (Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]))
                    {
                        rotate_left();
                        forward();
                    }
                }
            }
            else if (trail[x_cor][y_cor + 1] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
            {
                forward();
            }
            else if (trail[x_cor][y_cor + 1] > trail[x_cor - 1][y_cor]) // moved in the leading cell before?
            {
                rotate_left();
                forward();
            }
        }
        else if (face == south)
        {
            if (trail[x_cor + 1][y_cor] == trail[x_cor][y_cor - 1])
            {
                if (((x_cor + 1) + (y_cor)) < ((x_cor) + (y_cor - 1)))
                {
                    rotate_left();
                    forward();
                }
                if (((x_cor + 1) + (y_cor)) > ((x_cor) + (y_cor - 1)))
                {
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]) < (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        rotate_left();
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]) > (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        forward();
                    }
                }
            }
            else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
            {
                rotate_left();
                forward();
            }
            else if (trail[x_cor + 1][y_cor] > trail[x_cor][y_cor - 1]) // moved in the leading cell before?
            {
                forward();
            }
        }
        else if (face == east)
        {
            if (trail[x_cor + 1][y_cor] == trail[x_cor][y_cor + 1])
            {
                if (((x_cor + 1) + (y_cor)) < ((x_cor) + (y_cor + 1)))
                {
                    forward();
                }
                if (((x_cor + 1) + (y_cor)) > ((x_cor) + (y_cor + 1)))
                {
                    rotate_left();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) < (Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]))
                    {
                        rotate_left();
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) > (Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]))
                    {
                        forward();
                    }
                }
            }
            else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor + 1]) // moved in the right cell before?
            {
                forward();
            }
            else if (trail[x_cor + 1][y_cor] > trail[x_cor][y_cor + 1]) // moved in the leading cell before?
            {
                rotate_left();
                forward();
            }
        }
        else if (face == west)
        {
            if (trail[x_cor - 1][y_cor] == trail[x_cor][y_cor - 1])
            {
                if (((x_cor - 1) + (y_cor)) < ((x_cor) + (y_cor - 1)))
                {
                    forward();
                }
                if (((x_cor - 1) + (y_cor)) > ((x_cor) + (y_cor - 1)))
                {
                    rotate_left();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]) < (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]) > (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        rotate_left();
                        forward();
                    }
                    else
                    {
                        // north west quarter
                        if (x_cor <= x_fin && y_cor >= y_fin)
                        {
                            if (pot_south < pot_west)
                            {
                                rotate_left();
                                forward();
                            }
                        }
                        else
                        {
                            forward();
                        }
                    }
                }
            }
            else if (trail[x_cor][y_cor - 1] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
            {
                rotate_left();
                forward();
            }
            else if (trail[x_cor - 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the leading cell before?
            {
                forward();
            }
        }
    }
}
uint8_t max_value(uint8_t n, uint8_t s, uint8_t e, uint8_t w)
{

    if (decision == 1)
    {
    }
    // ToDo
    else if (decision == 0)
    {
        if ((n > s) && (n > e) && (n > w))
            return n;
        else if ((s > n) && (s > e) && (s > w))
            return s;
        else if ((w > n) && (w > e) && (w > s))
            return w;
        else if ((e > n) && (e > w) && (e > s))
            return e;
    }
    else if (e == n || e == s || e == w)
    {
        /* code */
    }

    return 0;
}
// Determine and return the minimum value among four input values (north, south, east, west).

uint8_t getLowestPotential(uint8_t x, uint8_t y)
{
    if (face == north)
        pot_south = Maze_Border; // dont check the cell behind
    else if (face == east)
        pot_west = Maze_Border; // dont check the cell behind
    else if (face == south)
        pot_north = Maze_Border; // dont check the cell behind
    else if (face == west)
        pot_east = Maze_Border; // dont check the cell behind

    // Initialize the lowest potential value and direction
    uint8_t lowestIndex = -1;
    // uint8_t min_pot = minimum_value(pot_north, pot_south, pot_east, pot_west);

    // Compare potential values and update the lowestIndex and lowestPot
    // if (pot_north == min_pot && !Blocking_H_Walls[x][y + 1])
    // {
    //     lowestIndex = north;
    // }
    // if (pot_east == min_pot && !Blocking_V_Walls[x + 1][y])
    // {
    //     lowestIndex = east;
    // }
    // if (pot_south == min_pot && !Blocking_H_Walls[x][y])
    // {
    //     lowestIndex = south;
    // }
    // if (pot_west == min_pot && !Blocking_V_Walls[x][y])
    // {
    //     lowestIndex = west;
    // }

    // return lowestIndex;
}
void Solve_3_ways()
{

    trail_north = (y_cor + 1 < ROWS) ? trail[x_cor][y_cor + 1] : 0;
    trail_east = (x_cor + 1 < COLS) ? trail[x_cor + 1][y_cor] : 0;
    trail_south = (y_cor - 1 >= 0) ? trail[x_cor][y_cor - 1] : 0;
    trail_west = (x_cor - 1 >= 0) ? trail[x_cor - 1][y_cor] : 0;
    uint8_t Quarter = 0;
    if (x_cor <= x_fin && y_cor >= y_fin)
        Quarter = north_west;
    else if (x_cor >= x_fin && y_cor >= y_fin)
        Quarter = north_east;
    else if (x_cor <= x_fin && y_cor <= y_fin)
        Quarter = south_west;
    else if (x_cor >= x_fin && y_cor <= y_fin)
        Quarter = south_east;

    if (face == north)
    {
        if (trail_north > trail_west && trail_north > trail_east)
        {
            forward();
        }
        else if (trail_west > trail_north && trail_west > trail_east)
        {
            rotate_left();
            forward();
        }
        if (trail_east > trail_west && trail_east > trail_north) // moved in the right cell before?
        {
            rotate_right();
            forward();
        }
        else if (trail_north == trail_west && trail_north == trail_east) // moved in the leading cell before?
        {
            // always your left will be less than your front and right
            //--> try for example the poin 4,6
            rotate_left();
            forward();
        }
        else if (trail_east == trail_north) // sum of X & Y will always be equal
        {
            if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) < (Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]))
            {
                forward();
            }
            else if ((Blocking_V_Walls[x_cor][y_cor + 2] + Blocking_H_Walls[x_cor][y_cor + 1]) > (Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]))
            {
                rotate_right();
                forward();
            }
            else
            {
                // needs to be seen as it wount matter anyway
                //  in the potential values your right will always be greater than your north
                //   north west quarter
                if (Quarter == north_east)
                {
                    rotate_right();
                    forward();
                }

                else if (Quarter == north_west)
                {
                    rotate_left();
                    forward();
                }
                else // weather you are east or west you will always go forward
                    forward();
            }
        }
        else if (trail_east == trail_west)
        {
            if ((Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]) < (Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]))
            {
                rotate_right();
                forward();
            }
            else if ((Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]) > (Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]))
            {
                rotate_left();
                forward();
            }
            else
            {

                rotate_left();
                forward();
            }
        }
        else if (trail_north == trail_west)
        {
            if ((Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]) < (Blocking_V_Walls[x_cor + 1][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]))
            {
                rotate_left();
                forward();
            }
            else if ((Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]) > (Blocking_V_Walls[x_cor + 1][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]))
            {
                forward();
            }
            else
            {

                rotate_left();
                forward();
            }
        }
    }
    else if (face == south)
    {
        if (trail[x_cor + 1][y_cor] == trail[x_cor - 1][y_cor])
        {
            if (((x_cor + 1) + (y_cor)) < ((x_cor - 1) + (y_cor)))
            {
                rotate_left();
                forward();
            }
            else if (((x_cor + 1) + (y_cor)) > ((x_cor - 1) + (y_cor)))
            {
                rotate_right();
                forward();
            }
            else
            {

                if ((Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]) < (Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]))
                {
                    rotate_left();
                    forward();
                }
                else if ((Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]) > (Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]))
                {
                    rotate_right();
                    forward();
                }
            }
        }
        else if (trail[x_cor + 1][y_cor] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
        {
            rotate_left();
            forward();
        }
        else if (trail[x_cor + 1][y_cor] > trail[x_cor - 1][y_cor]) // moved in the leading cell before?
        {
            rotate_right();
            forward();
        }
    }
    else if (face == east)
    {
        if (trail[x_cor][y_cor + 1] == trail[x_cor][y_cor - 1])
        {
            if (((x_cor) + (y_cor + 1)) < ((x_cor) + (y_cor - 1)))
            {
                rotate_left();
                forward();
            }
            if (((x_cor) + (y_cor + 1)) > ((x_cor) + (y_cor - 1)))
            {
                rotate_right();
                forward();
            }
            else
            {
                if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 1]) < (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                {
                    rotate_left();
                    forward();
                }
                else if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 1]) > (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                {
                    rotate_right();
                    forward();
                }
            }
        }
        else if (trail[x_cor][y_cor + 1] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
        {
            rotate_left();
            forward();
        }
        else if (trail[x_cor][y_cor + 1] > trail[x_cor][y_cor - 1]) // moved in the leading cell before?
        {
            rotate_right();
            forward();
        }
    }
    else if (face == west)
    {
        if (trail[x_cor][y_cor + 1] == trail[x_cor][y_cor - 1])
        {
            if (((x_cor) + (y_cor + 1)) < ((x_cor) + (y_cor - 1)))
            {
                rotate_right();
                forward();
            }
            if (((x_cor) + (y_cor + 1)) > ((x_cor) + (y_cor - 1)))
            {
                rotate_left();
                forward();
            }
            else
            {
                if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 1]) < (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                {
                    rotate_right();
                    forward();
                }
                else if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 1]) > (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                {
                    rotate_left();
                    forward();
                }
            }
        }
        else if (trail[x_cor][y_cor + 1] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
        {
            rotate_right();
            forward();
        }
        else if (trail[x_cor][y_cor + 1] > trail[x_cor][y_cor - 1]) // moved in the leading cell before?
        {
            rotate_left();
            forward();
        }
    }
    else if (way_left)
    {
        if (face == north)
        {
            if (trail[x_cor][y_cor + 1] == trail[x_cor + 1][y_cor])
            {
                if (((x_cor) + (y_cor + 1)) < ((x_cor + 1) + (y_cor)))
                {
                    forward();
                }
                if (((x_cor) + (y_cor + 1)) > ((x_cor + 1) + (y_cor)))
                {
                    rotate_right();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) < (Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]))
                    {
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) > (Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]))
                    {
                        rotate_right();
                        forward();
                    }
                }
            }
            else if (trail[x_cor][y_cor + 1] < trail[x_cor + 1][y_cor]) // moved in the right cell before?
            {
                forward();
            }
            else if (trail[x_cor][y_cor + 1] > trail[x_cor + 1][y_cor]) // moved in the leading cell before?
            {
                rotate_right();
                forward();
            }
        }
        else if (face == south)
        {
            if (trail[x_cor - 1][y_cor] == trail[x_cor][y_cor - 1])
            {
                if (((x_cor - 1) + (y_cor)) < ((x_cor) + (y_cor - 1)))
                {
                    forward();
                }
                if (((x_cor - 1) + (y_cor)) > ((x_cor) + (y_cor - 1)))
                {
                    rotate_right();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]) < (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        rotate_right();
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]) > (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        forward();
                    }
                }
            }
            else if (trail[x_cor][y_cor - 1] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
            {
                forward();
            }
            else if (trail[x_cor - 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the leading cell before?
            {
                rotate_right();
                forward();
            }
        }
        else if (face == east)
        {
            if (trail[x_cor + 1][y_cor] == trail[x_cor][y_cor - 1])
            {
                if (((x_cor + 1) + (y_cor)) < ((x_cor) + (y_cor - 1)))
                {
                    forward();
                }
                if (((x_cor + 1) + (y_cor)) > ((x_cor) + (y_cor - 1)))
                {
                    rotate_right();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]) < (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]) > (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        rotate_right();
                        forward();
                    }
                }
            }
            else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
            {
                forward();
            }
            else if (trail[x_cor + 1][y_cor] > trail[x_cor][y_cor - 1]) // moved in the leading cell before?
            {
                rotate_right();
                forward();
            }
        }
        else if (face == west)
        {
            if (trail[x_cor][y_cor + 1] == trail[x_cor - 1][y_cor])
            {
                if (((x_cor) + (y_cor + 1)) < ((x_cor - 1) + (y_cor)))
                {
                    rotate_right();
                    forward();
                }
                if (((x_cor) + (y_cor + 1)) > ((x_cor - 1) + (y_cor)))
                {
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) < (Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]))
                    {
                        rotate_right();
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) > (Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]))
                    {
                        forward();
                    }
                }
            }
            else if (trail[x_cor][y_cor + 1] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
            {
                rotate_right();
                forward();
            }
            else if (trail[x_cor][y_cor + 1] > trail[x_cor - 1][y_cor]) // moved in the leading cell before?
            {
                forward();
            }
        }
    }
    else if (way_right)
    {
        if (face == north)
        {
            if (trail[x_cor][y_cor + 1] == trail[x_cor - 1][y_cor])
            {
                if (((x_cor) + (y_cor + 1)) < ((x_cor - 1) + (y_cor)))
                {
                    forward();
                }
                if (((x_cor) + (y_cor + 1)) > ((x_cor - 1) + (y_cor)))
                {
                    rotate_left();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) < (Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]))
                    {
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) > (Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]))
                    {
                        rotate_left();
                        forward();
                    }
                }
            }
            else if (trail[x_cor][y_cor + 1] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
            {
                forward();
            }
            else if (trail[x_cor][y_cor + 1] > trail[x_cor - 1][y_cor]) // moved in the leading cell before?
            {
                rotate_left();
                forward();
            }
        }
        else if (face == south)
        {
            if (trail[x_cor + 1][y_cor] == trail[x_cor][y_cor - 1])
            {
                if (((x_cor + 1) + (y_cor)) < ((x_cor) + (y_cor - 1)))
                {
                    rotate_left();
                    forward();
                }
                if (((x_cor + 1) + (y_cor)) > ((x_cor) + (y_cor - 1)))
                {
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]) < (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        rotate_left();
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]) > (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        forward();
                    }
                }
            }
            else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
            {
                rotate_left();
                forward();
            }
            else if (trail[x_cor + 1][y_cor] > trail[x_cor][y_cor - 1]) // moved in the leading cell before?
            {
                forward();
            }
        }
        else if (face == east)
        {
            if (trail[x_cor + 1][y_cor] == trail[x_cor][y_cor + 1])
            {
                if (((x_cor + 1) + (y_cor)) < ((x_cor) + (y_cor + 1)))
                {
                    forward();
                }
                if (((x_cor + 1) + (y_cor)) > ((x_cor) + (y_cor + 1)))
                {
                    rotate_left();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) < (Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]))
                    {
                        rotate_left();
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor][y_cor + 1] + Blocking_H_Walls[x_cor][y_cor + 2]) > (Blocking_V_Walls[x_cor + 2][y_cor] + Blocking_H_Walls[x_cor + 1][y_cor]))
                    {
                        forward();
                    }
                }
            }
            else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor + 1]) // moved in the right cell before?
            {
                forward();
            }
            else if (trail[x_cor + 1][y_cor] > trail[x_cor][y_cor + 1]) // moved in the leading cell before?
            {
                rotate_left();
                forward();
            }
        }
        else if (face == west)
        {
            if (trail[x_cor - 1][y_cor] == trail[x_cor][y_cor - 1])
            {
                if (((x_cor - 1) + (y_cor)) < ((x_cor) + (y_cor - 1)))
                {
                    forward();
                }
                if (((x_cor - 1) + (y_cor)) > ((x_cor) + (y_cor - 1)))
                {
                    rotate_left();
                    forward();
                }
                else
                {
                    if ((Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]) < (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        forward();
                    }
                    else if ((Blocking_V_Walls[x_cor - 1][y_cor] + Blocking_H_Walls[x_cor - 1][y_cor]) > (Blocking_V_Walls[x_cor][y_cor - 1] + Blocking_H_Walls[x_cor][y_cor - 1]))
                    {
                        rotate_left();
                        forward();
                    }
                    else
                    {
                        // north west quarter
                        if (x_cor <= x_fin && y_cor >= y_fin)
                        {
                            if (pot_south < pot_west)
                            {
                                rotate_left();
                                forward();
                            }
                        }
                        else
                        {
                            forward();
                        }
                    }
                }
            }
            else if (trail[x_cor][y_cor - 1] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
            {
                rotate_left();
                forward();
            }
            else if (trail[x_cor - 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the leading cell before?
            {
                forward();
            }
        }
    }
}

// Calculate and return the direction (north, south, east, west) with the lowest potential.
void Find_Lowest_Path() // wont leave this function until x=0 y=0
{
    // Calculate potential field values for neighboring cells based on the current face
    // Access the array elements if within bounds

    // to be checked ----> change 0 to some thing
    trail_north = (y_cor + 1 < ROWS) ? trail[x_cor][y_cor + 1] : 0;
    trail_east = (x_cor + 1 < COLS) ? trail[x_cor + 1][y_cor] : 0;
    trail_south = (y_cor - 1 >= 0) ? trail[x_cor][y_cor - 1] : 0;
    trail_west = (x_cor - 1 >= 0) ? trail[x_cor - 1][y_cor] : 0;

    if (decision == 2)
    {
        oneway();
    }
    else if (decision == 1)
    {
        Solve_2_ways();
    }
    else if (decision == 3)
    {
        Solve_3_ways();
    }

    // uint8_t min_pot = max_value(pot_north, pot_south, pot_east, pot_west);

    // if (face == north)
    // {
    //     if (lowes_path == north)
    //     {
    //         forward();
    //     }
    //     if (lowes_path == east)
    //     {
    //         rotate_right();
    //         forward();
    //     }
    //     if (lowes_path == west)
    //     {
    //         rotate_left();
    //         forward();
    //     }
    // }
    // else if (face == east)
    // {
    //     if (lowes_path == north)
    //     {
    //         rotate_left();
    //         forward();
    //     }
    //     if (lowes_path == east)
    //     {
    //         forward();
    //     }
    //     if (lowes_path == south)
    //     {
    //         rotate_right();
    //         forward();
    //     }
    // }
    // else if (face == west)
    // {
    //     if (lowes_path == north)
    //     {
    //         rotate_right();
    //         forward();
    //     }
    //     if (lowes_path == west)
    //     {
    //         forward();
    //     }
    //     if (lowes_path == south)
    //     {
    //         rotate_left();
    //         forward();
    //     }
    // }
    // else if (face == south)
    // {
    //     if (lowes_path == west)
    //     {
    //         rotate_right();
    //         forward();
    //     }
    //     if (lowes_path == south)
    //     {
    //         forward();
    //     }
    //     if (lowes_path == east)
    //     {
    //         rotate_left();
    //         forward();
    //     }
    // }
}
// Implement a path-following algorithm based on potential fields and the current facing direction.
