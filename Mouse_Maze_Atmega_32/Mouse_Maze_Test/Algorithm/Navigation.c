#include "Navigate.h"
#include "HAL_Ultrasonic.h"
#include "Move.h"
#include "PID.h"
#include "MCAL_eeprom.h"
#include "Maze_Run.h"
#include "MCAL_interrupt.h"

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
uint8_t FirtPath = 0;


// Check if a wall is present using ultrasonic sensors
uint8_t wall_present(S_GPIO_t *portName, uint8_t trigPin, uint8_t echoPin)
{
	// Measure the distance from the sensor to the wall
	uint32_t distance = GetUltrasonicDistance(portName,trigPin, echoPin);

	// If the distance is greater than or equal to 100mm, assume no wall is present
	if (distance > THRESHOLD_DISTANCE && distance <= LARGEST_DISTANCE)
		return 0; // No wall
	else if (distance <= THRESHOLD_DISTANCE || distance <= SMALLEST_DISTANCE || distance > LARGEST_DISTANCE)
		return 1; // Wall detected
	return 11;
}

// Check for walls using ultrasonic sensors and update variables
void wallcheck()
{
	  Oop();

	// Check if there are walls in different directions
	way_left = wall_present(US_PORT, ltrigger, lecho);
	way_front = wall_present(US_PORT, ftrigger, fecho);
	way_right = wall_present(US_PORT, rtrigger, recho);
	if (isNavigating && Navegating_times != DONE_Navigating)
	{
		DetermineWallTypes(x_cor, y_cor);
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
// Perform a half-turn (rotate 180 degrees clockwise)
void half_turn()
{
	// Reset the tick counters for motor rotation
	GIE_disable();
	ticks_l = 0;
	ticks_r = 0;
	GIE_enable();

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

// Closes a cell after reaching a dead end.
void Close_Cell_After_Deadend()
{
	if (face == north)
		setHorizontalWall(x_cor, y_cor);
	else if (face == south)
		setHorizontalWall(x_cor, y_cor + 1);
	else if (face == east)
		setVerticalWall(x_cor, y_cor);
	else if (face == west)
		setVerticalWall(x_cor + 1, y_cor);
}

// Handle dead ends and find a way out
void dead_end()
{
	// Execute a half-turn to retrace the path
	half_turn();
	wallcheck();
	decision = way_left + way_right + way_front;
	while (decision == 2)
	{
		deadend[x_cor][y_cor] = 1;
		trail[x_cor][y_cor]++;
		EEPROM_write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);

		oneway();
		Close_Cell_After_Deadend();

		wallcheck();
		decision = way_left + way_right + way_front;
	}
}

// ==========================================================================================
//                                         1-WAY
// ==========================================================================================

// Handle one-way paths
void oneway()
{
	if (way_left == 0)
	{
		// If the left path is open, turn left and move forward
		rotate_left();
	}
	else if (way_right == 0)
	{
		// If the right path is open, turn right and move forward
		rotate_right();
	}
	else if (way_front == 0)
	{
		// If the front path is open, move forward
		forward();
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
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor + 1][y_cor] == 1)
      {
        forward();
      }
      // if you didnt move there before
      else if (trail[x_cor + 1][y_cor] == trail[x_cor][y_cor + 1])
      {
        if (pot_north < pot_east)
        {
          forward();
        }
        else if (pot_north >= pot_east)
        {
          rotate_right();
          forward();
        }
      }
      // if not entered the prev if then you must enter 1 of the following 2
      else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor + 1]) // moved in the right cell before?
      {
        //    if not
        rotate_right();
        forward();
      }
      else if (trail[x_cor][y_cor + 1] < trail[x_cor + 1][y_cor]) // moved in the front cell before?
      {
        //    if not
        forward();
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
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor][y_cor - 1] == 1)
      {
        forward();
      }
      else if (trail[x_cor + 1][y_cor] == trail[x_cor][y_cor - 1])
      {
        if (pot_east < pot_south)
        {
          forward();
        }
        else if (pot_east >= pot_south)
        {
          rotate_right();
          forward();
        }
      }
      // if not entered the prev if then you must enter 1 of the following 2
      else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the leading cell before?
      {
        forward();
      }
      else if (trail[x_cor][y_cor - 1] < trail[x_cor + 1][y_cor]) // moved in the benith cell before?
      {
        rotate_right();
        forward();
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
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor - 1][y_cor] == 1)
      {
        forward();
      }
      else if (trail[x_cor - 1][y_cor] == trail[x_cor][y_cor - 1])
      {
        if (pot_south <= pot_east)
        {
          forward();
        }
        else if (pot_south > pot_east)
        {
          rotate_right();
          forward();
        }
      }
      else if (trail[x_cor - 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
      {
        rotate_right();
        forward();
      }
      else if (trail[x_cor][y_cor - 1] < trail[x_cor - 1][y_cor]) // moved in the leading cell before?
      {
        forward();
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
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor][y_cor + 1] == 1)
      {
        forward();
      }
      else if (trail[x_cor - 1][y_cor] == trail[x_cor][y_cor + 1])
      {
        if (pot_north < pot_west)
        {
          rotate_right();
          forward();
        }
        else if (pot_north >= pot_west)
        {
          forward();
        }
      }
      else if (trail[x_cor - 1][y_cor] < trail[x_cor][y_cor + 1]) // moved in the right cell before?
      {
        forward();
      }
      else if (trail[x_cor][y_cor + 1] < trail[x_cor - 1][y_cor]) // moved in the leading cell before?
      {
        rotate_right();
        forward();
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
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor + 1][y_cor] == 1)
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor + 1][y_cor] == trail[x_cor - 1][y_cor])
      {
        if (pot_west < pot_east)
        {
          rotate_left();
          forward();
        }
        else if (pot_west >= pot_east)
        {
          rotate_right();
          forward();
        }
      }
      else if (trail[x_cor - 1][y_cor] < trail[x_cor + 1][y_cor]) // moved in the right cell before?
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor + 1][y_cor] < trail[x_cor - 1][y_cor]) // moved in the leading cell before?
      {
        rotate_right();
        forward();
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
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor][y_cor - 1] == 1)
      {
        rotate_left();
        forward();
      }

      else if (trail[x_cor][y_cor + 1] == trail[x_cor][y_cor - 1])
      {
        if (pot_north < pot_south)
        {
          rotate_left();
          forward();
        }
        else if (pot_north >= pot_south)
        {
          rotate_right();
          forward();
        }
      }
      else if (trail[x_cor][y_cor - 1] < trail[x_cor][y_cor + 1]) // moved in the right cell before?
      {
        rotate_right();
        forward();
      }
      else if (trail[x_cor][y_cor + 1] < trail[x_cor][y_cor - 1]) // moved in the leading cell before?
      {
        rotate_left();
        forward();
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
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor - 1][y_cor] == 1)
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor + 1][y_cor] == trail[x_cor - 1][y_cor])
      {
        if (pot_west < pot_east)
        {
          rotate_right();
          forward();
        }
        else if (pot_west >= pot_east)
        {
          rotate_left();
          forward();
        }
      }
      else if (trail[x_cor - 1][y_cor] < trail[x_cor + 1][y_cor]) // moved in the right cell before?
      {
        rotate_right();
        forward();
      }
      else if (trail[x_cor + 1][y_cor] < trail[x_cor - 1][y_cor]) // moved in the leading cell before?
      {
        rotate_left();
        forward();
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
        rotate_right();
        forward();
      }
      // Deadend: Right
      else if (deadend[x_cor][y_cor + 1] == 1)
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor][y_cor - 1] == trail[x_cor][y_cor + 1])
      {
        if (pot_north <= pot_south)
        {
          rotate_right();
          forward();
        }
        else if (pot_north > pot_south)
        {
          rotate_left();
          forward();
        }
      }
      else if (trail[x_cor][y_cor + 1] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
      {
        rotate_right();
        forward();
      }
      else if (trail[x_cor][y_cor - 1] < trail[x_cor][y_cor + 1]) // moved in the leading cell before?
      {
        rotate_left();
        forward();
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
        forward();
      }
      // Deadend: Front
      else if (deadend[x_cor][y_cor + 1] == 1)
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor][y_cor + 1] == trail[x_cor - 1][y_cor])
      {
        if (pot_west < pot_north)
        {
          rotate_left();
          forward();
        }
        else if (pot_west >= pot_north)
        {
          forward();
        }
      }
      else if (trail[x_cor][y_cor + 1] < trail[x_cor - 1][y_cor]) // moved in the right cell before?
      {
        // rotate_right();
        forward();
      }
      else if (trail[x_cor - 1][y_cor] < trail[x_cor][y_cor + 1]) // moved in the leading cell before?
      {
        rotate_left();
        forward();
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
        forward();
      }
      // Deadend: Front
      else if (deadend[x_cor + 1][y_cor] == 1)
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor][y_cor + 1] == trail[x_cor + 1][y_cor])
      {
        if (pot_north < pot_east)
        {
          rotate_left();
          forward();
        }
        else if (pot_north >= pot_east)
        {
          forward();
        }
      }
      else if (trail[x_cor][y_cor + 1] < trail[x_cor + 1][y_cor]) // moved in the right cell before?
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor + 1]) // moved in the leading cell before?
      {
        forward();
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
        forward();
      }
      // Deadend: Front
      else if (deadend[x_cor][y_cor - 1] == 1)
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor][y_cor - 1] == trail[x_cor + 1][y_cor])
      {
        if (pot_east < pot_south)
        {
          rotate_left();
          forward();
        }
        else if (pot_east >= pot_south)
        {
          forward();
        }
      }
      else if (trail[x_cor + 1][y_cor] < trail[x_cor][y_cor - 1]) // moved in the right cell before?
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor][y_cor - 1] < trail[x_cor + 1][y_cor]) // moved in the leading cell before?
      {
        forward();
      }
    }

    // Facing West
    else if (face == west)
    {
      // Deadend: Left
      // Deadend: Left
      if ((deadend[x_cor][y_cor - 1] == 1) && (deadend[x_cor - 1][y_cor] == 1))
      {
        dead_end();
      }
      else if (deadend[x_cor][y_cor - 1] == 1)
      {
        forward();
      }
      // Deadend: Front
      else if (deadend[x_cor - 1][y_cor] == 1)
      {
        rotate_left();
        forward();
      }
      else if (trail[x_cor][y_cor - 1] == trail[x_cor - 1][y_cor])
      {
        if (pot_south < pot_west)
        {
          rotate_left();
          forward();
        }
        else if (pot_south >= pot_west)
        {
          forward();
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
      rotate_right();
      forward();
    }
    else if ((trail_north < trail_east && trail_north < trail_west))
    {
      forward();
    }
    else if (trail_west < trail_east && trail_west < trail_north)
    {
      rotate_left();
      forward();
    }
    else if (trail_east == trail_north && trail_east == trail_west)
    {
      if (pot_east < pot_north && pot_east < pot_west)
      {
        rotate_right();
        forward();
      }
      else if (pot_north < pot_east && pot_north < pot_west)
      {
        forward();
      }
      else if (pot_west < pot_east && pot_west < pot_north)
      {
        rotate_left();
        forward();
      }
      else if (pot_east == pot_north && pot_east == pot_west)
      {
        rotate_right();
        forward();
      }
      else if (pot_east == pot_north || pot_east == pot_west)
      {
        rotate_right();
        forward();
      }
      else if (pot_north == pot_west)
      {
        forward();
      }
    }
    else if (trail_east == trail_north)
    {
      if (pot_north < pot_east)
      {
        forward();
      }
      else if (pot_north >= pot_east)
      {
        rotate_right();
        forward();
      }
    }
    else if (trail_east == trail_west)
    {
      if (pot_west < pot_east)
      {
        rotate_left();
        forward();
      }
      else if (pot_west >= pot_east)
      {
        rotate_right();
        forward();
      }
    }
    else if (trail_north == trail_west)
    {
      if (pot_north <= pot_west)
      {
        forward();
      }
      else if (pot_north > pot_west)
      {
        rotate_left();
        forward();
      }
    }
  }

  else if (face == east)
  {
    if (trail_east < trail_north && trail_east < trail_south)
    {
      forward();
    }
    else if (trail_north < trail_east && trail_north < trail_south)
    {
      rotate_left();
      forward();
    }
    else if (trail_south < trail_east && trail_south < trail_north)
    {
      rotate_right();
      forward();
    }
    else if (trail_east == trail_north && trail_east == trail_south)
    {
      if (pot_east < pot_north && pot_east < pot_south)
      {
        forward();
      }
      else if (pot_north < pot_east && pot_north < pot_south)
      {
        rotate_left();
        forward();
      }
      else if (pot_south < pot_east && pot_south < pot_north)
      {
        rotate_right();
        forward();
      }
      else if (pot_east == pot_north && pot_east == pot_south)
      {
        rotate_right();
        forward();
      }
      else if (pot_east == pot_north || pot_east == pot_south)
      {
        forward();
      }
      else if (pot_north == pot_south)
      {
        rotate_right();
        forward();
      }
    }
    else if (trail_east == trail_north)
    {
      if (pot_north < pot_east)
      {
        rotate_left();
        forward();
      }
      else if (pot_north >= pot_east)
      {
        forward();
      }
    }
    else if (trail_east == trail_south)
    {
      if (pot_south < pot_east)
      {
        rotate_right();
        forward();
      }
      else if (pot_south >= pot_east)
      {
        forward();
      }
    }
    else if (trail_south == trail_north)
    {
      if (pot_north < pot_south)
      {
        rotate_left();
        forward();
      }
      else if (pot_north >= pot_south)
      {
        rotate_right();
        forward();
      }
    }
  }
  else if (face == south)
  {
    if (trail_east < trail_south && trail_east < trail_west)
    {
      rotate_left();
      forward();
    }
    else if (trail_south < trail_east && trail_south < trail_west)
    {
      forward();
    }
    else if (trail_west < trail_east && trail_west < trail_south)
    {
      rotate_right();
      forward();
    }
    else if (trail_east == trail_south && trail_east == trail_west)
    {
      if (pot_east < pot_south && pot_east < pot_west)
      {
        rotate_left();
        forward();
      }
      else if (pot_south < pot_east && pot_south < pot_west)
      {
        forward();
      }
      else if (pot_west < pot_east && pot_west < pot_south)
      {
        rotate_right();
        forward();
      }
      else if (pot_east == pot_south && pot_south == pot_west)
      {
        rotate_right();
        forward();
      }
      else if (pot_south == pot_west || pot_west == pot_east)
      {
        rotate_right();
        forward();
      }
      else if (pot_east == pot_south)
      {
        forward();
      }
    }

    else if (trail_east == trail_south)
    {
      if (pot_south <= pot_east)
      {
        forward();
      }
      else if (pot_south > pot_east)
      {
        rotate_left();
        forward();
      }
    }
    else if (trail_east == trail_west)
    {
      if (pot_west <= pot_east)
      {
        rotate_right();
        forward();
      }
      else if (pot_west > pot_east)
      {
        rotate_left();
        forward();
      }
    }
    else if (trail_south == trail_west)
    {
      if (pot_south < pot_west)
      {
        forward();
      }
      else if (pot_south >= pot_west)
      {
        rotate_right();
        forward();
      }
    }
  }
  else if (face == west)
  {
    if (trail_west < trail_north && trail_west < trail_south)
    {
      forward();
    }
    else if ((trail_north < trail_west && trail_north < trail_west))
    {
      rotate_right();
      forward();
    }
    else if (trail_south < trail_west && trail_south < trail_north)
    {
      rotate_left();
      forward();
    }
    else if (trail_west == trail_north && trail_south == trail_west)
    {
      if (pot_west < pot_north && pot_west < pot_south)
      {
        forward();
      }
      else if (pot_north < pot_south && pot_north < pot_west)
      {
        rotate_right();
        forward();
      }
      else if (pot_south < pot_west && pot_south < pot_north)
      {
        rotate_left();
        forward();
      }
      else if (pot_west == pot_north && pot_west == pot_south)
      {
        rotate_right();
        forward();
      }
      else if (pot_west == pot_north || pot_north == pot_south)
      {
        rotate_right();
        forward();
      }
      else if (pot_north == pot_west)
      {
        forward();
      }
    }

    else if (trail_west == trail_north)
    {
      if (pot_north <= pot_west)
      {
        rotate_right();
        forward();
      }
      else if (pot_north > pot_west)
      {
        forward();
      }
    }
    else if (trail_west == trail_south)
    {
      if (pot_south < pot_west)
      {
        rotate_left();
        forward();
      }
      else if (pot_south >= pot_west)
      {
        forward();
      }
    }
    else if (trail_north == trail_west)
    {
      if (pot_north <= pot_west)
      {
        rotate_right();
        forward();
      }
      else if (pot_north > pot_west)
      {
        forward();
      }
    }
  }
}
// Decision-making routine
void decisions()
{
	// Decision-making is a quite important subroutine. With information from the wall below,
	// the algorithm now can make decisions on whether to move forward, rotate, or make a turn.
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



void firstPath()
{

  wallcheck();
  decision = way_left + way_right + way_front;
  while (decision == 2)
  {
    deadend[x_cor][y_cor] = 1;
    trail[x_cor][y_cor] = 200;
    // EEPROM.write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
    oneway();
    Close_Cell_After_Deadend();

    wallcheck();
    decision = way_left + way_right + way_front;
  }
  FirtPath = 1;
}