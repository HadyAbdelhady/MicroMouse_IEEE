#include <Arduino.h>
// #include <EEPROM.h>

#define v_Walls_BASE ((ROWS - 1) * 10 + (COLS))     // last address of the eeprom to Save the Cells values
#define h_Walls_BASE 2 * ((ROWS - 1) * 10 + (COLS)) // last address of the eeprom to Save the V_Walls values
// Define movement states
#define FORWARD 0
#define HALF_TURN 1
#define RIGHT 2
#define LEFT 3
#define STOP 4

#define Switches_PORT GPIOB

#define Full_cycle 700
#define MOVED_CELL (((Full_cycle) / 2))
#define TURN_CELL 200
#define CHANGE_FACE (((TURN_CELL)*2) + 80)
#define north_west 0
#define north_east 1
#define south_west 2
#define south_east 3

#define SPEED_OF_SOUND_CM_PER_US 0.0343 // Speed of sound in centimeters per microsecond
#define MM_PER_CM 10
#define NAV_switch 0
#define RUN_switch 1
#define CLR_EEPROM_switch 2

#define SMALLEST_DISTANCE 0
#define THRESHOLD_DISTANCE 75
#define LARGEST_DISTANCE 2880

// Define constants for maze dimensions
#define ROWS 16
#define COLS 16
#define Maze_Border 255 // A large number to prevent out-of-range
#define x_fin 8
#define y_fin 8

// Define Ultrasonic pins
#define lecho A5
#define ltrigger A4

#define fecho 8
#define ftrigger 7

#define recho 4
#define rtrigger 5

#define Motor_A_6 6  //          IN4
#define Motor_A_5 11 // Channel A IN3
#define Motor_B_4 9  // Channel B IN2
#define Motor_B_3 10 //          IN1

// Define compass directions
#define north 0
#define east 1
#define south 2
#define west 3
#define M_distance 100
#define DONE_Navigating 3
// Declare external variables and arrays

// Constants for PID tuning
#define Kp 1.0  // Proportional constant
#define Ki 0.01 // Integral constant
#define Kd 0.01 // Derivative constant

// Desired motor speed
#define setpoint 200

// Declare external variables
float prev_error;
float integral;
float integral_2;
float prev_error_2;

volatile unsigned long ticks_r;
volatile unsigned long ticks_l;

uint8_t x_cor;
uint8_t y_cor;
uint8_t face;
uint8_t trail[ROWS][COLS];
uint8_t deadend[ROWS][COLS];
uint8_t pot_field[ROWS][COLS];

uint8_t Blocking_V_Walls[ROWS + 1][COLS]; // Vertical walls Readings
uint8_t Blocking_H_Walls[ROWS][COLS + 1]; // Horizontal walls Readings
uint8_t decision;
uint8_t Navegating_times;
uint8_t way_left;
uint8_t way_front;
uint8_t way_right;
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
void Oop();

// Declare external timer configuration variables
// Declare a 2D array to represent maze cells
uint8_t Cell[ROWS][COLS];
uint8_t PATH[ROWS][COLS];

uint8_t isNavigating = 1; // navigating should be button to check wheather you are navigating or not (TODO) SOLVING MAZE --> it's done but not tested properly
// Measure and return the distance using the ultrasonic sensor
uint16_t GetUltrasonicDistance(uint8_t trig_pin, uint8_t echo_pin)
{

  // Trigger the ultrasonic sensor
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  /////////////////////////////////////// ARDUINO ///////////////////////////////////////////

  // Measure the time for the echo to return
  long duration = pulseIn(echo_pin, HIGH);

  // Calculate distance in centimeters
  float distance_cm = duration * 0.034 / 2.0; // Speed of sound is approximately 0.034 cm/microsecond

  return distance_cm * 10;
}

// Control the motors based on the specified movement and speeds
void Movements(uint8_t MOVING, uint8_t speed_A, uint8_t speed_B)
{
  switch (MOVING)
  {
  case FORWARD:
    // Increase speed gradually
    analogWrite(Motor_A_6, 0);
    analogWrite(Motor_A_5, speed_A);
    analogWrite(Motor_B_4, 0);
    analogWrite(Motor_B_3, speed_B);
    delay(10); // Adjust the delay for speed change rate
    break;

  case HALF_TURN:
    // Increase speed gradually
    analogWrite(Motor_A_6, speed_A);
    analogWrite(Motor_A_5, 0);
    analogWrite(Motor_B_4, 0);
    analogWrite(Motor_B_3, speed_B);
    delay(10); // Adjust the delay for speed change rate

    break;

  case RIGHT:
    // Increase speed gradually
    analogWrite(Motor_A_6, speed_A);
    analogWrite(Motor_A_5, 0);
    analogWrite(Motor_B_4, 0);
    analogWrite(Motor_B_3, speed_B);
    delay(10); // Adjust the delay for speed change rate
    break;

  case LEFT:
    // Increase speed gradually
    analogWrite(Motor_A_6, 0);
    analogWrite(Motor_A_5, speed_A);
    analogWrite(Motor_B_4, speed_B);
    analogWrite(Motor_B_3, 0);
    delay(10); // Adjust the delay for speed change rate break;
    break;
  case STOP:
    // Serial.println(MOVING);
    // Increase speed gradually
    analogWrite(Motor_A_6, 0);
    analogWrite(Motor_A_5, 0);
    analogWrite(Motor_B_4, 0);
    analogWrite(Motor_B_3, 0);
    delay(10); // Adjust the delay for speed change rate break;
    break;
  default:
    // Stop both motors
    // Increase speed gradually
    analogWrite(Motor_A_6, 0);
    analogWrite(Motor_A_5, 0);
    analogWrite(Motor_B_4, 0);
    analogWrite(Motor_B_3, 0);
    delay(10); // Adjust the delay for speed change rate
    break;
  }
}

uint8_t wall_present(uint8_t trigPin, uint8_t echoPin)
{
  // Measure the distance from the sensor to the wall
  uint32_t distance = GetUltrasonicDistance(trigPin, echoPin);

  // If the distance is greater than or equal to 100mm, assume no wall is present
  // If the distance is greater than or equal to 100mm, assume no wall is present
  if (distance > THRESHOLD_DISTANCE && distance <= LARGEST_DISTANCE)
    return 0; // No wall
  else if (distance <= THRESHOLD_DISTANCE || distance <= SMALLEST_DISTANCE || distance > LARGEST_DISTANCE)
    return 1; // Wall detected
  return 11;
}

void EEPROM_erase(uint16_t start_byte, uint16_t end_byte)
{
  for (; start_byte < end_byte; start_byte++)
  {
    // EEPROM.write(start_byte, 0xFF); // Write 0xFF to erase each byte
  }
}
// put function declarations here:
void readMazeFromEEPROM()
{
  // Read data from EEPROM and populate the 'maze' array
  // Adjust this part to match your EEPROM data format
  for (int x = 0; x < ROWS; x++)
  {
    // for (int y = 0; y < COLS; y++)
    // PATH[x][y] = EEPROM.read(x * 10 + y);
  }
  // Read data from EEPROM and store it in the 'PATH' array based on the EEPROM data format.

  for (int x = 0; x <= ROWS; x++)
  {
    for (int y = 0; y < COLS; y++)
    {
      // Blocking_V_Walls[x][y] = EEPROM.read(v_Walls_BASE + (x * ROWS + y));
    }
  }
  // Read data from EEPROM and store it in the 'Blocking_V_Walls' array based on the EEPROM data format.

  for (int x = 0; x < ROWS; x++)
  {
    for (int y = 0; y <= COLS; y++)
    {
      // Blocking_H_Walls[x][y] = EEPROM.read(h_Walls_BASE + (x * ROWS + y));
    }
  }
  // Read data from EEPROM and store it in the 'Blocking_H_Walls' array based on the EEPROM data format.
}
// Function to initialize maze cells for flood-fill algorithm
void Cells_init_FloodFill(void)
{
  for (uint8_t i = 0; i < ROWS; i++)
  {
    for (uint8_t j = 0; j < COLS; j++)
    {
      pot_field[i][j] = (uint8_t)abs(8 - i) + (uint8_t)abs(8 - j);
    }
  }
}

// Function to increment the right tick count
void add_right()
{
  ticks_r++;
  // Serial.print("RIGHT: ");
  // Serial.print(ticks_r);
  // Serial.print("\t\n");
}

// Function to increment the left tick count
void add_left()
{
  // if (ticks_l <= MOVED_CELL)
  ticks_l++;
  // Serial.print("LEFT: ");
  // Serial.print(ticks_l);
  // Serial.print("\t\n");
}
// Check if the micromouse has reached the finish cell
void finishcheck()
{
  if (x_cor == x_fin && y_cor == y_fin)
  {
    if (Navegating_times < DONE_Navigating)
    {
      Navegating_times++;
      delay(15 * 1000);
      x_cor = 0;
      y_cor = 0;
      face = north;
    }
  }
}

// Function to set a vertical wall at a specific location
void setVerticalWall(uint8_t x, uint8_t y)
{
  if (x >= 0 && x <= ROWS && y >= 0 && y < COLS)
  {
    // Write a value (e.g., 1) to the calculated EEPROM address
    // EEPROM.write(v_Walls_BASE + (x * ROWS + y), 1);
    Blocking_V_Walls[x][y] = 1;
  }
}

// Function to set a horizontal wall at a specific location
void setHorizontalWall(uint8_t x, uint8_t y)
{
  if (x >= 0 && x < ROWS && y >= 0 && y <= COLS)
  {
    // Write a value (e.g., 1) to the calculated EEPROM address
    // EEPROM.write(h_Walls_BASE + (x * ROWS + y), 1);
    Blocking_H_Walls[x][y] = 1;
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
// Check for walls using ultrasonic sensors and update variables
void wallcheck()
{
  Oop();
  // Check if there are walls in different directions
  way_left = wall_present(ltrigger, lecho);
  way_front = wall_present(ftrigger, fecho);
  way_right = wall_present(rtrigger, recho);
  if (isNavigating && (Navegating_times != DONE_Navigating))
  {
    DetermineWallTypes(x_cor, y_cor);
  }
}

// Function to set all maze-related arrays to zero
void array_null()
{
  for (uint8_t n = 0; n < ROWS; n++)
  {
    for (uint8_t m = 0; m < COLS; m++)
    {
      trail[m][n] = 0;
      deadend[m][n] = 0;
      pot_field[m][n] = 0;
    }
  }
}

// Stop the motors
void Oop()
{
  // Stop all movements
  Movements(STOP, 0, 0);
  delay(100);
}

// Function to calculate PID control output
uint16_t calculatePID_M1(float input) // 150 ---> 200
{
  // Calculate PID control output based on the input value and PID constants
  float error = setpoint - input;
  integral += error;
  float derivative = error - prev_error;

  float pid_output = Kp * error + Ki * integral + Kd * derivative;
  prev_error = error;
  return pid_output;
}

uint16_t calculatePID_M2(float input) // 150 ---> 200
{
  // Calculate PID control output based on the input value and PID constants
  float error = setpoint - input;
  integral_2 += error;
  float derivative = error - prev_error_2;

  float pid_output = Kp * (error) + Ki * integral_2 + Kd * derivative;
  prev_error_2 = error;
  return pid_output;
}
uint8_t customConstrain(uint16_t value, uint8_t minVal, uint8_t maxVal)
{
  if (value < minVal)
    return minVal;
  else if (value > maxVal)
    return maxVal;
  else
    return value;
}
// PID Controller function
void PID_Controller(uint8_t Moves)
{
  // Calculate motor speeds, apply PID control, and control the motors
  // This function calculates motor speeds based on PID control and controls the motors accordingly

  // Calculate left and right motor speeds in RPM (Revolutions Per Minute) based on encoder counts
  // Adjust 'ticksPerResolution' as needed for your specific encoder and system

  float ticksPerResolution = 26;
  float leftSpeed = (ticks_l / ticksPerResolution) * 60; // Convert to RPM
  float rightSpeed = (ticks_r / ticksPerResolution) * 60; // Convert to RPM

  // Apply PID control to adjust motor speeds individually
  uint16_t leftOutput = calculatePID_M1(leftSpeed);
  uint16_t rightOutput = calculatePID_M2(rightSpeed);

  // Constrain the PID control output to ensure it falls within a valid range
  uint8_t Speed_1 = customConstrain(leftOutput, 0, 255);
  uint8_t Speed_2 = customConstrain(rightOutput, 0, 255);

  // Call the 'Movements()' function to control the motors based on calculated speeds
  Movements(Moves, Speed_1, Speed_2);
  // Delay for control update interval
  delay(100); // Adjust the delay as needed for your application
}
// Rotate 90 degrees counterclockwise
void rotate_left()
{
  // Reset the tick counters for motor rotation

  cli(); // Disable global interrupts
  ticks_l = 0;
  ticks_r = 0;
  sei(); // Enable global interrupts

  // Rotate the micromouse 90 degrees (20 ticks per 90 degrees)
  // while (/*(ticks_r < (Full_cycle / 4)) ||*/ (ticks_l < TURN_CELL) )
  // {
  //   // Use the PID controller to control the left wheel's motion
  //   PID_Controller(LEFT);
  // }
  while (ticks_l < TURN_CELL)
  {
    Movements(LEFT, 100, 100);
  }

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
  cli(); // Disable global interrupts
  ticks_l = 0;
  ticks_r = 0;
  sei(); // Enable global interrupts

  // Rotate the micromouse 90 degrees (20 ticks per 90 degrees)
  // while ( /*(ticks_r < (Full_cycle / 4)) ||*/ (ticks_l < TURN_CELL) )
  // {
  //   // Use the PID controller to control the right wheel's motion
  //   PID_Controller(RIGHT);
  // }
  while (ticks_l < TURN_CELL)
  {
    Movements(RIGHT, 100, 100);
  }

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

// Move forward one cell (92 ticks)
void forward()
{
  // Increment the trail marker for the current cell
  if (isNavigating)
  {
    if (Navegating_times == DONE_Navigating)
    {
      trail[x_cor][y_cor] = 0;
      // EEPROM.write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
    }
    else
    {
      if (x_cor != 0 && y_cor != 0)
      {
        trail[x_cor][y_cor]++;
        // EEPROM.write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
      }
      else
      {
        trail[x_cor][y_cor] = 255;
        // EEPROM.write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
      }
    }
  }

  // Reset encoder counts for the next control iteration
  // Disable interrupts while resetting to ensure atomicity
  cli(); // Disable global interrupts
  ticks_l = 0;
  ticks_r = 0;
  sei(); // Enable global interrupts

  // Move the micromouse forward by 92 ticks (equivalent to one cell)
  // while (/*(ticks_r < Full_cycle) || */ (ticks_l < MOVED_CELL))
  // {
  //   // Use the PID controller to maintain forward motion
  //   PID_Controller(FORWARD);
  // }
  while (ticks_l<MOVED_CELL)
  {
    Movements(FORWARD,100,100);
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

// Handle one-way paths
void oneway()
{
  if (way_left == 0)
  {
    // If the left path is open, turn left and move forward
    rotate_left();
    forward();
  }
  else if (way_right == 0)
  {
    // If the right path is open, turn right and move forward
    rotate_right();
    forward();
  }
  else if (way_front == 0)
  {
    // If the front path is open, move forward
    forward();
  }
}

// Perform a half-turn (rotate 180 degrees clockwise)
void half_turn()
{

  /////////////////////////////////////////// ARDUINO////////////////////////////////////////////////////
  cli(); // Disable global interrupts
  ticks_l = 0;
  ticks_r = 0;
  sei(); // Enable global interrupts

  // Rotate 180 degrees clockwise (40 ticks per 180 degrees)
  // while (/*(ticks_r < (Full_cycle / 2)) ||*/ (ticks_l < HALF_TURN))
  // {
  //   // Use the PID controller to control the wheels for a half-turn
  //   PID_Controller(HALF_TURN);
  // }
  while (ticks_l < CHANGE_FACE)
  {
    Movements(HALF_TURN, 100, 100);
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
    // EEPROM.write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
    oneway();
    Close_Cell_After_Deadend();
    wallcheck();
    decision = way_left + way_right + way_front;
  }
}
void Go_Back()
{
  half_turn();
  while (x_cor != 0 && y_cor != 0)
  {
    wallcheck();
    // Find_Lowest_Path();
  }
}

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

// Handle dead ends and find a way out
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

uint8_t FirtPath = 0;
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
void Maze_Navigate()
{
  // This function is responsible for navigating the maze.
  // Check if the maze solving is finished.

  finishcheck();

  if (!FirtPath)
    firstPath();
  // Check for walls and update maze information.
  wallcheck();
  // Make navigation decisions based on the current state.
  decisions();
  // Perform the next move based on the decisions.
}

void Maze_Run()
{
  // Find the lowest path through the maze.
  // Find_Lowest_Path();
  wallcheck();
}
void setup()
{
  // EEPROM.begin(); // Adjust the size based on your requirements
  // Initialize motor pins as OUTPUT
  pinMode(Motor_A_6, OUTPUT);
  pinMode(Motor_A_5, OUTPUT);
  pinMode(Motor_B_3, OUTPUT);
  pinMode(Motor_B_4, OUTPUT);

  // Initialize ultrasonic sensor trigger pins as OUTPUT
  pinMode(ltrigger, OUTPUT);
  pinMode(ftrigger, OUTPUT);
  pinMode(rtrigger, OUTPUT);

  // Initialize ultrasonic sensor echo pins as INPUT
  pinMode(lecho, INPUT);
  pinMode(fecho, INPUT);
  pinMode(recho, INPUT);

  // Initialize switch pins as INPUT
  pinMode(NAV_switch, INPUT);
  pinMode(RUN_switch, INPUT);
  pinMode(CLR_EEPROM_switch, INPUT);

  pinMode(2, INPUT);     // Set the pin as an input
  digitalWrite(2, HIGH); // Enable the external pull-up resistor

  // Initialize interrupts (INT0, INT1, INT2) and callbacks
  attachInterrupt(digitalPinToInterrupt(2), add_left, RISING);
  attachInterrupt(digitalPinToInterrupt(3), add_right, RISING);

  // Enable global interrupts
  sei();
  // Serial.begin(9600);
  // Initialize other variables and arrays
  x_cor = 0;
  y_cor = 0;
  face = north; // Replace 'north' with your actual orientation value

  // Initialize maze-related arrays
  array_null();
  if (isNavigating)
    Cells_init_FloodFill();
  else
    readMazeFromEEPROM();
}
void Ultrasonics_Test()
{
  // Check if there are walls in different directions
  uint32_t way_left = GetUltrasonicDistance(ltrigger, lecho);
  uint32_t way_front = GetUltrasonicDistance(ftrigger, fecho);
  uint32_t way_right = GetUltrasonicDistance(rtrigger, recho);

  Serial.print("Left Sensor:");
  Serial.print(way_left);

  Serial.print("\tFront Sensor: ");
  Serial.print(way_front);

  Serial.print("\tRight Sensor: ");
  Serial.println(way_right);
  Serial.print("==============================");
  Serial.println();
}
void Motors_Test()
{
  cli(); // Disable global interrupts
  ticks_l = 0;
  ticks_r = 0;
  sei(); // Enable global interrupts
  while (ticks_l < MOVED_CELL)
  {
    Movements(FORWARD, 100, 100);
  }
  Oop();
  delay(1000);
  rotate_left();
  // Movements(LEFT, 100, 100);
  Oop();
  delay(1000);
  rotate_right();
  // Movements(RIGHT, 100, 100);
  Oop();
  delay(1000);
  half_turn();
  // Movements(HALF_TURN, 100, 100);
  Oop();
  delay(1000);
}
void loop()
{
  // Motors_Test();
  Maze_Navigate();

  //  Ultrasonics_Test();
}
