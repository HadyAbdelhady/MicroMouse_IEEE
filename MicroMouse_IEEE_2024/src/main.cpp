#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <STM32FreeRTOS.h> // STM32duino's built-in FreeRTOS
#include <Wire.h>
// #include <EEPROM.h>

#define v_Walls_BASE ((ROWS - 1) * 10 + (COLS))     // last address of the eeprom to Save the Cells values
#define h_Walls_BASE 2 * ((ROWS - 1) * 10 + (COLS)) // last address of the eeprom to Save the V_Walls values
// Define movement states
#define FORWARD 0
#define HALF_TURN 1
#define RIGHT 2
#define LEFT 3
#define STOP 4

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

typedef enum DIRECTION
{
  DONTCARE,
  NORTH,
  SOUTH,
  EAST,
  WEST
} E_Direction_t;

#define M_distance 100
#define DONE_Navigating 3
#define WINDOW_SIZE 10 // Window size for the moving average filter

bool MovedCell = false;

uint8_t x_cor;
uint8_t y_cor;
uint8_t trail[ROWS][COLS];
uint8_t deadend[ROWS][COLS];
uint8_t pot_field[ROWS][COLS];
uint8_t FirtPath = 0;

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

unsigned long start_time = 0;
unsigned long previousTime = 0;
float total_displacement = 0.0;
double ax_samples[WINDOW_SIZE] = {0};
int sample_index = 0;

float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gyroXcal = 0, gyroYcal = 0, gyroZcal = 0;

float yaw = 0.0, PrevYaw = 0.0;
float Accelration = 0, Velocity = 0;
int Displacement = 0;
bool t_flag = false;
Adafruit_MPU6050 mpu;
void Oop();

// Declare external timer configuration variables
// Declare a 2D array to represent maze cells
uint8_t Cell[ROWS][COLS];
uint8_t PATH[ROWS][COLS];

uint8_t isNavigating = 1; // navigating should be button to check wheather you are navigating or not (TODO) SOLVING MAZE --> it's done but not tested properly

/**
 * @brief Determines the current direction based on the yaw angle.
 *
 * This function checks the current yaw angle and returns the corresponding cardinal direction.
 *
 * @return E_Direction_t The current direction (NORTH, SOUTH, WEST, EAST, or UNKNOWN).
 */
E_Direction_t CurrentDirection()
{
  if ((yaw >= 345 || yaw <= 15)) // 0
    return NORTH;
  else if ((yaw >= 165 && yaw <= 195)) // 180
    return SOUTH;
  else if ((yaw >= 75 && yaw <= 105)) // 90
    return WEST;
  else if ((yaw >= 255 && yaw <= 285)) // 270
    return EAST;
  return DONTCARE;
}
void calibrate_sensor()
{
  Serial.println("Calibrating...");

  int samples = 1000;
  for (int i = 0; i < samples; i++)
  {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    ax_offset += accel.acceleration.x;
    ay_offset += accel.acceleration.y;
    az_offset += accel.acceleration.z;

    gyroXcal += gyro.gyro.x;
    gyroYcal += gyro.gyro.y;
    gyroZcal += gyro.gyro.z;

    delay(2); // Small delay to ensure accurate readings
  }

  ax_offset /= samples;
  ay_offset /= samples;
  az_offset /= samples;

  gyroXcal /= samples;
  gyroYcal /= samples;
  gyroZcal /= samples;

  Serial.println("\nCalibration complete.");
  Serial.print("Accel Offsets: ");
  Serial.print(ax_offset);
  Serial.print(", ");
  Serial.print(ay_offset);
  Serial.print(", ");
  Serial.println(az_offset);
  Serial.print("Gyro Offsets: ");
  Serial.print(gyroXcal);
  Serial.print(", ");
  Serial.print(gyroYcal);
  Serial.print(", ");
  Serial.println(gyroZcal);
}

void initializeMPU()
{
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");
  // SerialBT.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  calibrate_sensor();
  delay(100);
}
void start_timer()
{
  start_time = millis();
}

void readSensorData(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp)
{
  mpu.getEvent(accel, gyro, temp);
  // Apply calibration offsets
  // float ax = accel.acceleration.x - ax_offset;
  // float ay = accel.acceleration.y - ay_offset;
  // float az = accel.acceleration.z - az_offset;

  // float gx = gyro.gyro.x - gyroXcal;
  // float gy = gyro.gyro.y - gyroYcal;
  // float gz = gyro.gyro.z - gyroZcal;
}

void printResults()
{
  Serial.print("A : ");
  Serial.print(Accelration);
  Serial.print(" V : ");
  Serial.print(Velocity);
  Serial.print(" D: ");
  Serial.println(Displacement);
  // Serial.print(" m, Yaw: ");
  // SerialBT.print(" m, Yaw: ");
  // Serial.print(yaw);
  // Serial.println(" degrees");
}

/**
 * @brief Calculates the moving average of the given samples.
 *
 * @param new_sample The new sample to add to the moving average.
 * @param samples The array holding the past samples.
 * @param size The size of the samples array.
 * @return double The updated moving average.
 */
double moving_average(double new_sample, double *samples, int size)
{
  samples[sample_index % size] = new_sample;
  sample_index++;
  double sum = 0;
  for (int i = 0; i < size; i++)
  {
    sum += samples[i];
  }
  return sum / size;
}

/**
 * @brief Normalizes an angle to the range [0, 360] degrees.
 *
 * @param angle The angle to normalize.
 * @return float The normalized angle.
 */
float normalize_angle(float angle)
{
  // Use modulo to wrap the angle within 0 to 360
  angle = fmod(angle, 360.0);

  // Ensure positive angle
  if (angle < 0)
    angle += 360.0;

  return angle;
}

void Calculate_Accelration_Velocity_Displacement_Yaw(void *pvParameters)
{
  while (true)
  {
    unsigned long currentTime = millis();
    float elapsedTime = (currentTime - previousTime) / 1000.0; // Convert to seconds

    sensors_event_t accel, gyro, temp;
    readSensorData(&accel, &gyro, &temp);
    float ax = accel.acceleration.x - ax_offset;

    // Calculate acceleration in m/s^2
    Accelration = moving_average(ax, ax_samples, WINDOW_SIZE);
    // Integrate acceleration to get velocity
    Velocity = fabs(Accelration) * elapsedTime;

    // Integrate velocity to get displacement
    Displacement = Velocity * elapsedTime;
    if (total_displacement == 15)
    {
      total_displacement = 0;
      MovedCell = true;
    }
    else
    {
      MovedCell = false;
      total_displacement += Displacement;
    }
    // Get the yaw angle (gyroscope z-axis)
    // Integrate gyroscope data to get yaw angle
    float gz = gyro.gyro.z - gyroZcal;

    yaw += gz * elapsedTime;    // Integrate to get angle in degrees
    yaw = normalize_angle(yaw); // Normalize to 0-360 degrees

    // yaw = normalize_angle(gyro.gyro.z); // In radians/sec

    printResults();

    previousTime = currentTime;
  }
}
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
  if (CurrentDirection() == NORTH)
  {
    if (way_left)
      setVerticalWall(x, y);
    if (way_front)
      setHorizontalWall(x, y + 1);
    if (way_right)
      setVerticalWall(x + 1, y);
  }
  else if (CurrentDirection() == EAST)
  {
    if (way_left)
      setHorizontalWall(x, y + 1);
    if (way_front)
      setVerticalWall(x + 1, y);
    if (way_right)
      setHorizontalWall(x, y);
  }
  else if (CurrentDirection() == SOUTH)
  {
    if (way_left)
      setVerticalWall(x + 1, y);
    if (way_front)
      setHorizontalWall(x, y);
    if (way_right)
      setVerticalWall(x, y);
  }
  else if (CurrentDirection() == WEST)
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

// Rotate 90 degrees counterclockwise
void rotate_left()
{
  if (CurrentDirection() == NORTH)
  {
    while (CurrentDirection() == WEST)
      Movements(RIGHT, 100, 100);
  }

  else if (CurrentDirection() == EAST)
  {
    while (CurrentDirection() == NORTH)
      Movements(RIGHT, 100, 100);
  }

  else if (CurrentDirection() == SOUTH)
  {
    while (CurrentDirection() == EAST)
      Movements(RIGHT, 100, 100);
  }

  else if (CurrentDirection() == WEST)
  {
    while (CurrentDirection() == SOUTH)
      Movements(RIGHT, 100, 100);
  }
}

// Rotate 90 degrees clockwise
void rotate_right()
{

  if (CurrentDirection() == NORTH)
  {
    while (CurrentDirection() == EAST)
      Movements(LEFT, 100, 100);
  }

  else if (CurrentDirection() == EAST)
  {
    while (CurrentDirection() == SOUTH)
      Movements(LEFT, 100, 100);
  }

  else if (CurrentDirection() == SOUTH)
  {
    while (CurrentDirection() == WEST)
      Movements(LEFT, 100, 100);
  }

  else if (CurrentDirection() == WEST)
  {
    while (CurrentDirection() == NORTH)
      Movements(LEFT, 100, 100);
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

  while (!MovedCell)
    Movements(FORWARD, 100, 100);
}

void Close_Cell_After_Deadend()
{
  if (CurrentDirection() == NORTH)
    setHorizontalWall(x_cor, y_cor);
  else if (CurrentDirection() == SOUTH)
    setHorizontalWall(x_cor, y_cor + 1);
  else if (CurrentDirection() == EAST)
    setVerticalWall(x_cor, y_cor);
  else if (CurrentDirection() == WEST)
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

  if (CurrentDirection() == NORTH)
  {
    while (CurrentDirection() == SOUTH)
      Movements(RIGHT, 100, 100);
  }

  else if (CurrentDirection() == EAST)
  {
    while (CurrentDirection() == WEST)
      Movements(RIGHT, 100, 100);
  }

  else if (CurrentDirection() == SOUTH)
  {
    while (CurrentDirection() == NORTH)
      Movements(RIGHT, 100, 100);
  }

  else if (CurrentDirection() == WEST)
  {
    while (CurrentDirection() == EAST)
      Movements(RIGHT, 100, 100);
  }
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
    if (CurrentDirection() == NORTH)
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
    else if (CurrentDirection() == EAST)
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
    else if (CurrentDirection() == SOUTH)
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
    else if (CurrentDirection() == WEST)
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
    if (CurrentDirection() == NORTH)
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
    else if (CurrentDirection() == EAST)
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
    else if (CurrentDirection() == SOUTH)
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
    else if (CurrentDirection() == WEST)
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
    if (CurrentDirection() == NORTH)
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
    else if (CurrentDirection() == EAST)
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
    else if (CurrentDirection() == SOUTH)
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
    else if (CurrentDirection() == WEST)
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

  if (CurrentDirection() == NORTH)
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

  else if (CurrentDirection() == EAST)
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
  else if (CurrentDirection() == SOUTH)
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
  else if (CurrentDirection() == WEST)
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

  initializeMPU();

  // Serial.begin(9600);
  // Initialize other variables and arrays
  x_cor = 0;
  y_cor = 0;

  // Initialize maze-related arrays
  array_null();
  if (isNavigating)
    Cells_init_FloodFill();
  else
    readMazeFromEEPROM();

  // Create a FreeRTOS task
  xTaskCreate(
      Calculate_Accelration_Velocity_Displacement_Yaw, // Task function
      "MotionTask",                                    // Task name
      1000,                                            // Stack size (in words, not bytes)
      NULL,                                            // Task input parameter (if any)
      1,                                               // Task priority (higher number = higher priority)
      NULL                                             // Task handle (optional)
  );

  // Start the scheduler
  vTaskStartScheduler();
  previousTime = millis();
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
  // cli(); // Disable global interrupts
  // ticks_l = 0;
  // ticks_r = 0;
  // sei(); // Enable global interrupts
  // while (ticks_l < MOVED_CELL)
  // {
  //   Movements(FORWARD, 100, 100);
  // }
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
  //  Ultrasonics_Test();
  Maze_Navigate();
}
