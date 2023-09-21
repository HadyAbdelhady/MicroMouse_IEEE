#ifndef __MOVEMENT__
#define __MOVEMENT__

#include "MCAL_gpio.h"

// Define movement states
#define FORWARD 0
#define HALF_TURN 1
#define RIGHT 2
#define LEFT 3
#define STOP 4

// Motor control pins
#define MOTOR_PORTS GPIOD

#define Motor_A_6 6     //          IN4
#define Motor_A_5 5     //Channel A IN3
#define Motor_B_4 4     //Channel B IN2
#define Motor_B_3 3     //          IN1 

#define Switches_PORT GPIOB

#define Full_cycle 92

// Function to Stop 
void Oop();

// Function to move forward 
void forward();

// Function to rotate left 
void rotate_left();

// Function to rotate right 
void rotate_right();

// Function to control movements based on the specified parameters 
void Movements(uint8_t MOVING, uint8_t speed_A, uint8_t speed_B);

// Function to perform a half-turn 
void half_turn();

#endif // __MOVEMENT__
