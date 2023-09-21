#ifndef __PID__
#define __PID__

#include "Inc/MCAL/MCAL_timer.h"
#include "Inc/Utilities/Bit_Math.h"

// Constants for PID tuning
#define Kp 1.0  // Proportional constant
#define Ki 0.01 // Integral constant
#define Kd 0.01 // Derivative constant

// Desired motor speed
#define setpoint 200

// Declare external variables
extern float prev_error;
extern float integral;
extern float integral_2;
extern float prev_error_2;

extern int volatile unsigned long ticks_r;
extern int volatile unsigned long ticks_l;

// Function to handle Motor A overflow interrupt
void MA_OVF0();

// Function to handle Motor B overflow interrupt
void MB_OVF0();
void MB_CM0();

void MA_CM0();

// Function to calculate the PID control output
uint16_t calculatePID_M1(float input);
uint16_t calculatePID_M2(float input);

// Function to constrain a value within a specified range
uint8_t customConstrain(uint16_t value, uint8_t minVal, uint8_t maxVal);

// Function to control the PID controller based on specified moves
void PID_Controller(uint8_t Moves);

#endif // __PID__
