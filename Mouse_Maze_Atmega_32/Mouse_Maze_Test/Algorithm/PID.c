#include "PID.h"
#include "Move.h"
#include "MCAL_gpio.h"
#include "MCAL_interrupt.h"

// PID variables
float prev_error = 0;
float integral = 0;
float integral_2=0;
float prev_error_2 = 0;

// Function to toggle a GPIO pin for the left motor control
void MA_OVF0()
{
    // Toggle the specified bit (Motor_A_2) in the MOTOR_PORTS GPIO register
    TOG_BIT(*(volatile uint8_t*) MOTOR_PORTS, Motor_A_6);
}

// Function to toggle a GPIO pin for the right motor control
void MB_OVF0()
{
    // Toggle the specified bit (Motor_B_3) in the MOTOR_PORTS GPIO register
    TOG_BIT(*(volatile uint8_t*)MOTOR_PORTS, Motor_B_3);
}
void MA_CM0()
{
    // Toggle the specified bit (Motor_A_2) in the MOTOR_PORTS GPIO register
    TOG_BIT(*(volatile uint8_t*)MOTOR_PORTS, Motor_A_6);
}
void MB_CM0()
{
    // Toggle the specified bit (Motor_A_2) in the MOTOR_PORTS GPIO register
    TOG_BIT(*(volatile uint8_t*)MOTOR_PORTS, Motor_B_3);
}

// Function to calculate PID control output
uint16_t calculatePID_M1(float input) // 150 ---> 200
{
    // Calculate PID control output based on the input value and PID constants
    float error = setpoint- input;
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
    float leftSpeed = (ticks_l / ticksPerResolution) * 60;  // Convert to RPM
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
    delay_ms(100); // Adjust the delay as needed for your application
}