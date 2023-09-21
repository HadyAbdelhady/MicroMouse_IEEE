#include "Move.h"
#include "PID.h"
#include "Navigate.h"
#include "MCAL_gpio.h"
#include "MCAL_interrupt.h"
#include "MCAL_eeprom.h"

// Stop the motors
void Oop()
{
    // Stop all movements
    Movements(STOP, 0, 0);
}

// Rotate 90 degrees counterclockwise
void rotate_left()
{
    // Reset the tick counters for motor rotation
    GIE_disable();
    ticks_l = 0;
    ticks_r = 0;
    GIE_enable();
    // Rotate the micromouse 90 degrees (20 ticks per 90 degrees)
    while ((ticks_r < (Full_cycle / 4)) || (ticks_l < (Full_cycle / 4)))
    {
        // Use the PID controller to control the left wheel's motion
        PID_Controller(LEFT);
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
    GIE_disable();
    ticks_l = 0;
    ticks_r = 0;
    GIE_enable();
    // Rotate the micromouse 90 degrees (20 ticks per 90 degrees)
    while ((ticks_r < (Full_cycle / 4)) || (ticks_l < (Full_cycle / 4)))
    {
        // Use the PID controller to control the right wheel's motion
        PID_Controller(RIGHT);
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
            trail[x_cor][y_cor] == 0;
            EEPROM_write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
        }
        else
        {
            if (x_cor != 0 && y_cor != 0)
            {
                trail[x_cor][y_cor]++;
                EEPROM_write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
            }
            else
            {
                trail[x_cor][y_cor] = 255;
                EEPROM_write(x_cor * 10 + y_cor, trail[x_cor][y_cor]);
            }
        }
    }

    // Reset encoder counts for the next control iteration
    // Disable interrupts while resetting to ensure atomicity
    GIE_disable();
    ticks_l = 0;
    ticks_r = 0;
    GIE_enable();

    // Move the micromouse forward by 92 ticks (equivalent to one cell)
    while ((ticks_r < Full_cycle) || (ticks_l < Full_cycle))
    {
        // Use the PID controller to maintain forward motion
        PID_Controller(FORWARD);
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

// Control the motors based on the specified movement and speeds
void Movements(uint8_t MOVING, uint8_t speed_A, uint8_t speed_B)
{

    TIMER0_Start();
    TIMER1_Start();

    switch (MOVING)
    {
    case FORWARD:
        TIMER1_SetCounterValue(0);
        TIMER0_SetCounter(0);

        // Right motor
        MCAL_GPIO_WritePin(MOTOR_PORTS, Motor_A_6, LOW);
        TIMER1_SetCompareValue(speed_A, TIMER1_SELECT_CHANNEL_A);

        // Left motor
        MCAL_GPIO_WritePin(MOTOR_PORTS, Motor_B_3, LOW);
        TIMER1_SetCompareValue(speed_B, TIMER1_SELECT_CHANNEL_B);
        break;

    case HALF_TURN:

        // Right motor
        MCAL_GPIO_WritePin(MOTOR_PORTS, Motor_A_6, HIGH);
        TIMER0_CALLBACK_Overflow_INTERRUPT(MA_OVF0);
        TIMER0_SetCompare(speed_A);
        TIMER0_CALLBACK_CompareMatch_INTERRUPT(MA_CM0);

        TIMER0_SetCounter(0);
        TIMER1_SetCounterValue(0);

        // CustomPWM(speed_A, Motor_A_6, MOTOR_PORTS);
        MCAL_GPIO_WritePin(MOTOR_PORTS, Motor_A_5, LOW);

        // Left motor
        MCAL_GPIO_WritePin(MOTOR_PORTS, Motor_B_3, LOW);
        TIMER1_SetCompareValue(speed_B, TIMER1_SELECT_CHANNEL_B);
        break;

    case RIGHT:
        // Right motor
        MCAL_GPIO_WritePin(MOTOR_PORTS, Motor_A_6, HIGH);
        TIMER0_CALLBACK_Overflow_INTERRUPT(MA_OVF0);
        TIMER0_SetCompare(speed_A);
        TIMER0_CALLBACK_CompareMatch_INTERRUPT(MA_CM0);

        TIMER0_SetCounter(0);
        TIMER1_SetCounterValue(0);

        MCAL_GPIO_WritePin(MOTOR_PORTS, Motor_A_5, LOW);

        // Left motor
        MCAL_GPIO_WritePin(MOTOR_PORTS, Motor_B_3, LOW);
        TIMER1_SetCompareValue(speed_B, TIMER1_SELECT_CHANNEL_B);
        break;

    case LEFT:
        // Right motor
        MCAL_GPIO_WritePin(MOTOR_PORTS, Motor_A_6, LOW);
        TIMER1_SetCompareValue(speed_A, TIMER1_SELECT_CHANNEL_A);

        // Left motor
        MCAL_GPIO_WritePin(MOTOR_PORTS, Motor_B_3, HIGH);
        TIMER0_CALLBACK_Overflow_INTERRUPT(MB_OVF0);
        TIMER0_SetCompare(speed_B);
        TIMER0_CALLBACK_CompareMatch_INTERRUPT(MB_CM0);

        TIMER0_SetCounter(0);
        TIMER1_SetCounterValue(0);

        MCAL_GPIO_WritePin(MOTOR_PORTS, Motor_B_4, LOW);
        break;
		
    default:
        TIMER1_SetCounterValue(0);
        TIMER0_SetCounter(0);
        // Stop both motors
        MCAL_GPIO_WritePin(MOTOR_PORTS, Motor_B_4, LOW);
        MCAL_GPIO_WritePin(MOTOR_PORTS, Motor_B_3, LOW);
        MCAL_GPIO_WritePin(MOTOR_PORTS, Motor_A_6, LOW);
        MCAL_GPIO_WritePin(MOTOR_PORTS, Motor_A_5, LOW);
        break;
    }
    TIMER0_Stop();
    TIMER1_Stop();
}
