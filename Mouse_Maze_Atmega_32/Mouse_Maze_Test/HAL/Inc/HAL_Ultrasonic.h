
#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include <stdint.h>
#include "MCAL_gpio.h"
#include "MCAL_interrupt.h"

#define TRIG_PIN_1 0
#define ECHO_PIN_1 1

#define TRIG_PIN_2 2
#define ECHO_PIN_2 3

#define TRIG_PIN_3 4
#define ECHO_PIN_3 5

#define US_PORT GPIOA

#define SPEED_OF_SOUND_CM_PER_US 0.0343 // Speed of sound in centimeters per microsecond
#define MM_PER_CM 10                    // Millimeters per centimeter (1 cm = 10 mm)


/**********************Ultra_Trig_Init****************************
 * Description: This function is used to initialize all motors
 * thorough initializing all the bits connected to the motors
 * by calling MOTOR1_Init(), MOTOR2_Init(), MOTOR3_Init(), and
 * MOTOR4_Init() functions. It also initializes the TIMER0 and
 * starts it because of the PWM we use it to control the speed
 * of the motors.
 *
 * Input: It takes nothing like all the functions inside it
 * Return: It returns void
 **********************Ultra_Trig_Init****************************/

void Ultra_Init(S_GPIO_t *portName, uint8_t trig_pin, uint8_t echo_pin);
uint16_t GetUltrasonicDistance(S_GPIO_t *portName, uint8_t trig_pin, uint8_t echo_pin);

/*
 * Description: This function is used to check if the ultrasonic is
 * working correctly or not. It checks the echo pin for amount of time.
 *
 */
#endif /* ULTRASONIC_H */
