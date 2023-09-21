#include "HAL_Ultrasonic.h"
#include "MCAL_gpio.h"
#include "MCAL_timer.h"
#include "Bit_Math.h"

// Function to calculate the total counter value including overflow counts
uint32_t Get_counter_value()
{
	return (TCNT0 + (TIMER0_u8_OVF_Number * 256)); // Calculate total counts
}

// Initialize the ultrasonic sensor
void Ultra_Init(S_GPIO_t *portName, uint8_t trig_pin, uint8_t echo_pin)
{
	// Configure the trigger and echo pins as output and input respectively
	MCAL_GPIO_SET_Pin(portName, Output, trig_pin);
	MCAL_GPIO_SET_Pin(portName, Input, echo_pin);
	MCAL_GPIO_WritePin(portName, echo_pin, LOW);
}

// Measure and return the distance using the ultrasonic sensor
uint16_t GetUltrasonicDistance(S_GPIO_t *portName, uint8_t trig_pin, uint8_t echo_pin)
{
	uint16_t distance;

	// Trigger the ultrasonic sensor
	MCAL_GPIO_WritePin(portName, trig_pin, HIGH);
	delay_us(10);
	MCAL_GPIO_WritePin(portName, trig_pin, LOW);

	// Measure the time for the echo to return
	while (!(MCAL_GPIO_ReadPin(portName, echo_pin)))
		;

	TIMER0_u8_OVF_Number = 0; // Clear overflow count
	TIMER0_CALLBACK_Overflow_INTERRUPT(NULL);
	TIMER0_SetCounter(0);
	TIMER0_Start();

	while (MCAL_GPIO_ReadPin(portName, echo_pin))
		;

	TIMER0_Stop();
	uint32_t Total_ticks = Get_counter_value();

	uint32_t elapsedTime = Total_ticks * (PRESCALER / CPU_F);					// 64 stands for the preScaler
																			// 8000000.0 is the cpu freq
	distance = (elapsedTime * SPEED_OF_SOUND_CM_PER_US * MM_PER_CM) / 2; 	// Distance in millimeters

	return distance;
}
