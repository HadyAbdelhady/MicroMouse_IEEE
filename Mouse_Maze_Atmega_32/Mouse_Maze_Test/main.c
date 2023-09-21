#include "G_init.h"
#include "PID.h"
#include "Move.h"
#include "Navigate.h"
#include "MCAL_eeprom.h"
#include "Maze_Run.h"
#include "MCAL_uart.h"
#include "HAL_Ultrasonic.h"
void Ultrasonics_Test()
{
	// Check if there are walls in different directions
	uint8_t *way_left = (uint8_t *)wall_present(US_PORT, ltrigger, lecho);
	uint8_t *way_front = (uint8_t *)wall_present(US_PORT, ftrigger, fecho);
	uint8_t *way_right = (uint8_t *)wall_present(US_PORT, rtrigger, recho);

	UART_write_string("Left Sensor:");
	UART_write_string(way_left);

	UART_write_string("Front Sensor:");
	UART_write_string(way_front);

	UART_write_string("Right Sensor:");
	UART_write_string(way_right);

	UART_write_char('\n');
}
void Motors_Test()
{
	// Rotate Left
	PID_Controller(LEFT);
	// rotate_left();

	// Rotate Right
	PID_Controller(RIGHT);
	// rotate_right();

	// Forward
	PID_Controller(FORWARD);
	// forward();
}

void Maze_Navigate()
{
	// This function is responsible for navigating the maze.
	// Check if the maze solving is finished.
	if (!FirtPath)
		firstPath();
	if (Navegating_times == DONE_Navigating)
		Go_Back();
	finishcheck();
	// Check for walls and update maze information.
	wallcheck();
	// Make navigation decisions based on the current state.
	decisions();
	// Perform the next move based on the decisions.
	Oop();
	// Delay for a half second (500 milliseconds).
	delay_ms(500);
}

void Maze_Run()
{
	// Find the lowest path through the maze.
	Find_Lowest_Path();
	wallcheck();
}

int main()
{
	// Initialize the micro-controller and peripherals.
	G_Init();
	while (1)
	{
		// Check if the navigation switch is on (GPIOA_PIN_0) and the run switch is off (GPIOA_PIN_1).
		if (MCAL_GPIO_ReadPin(Switches_PORT, NAV_switch) && !MCAL_GPIO_ReadPin(Switches_PORT, RUN_switch))
		{
			// Set the navigation mode flag.
			isNavigating = 1;
		}
		// Check if the navigation switch is off and the run switch is on.
		else if (!MCAL_GPIO_ReadPin(Switches_PORT, NAV_switch) && MCAL_GPIO_ReadPin(Switches_PORT, RUN_switch))
		{
			// Set the run mode flag.
			isNavigating = 0;
		}
		// Check the current mode and execute the corresponding function.
		if (isNavigating)
		{
			// Execute the maze navigation function.
			Maze_Navigate();
		}
		else if (!isNavigating)
		{
			// Execute the maze solving function.
			Maze_Run();
		}
		else
		{
			// Handle any other case (shouldn't reach this point).
			Oop();
		}
	}

	return 0;
}
