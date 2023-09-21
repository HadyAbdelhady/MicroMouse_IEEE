#include "G_init.h"
#include "Move.h"
#include "Navigate.h"
#include "MCAL_gpio.h"
#include "MCAL_interrupt.h"
#include "MCAL_timer.h"
#include "HAL_Ultrasonic.h"
#include "MCAL_eeprom.h"

int volatile unsigned long ticks_r = 0;
int volatile unsigned long ticks_l = 0;

// Timer configuration pointers
Timer_Config_t *configuartion = 0;
TIMER1_Config_t *configuartion_1 = 0;

// Maze-related variables and arrays
// uint8_t Cell[ROWS][COLS];
uint8_t x_cor, y_cor;
uint8_t face;
uint8_t trail[ROWS][COLS];
uint8_t deadend[ROWS][COLS];
uint8_t pot_field[ROWS][COLS];
uint8_t decision;
uint8_t way_left, way_front, way_right;

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
// Initialization function for the robot
void G_Init()
{
    // ACTUATORS
    MCAL_GPIO_SET_Pin(MOTOR_PORTS, Output, Motor_A_6);
    MCAL_GPIO_SET_Pin(MOTOR_PORTS, Output, Motor_A_5); // OC1A
    MCAL_GPIO_SET_Pin(MOTOR_PORTS, Output, Motor_B_3);
    MCAL_GPIO_SET_Pin(MOTOR_PORTS, Output, Motor_B_4); // OC1B

    // SENSORS
    MCAL_GPIO_SET_Pin(US_PORT, Output, ltrigger);
    MCAL_GPIO_SET_Pin(US_PORT, Output, ftrigger);
    MCAL_GPIO_SET_Pin(US_PORT, Output, rtrigger);
    MCAL_GPIO_SET_Pin(US_PORT, Input, lecho);
    MCAL_GPIO_SET_Pin(US_PORT, Input, fecho);
    MCAL_GPIO_SET_Pin(US_PORT, Input, recho);
    Ultra_Init(US_PORT, ltrigger, lecho);
    Ultra_Init(US_PORT, rtrigger, recho);
    Ultra_Init(US_PORT, ftrigger, fecho);

    // Switches
    MCAL_GPIO_SET_Pin(Switches_PORT, Input, NAV_switch);
    MCAL_GPIO_SET_Pin(Switches_PORT, Input, RUN_switch);
    MCAL_GPIO_SET_Pin(Switches_PORT, Input, CLR_EEPROM_switch);

    INT_0_init(on_change);
    INT_1_init(on_change);
    INT_2_init(rising_edge);

    INT0_CallBack(add_left);
    INT1_CallBack(add_right);
    INT2_CallBack(Reset_eeprom);

    configuartion->mode = Normal;
    configuartion->Overflow = TOIE_ENABLE;
    configuartion->Compare = OCIE_ENABLE; // to control the ultraSonic and motors
    configuartion->CLK = PRESCALING_CLK64;
    configuartion->PWM0_MODE = Fast_PWM;
    configuartion->PWM0_STATE = PWM0_STATE_Disable;

    configuartion_1->MODE = PWM1_Fast_ICR1_MODE;
    configuartion_1->OCAM_Interrupt = OCMIE1A_Enable;
    configuartion_1->OCBM_Interrupt = OCMIE1B_Enable;
    configuartion_1->SELECT_CHANNEL = TIMER1_SELECT_CHANNEL_A_B;
    configuartion_1->PRESCALER_CLK = PRESCALING_CLK64;
    configuartion_1->OVF_Interrupt = TOVIE1_Disable;
    configuartion_1->PWM1_STATE = PWM1_STATE_CHA_CHB_INVERTING;

    TIMER0_Init(configuartion);
    TIMER1_Init(configuartion_1);

    TIMER1_SetICR1Value(0x00FF);

    TIMER0_Stop();
    TIMER1_Stop();

    GIE_enable();

    // Initial position and orientation
    x_cor = 0;
    y_cor = 0;
    face = north;

    // Initialize maze-related arrays
    array_null();
    if (isNavigating)
        Cells_init_FloodFill();
    else
        readMazeFromEEPROM();

    // delay_ms(3000);
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
}

// Function to increment the left tick count
void add_left()
{
    ticks_l++;
}
void Reset_eeprom()
{
   EEPROM_erase(0,1023);
}