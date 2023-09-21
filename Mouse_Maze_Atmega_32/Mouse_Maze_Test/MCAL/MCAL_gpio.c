#include "mcal_gpio.h"
#include "Bit_Math.h"
void MCAL_GPIO_SET_PinMode_and_number(S_GPIO_t *GPIOx, GPIO_Pin_Config_t *P_Config)
{
    // Configure the pin mode
    if (P_Config->GPIO_Mode == Input)
    {
        CLR_BIT(GPIOx->DDR, P_Config->GPIO_Pin_Number); // Set as input
    }
    else
    {
        SET_BIT(GPIOx->DDR, P_Config->GPIO_Pin_Number); // Set as output
    }
}
void MCAL_GPIO_SET_Pin(S_GPIO_t *GPIOx, uint8_t mode, uint8_t number)
{
    GPIO_Pin_Config_t *P_Config= 0;
    P_Config->GPIO_Mode = mode;
    P_Config->GPIO_Pin_Number = number;
    MCAL_GPIO_SET_PinMode_and_number(GPIOx, P_Config);
}
void MCAL_GPIO_Reset(S_GPIO_t *GPIOx)
{
    // Reset the port (set all pins to low)
    GPIOx->PORT = 0x00;
}
uint8_t MCAL_GPIO_ReadPin(S_GPIO_t *GPIOx, uint16_t Pin_Number)
{
    // Read the state of the specified pin
    return READ_BIT(GPIOx->PIN, Pin_Number);
}
uint16_t MCAL_GPIO_ReadPORT(S_GPIO_t *GPIOx)
{
    // Read the entire port state
    return GPIOx->PIN;
}
void MCAL_GPIO_WritePin(S_GPIO_t *GPIOx, uint16_t Pin_Number, GPIO_PIN_State value)
{
    // Write the specified value to the pin
    if (value == HIGH)
    {
        SET_BIT(GPIOx->PORT, Pin_Number);
    }
    else
    {
        CLR_BIT(GPIOx->PORT, Pin_Number);
    }
}
void MCAL_GPIO_WritePort(S_GPIO_t *GPIOx, uint16_t value)
{
    // Write the entire port
    GPIOx->PORT = value;
}
void MCAL_GPIO_Set_Port_Direction(S_GPIO_t *GPIOx, uint16_t value)
{
    // Write the entire port
    GPIOx->DDR = value;
}
void MCAL_GPIO_TogglePin(S_GPIO_t *GPIOx, uint16_t Pin_Number)
{
    // Toggle the state of the specified pin
    TOG_BIT(GPIOx->PORT, Pin_Number);
}