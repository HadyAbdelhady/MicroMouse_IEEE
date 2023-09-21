#ifndef GPIO_AVR_H_
#define GPIO_AVR_H_

#include <stdint.h>

// Define the GPIO struct
typedef struct {
    volatile uint8_t PIN;     // Input Pins Address
    volatile uint8_t DDR;     // Data Direction Register
    volatile uint8_t PORT;    // Output Register
} S_GPIO_t;

// Define the GPIO pin configuration struct
typedef struct {
    uint8_t GPIO_Pin_Number;   // Specifies the GPIO PIN to be configured
    uint8_t GPIO_Mode;         // Specifies the Operating Mode of the Selected PIN
} GPIO_Pin_Config_t;

// GPIO Mode definitions
#define GPIO_Mode_Analog     0


// GPIO PIN state definitions
typedef enum {
    Input = 0,
    Output = 1
} GPIO_PIN_Mode;
// GPIO PIN state definitions
typedef enum {
    LOW = 0,
    HIGH = 1
} GPIO_PIN_State;


// Define the GPIO base addresses
#define GPIOA ((S_GPIO_t *) 0x39)
#define GPIOB ((S_GPIO_t *)0x36)
#define GPIOC ((S_GPIO_t *)0x33)
#define GPIOD ((S_GPIO_t *)0x30)



// Define GPIO_PINs
#define GPIO_PIN_0   0x01
#define GPIO_PIN_1   0x02
#define GPIO_PIN_2   0x04
#define GPIO_PIN_3   0x08
#define GPIO_PIN_4   0x10
#define GPIO_PIN_5   0x20
#define GPIO_PIN_6   0x40
#define GPIO_PIN_7   0x80

// APIs Supported by "MCAL GPIO DRIVER"
void MCAL_GPIO_Reset(S_GPIO_t *GPIOx);
uint8_t MCAL_GPIO_ReadPin(S_GPIO_t *GPIOx, uint16_t Pin_Number);
uint16_t MCAL_GPIO_ReadPORT(S_GPIO_t *GPIOx);
void MCAL_GPIO_WritePin(S_GPIO_t *GPIOx, uint16_t Pin_Number, GPIO_PIN_State value);
void MCAL_GPIO_WritePort(S_GPIO_t *GPIOx, uint16_t value);
void MCAL_GPIO_TogglePin(S_GPIO_t *GPIOx, uint16_t Pin_Number);
void MCAL_GPIO_SET_Pin(S_GPIO_t *GPIOx, uint8_t mode, uint8_t number);
void MCAL_GPIO_Set_Port_Direction(S_GPIO_t *GPIOx, uint16_t value);
#endif /* GPIO_AVR_H_ */
