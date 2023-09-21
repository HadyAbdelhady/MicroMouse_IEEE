#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include "Bit_Math.h"
#define SETBIT(reg, bitNum, value)

// Define macros for UART_UCSRA_PIN_t
#define MPCM  0
#define U2X   1
#define PE    2
#define DOR   3
#define FE    4
#define UDRE  5
#define TXC   6
#define RXC   7

// Define macros for UART_UCSRB_PIN_t
#define TXB8  0
#define RXB8  1
#define UCSZ2 2
#define TXEN  3
#define RXEN  4
#define UDRIE 5
#define TXCIE 6
#define RXCIE 7

// Define macros for UART_UCSRC_PIN_t
#define UCPOL 0
#define UCSZ0 1
#define UCSZ1 2
#define USBS  3
#define UPM0  4
#define UPM1  5
#define UMSEL 6
#define URSEL 7
// Enumerate the UART register pins for UCSRA
/*
typedef enum
{
    MPCM,
    U2X,
    PE,
    DOR,
    FE,
    UDRE,
    TXC,
    RXC
} UART_UCSRA_PIN_t;

// Enumerate the UART register pins for UCSRB
typedef enum
{
    TXB8 = 0,
    RXB8,
    UCSZ2,
    TXEN,
    RXEN,
    UDRIE,
    TXCIE,
    RXCIE
} UART_UCSRB_PIN_t;

// Enumerate the UART register pins for UCSRC
typedef enum
{
    UCPOL = 0,
    UCSZ0,
    UCSZ1,
    USBS,
    UPM0,
    UPM1,
    UMSEL,
    URSEL
} UART_UCSRC_PIN_t;
*/
typedef enum
{
    Async,
    Sync
} UART_Mode;

typedef enum
{
    Parity_Disabled,
    Parity_Odd,
    Parity_Even
} UART_Parity;

// Define register addresses
#define UDR		*((volatile uint8_t *)0x2C)
#define UCSRA	*((volatile uint8_t *)0x2B)
#define UCSRB	*((volatile uint8_t *)0x2A)
#define UBRRL	*((volatile uint8_t *)0x29)
#define UCSRC	*((volatile uint8_t *)0x40)
#define UBRRH	*((volatile uint8_t *)0x40)
#define SREG	*((volatile uint8_t *)0x5F)

// Define a default stop character
#define DefaultStop '\r'

// Function prototypes
void UART_init(uint8_t num_data, uint8_t mode, uint8_t parity, uint8_t stop, uint16_t baud_rate);
void UART_write_char(char data);
void UART_write_string(char *str);
char UART_read_char(void);

#endif /* UART_H_ */
