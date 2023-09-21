#ifndef TIMER
#define TIMER

/******************************************************************************
*                               INCLUDES			                          *
// *******************************************************************************/
#include <stdint.h>
#include "Bit_Math.h"
/******************************************************************************
 *					Typedef-Defines-ENUMS	                				  *
 *******************************************************************************/

typedef void (*Ptr_Func)(void);
#define NULL (void *)0

extern uint8_t TIMER0_u8_OVF_Number; // Overflow count variable

// TIMSK: (Timer/Counter Interrupt Mask Register)
typedef enum
{
    TOIE0 = 0, // Bit 0 – TOIE0: Timer/Counter0 Overflow Interrupt Enable
    OCIE0,     // Bit 1 – OCIE0: Timer/Counter0 Output Compare Match Interrupt Enable
    TOIE1,
    OCIE1B,
    OCIE1A,
    TICIE1,
    TOIE2,
    OCIE2 = 7
} TIMER0_TIMSK_PIN;

// TCCR0 (Timer/Counter Control Register)
typedef enum
{
    CS00 = 0, // CLK System Regs
    CS01,
    CS02,
    WGM01,   // Bit 3 – WGM01:0: Waveform Generation Mode
    COM00,   // Bit 4 – COM00: Compare Match Output Mode
    COM01,   // Bit 5 – COM01: Compare Match Output Mode
    WGM00,   // Bit 6 – WGM01:0: Waveform Generation Mode
    FOC0 = 7 // Bit 7 – FOC0: Force Output Compare
} TIMER0_TCCR0_PIN;

// TIFR:  Timer/Counter Interrupt Flag Register
typedef enum
{
    TOV0 = 0, // • Bit 0 – TOV0: Timer/Counter0 Overflow Flag
    OCF0,     // • Bit 1 – OCF0: Output Compare Flag 0
    TOV1,
    OCF1B,
    OCF1A,
    ICF1,
    TOV2,
    OCF2 = 7
} TIMER0_TIFR_PIN;
typedef enum
{
    Normal,
    CTC = 0x08
} Timer_Modes_t;

/*• Bit 0 – TOIE0: Timer/Counter0 Overflow Interrupt Enable
When the TOIE0 bit is written to one, and the I-bit in the Status Register is set (one), the
Timer/Counter0 Overflow interrupt is enabled.*/
typedef enum
{
    TOIE_DISABLE,
    TOIE_ENABLE
} TOIE_t;

/*• Bit 1 – OCIE0: Timer/Counter0 Output Compare Match Interrupt Enable
When the OCIE0 bit is written to one, and the I-bit in the Status Register is set (one), the
Timer/Counter0 Compare Match interrupt is enabled.*/
typedef enum
{
    OCIE_DISABLE = 0x01,
    OCIE_ENABLE = 0x02
} OCIE_t;

typedef unsigned char E_STATUS_t;
#define E_OK (E_STATUS_t)(0)
#define E_NOK (E_STATUS_t)(1)

typedef enum
{
    NO_CLK_SRC,
    NO_PRESCALING,
    PRESCALING_CLK8,
    PRESCALING_CLK64,
    PRESCALING_CLK256,
    PRESCALING_CLK1024,
    EXT_CLK_FALLING,
    EXT_CLK_RISING,
} TIMER_CLOCK_t;
typedef enum
{
    PWM0_STATE_Disable,
    PWM0_STATE_INVERTING,
    PWM0_STATE_NON_INVERTING
} PWM0_STATE_t;
typedef enum
{
    Fast_PWM = 0x40,
    Phase_Correct_PWM = 0x08
} PWM_Modes_t;
/******************************************************************************
 *									STRUCT	                				  *
 *******************************************************************************/

typedef struct
{
    Timer_Modes_t mode;
    TOIE_t Overflow;
    OCIE_t Compare;
    TIMER_CLOCK_t CLK;
    PWM_Modes_t PWM0_MODE;
    PWM0_STATE_t PWM0_STATE;

} Timer_Config_t;

typedef struct
{
    uint8_t MODE;           // Timer mode, @ref TIMER1_MODE_Define
    uint8_t SELECT_CHANNEL; // Select between CHANNEL(A,B)	@ref TIMER1_SELECT_CHANNEL_Define
    uint8_t PRESCALER_CLK;  // Prescaler clock, @ref TIMER0_1_PRESCALER_CLK_Define
    uint8_t OCAM_Interrupt; // Output Compare A Match interrupt, @ref TIMER1_OCAMIE_Define
    uint8_t OCBM_Interrupt; // Output Compare B Match interrupt, @ref TIMER1_OCMBIE_Define
    uint8_t OVF_Interrupt;  // Overflow interrupt, @ref TIMER1_TOVIE1_Define
    uint8_t PWM1_STATE;     // PWM0 state, @ref PWM1_STATE_Define

} TIMER1_Config_t;

//******************************************************************
/*************************       MACROS        *******************
******************************************************************/

#define TCCR0 *((volatile uint8_t *)0x53)
// TCNT0: Timer/Counter0 (8 Bits)
#define TCNT0 *((volatile uint8_t *)0x52)
// OCR0: Timer/Counter0 Output Compare Register
#define OCR0 *((volatile uint8_t *)0x5C)

#define SREG *((volatile uint8_t *)0x5F)
#define SREG_BIT7 7

#define WDTCR *((volatile uint8_t *)0x41)

typedef enum
{
    TimeOut_16ms,
    TimeOut_32ms,
    TimeOut_65ms,
    TimeOut_130ms,
    TimeOut_260ms,
    TimeOut_520ms,
    TimeOut_1s,
    TimeOut_2s,
} WDT_TimeOut_t;

// Bit 3 – WDE: Watchdog Enable
#define WDE (uint8_t)3
// Bit 4 – WDTOE: Watchdog Turn-off Enable
#define WDTOE (uint8_t)4

//----------------
// TIMER1 Registers
//----------------
// TCCR1A: COM1A1 COM1A0 COM1B1 COM1B0 FOC1A FOC1B WGM11 WGM10
#define TCCR1A *((volatile uint8_t *)0x4F)
// TCCR1B: ICNC1 ICES1 – WGM13 WGM12 CS12 CS11 CS10
#define TCCR1B *((volatile uint8_t *)0x4E)

#define TCNT1 *((volatile uint16_t *)0x4C)
// TCNT1L: Timer/Counter1 – Counter Register Low Byte
#define TCNT1L *((volatile uint8_t *)0x4C)
// TCNT1H: Timer/Counter1 – Counter Register High Byte
#define TCNT1H *((volatile uint8_t *)0x4D)

#define OCR1A *((volatile uint16_t *)0x4A)
// OCR1AL: Timer/Counter1 – Output Compare Register A Low Byte
#define OCR1AL *((volatile uint8_t *)0x4A)
// OCR1AH: Timer/Counter1 – Output Compare Register A High Byte
#define OCR1AH *((volatile uint8_t *)0x4B)

#define OCR1B *((volatile uint16_t *)0x48)
// OCR1BL: Timer/Counter1 – Output Compare Register B Low Byte
#define OCR1BL *((volatile uint8_t *)0x48)
// OCR1BH: Timer/Counter1 – Output Compare Register B High Byte
#define OCR1BH *((volatile uint8_t *)0x49)

#define ICR1 *((volatile uint16_t *)0x46)
// ICR1L: Timer/Counter1 – Input Capture Register Low Byte
#define ICR1L *((volatile uint8_t *)0x46)
// ICR1H: Timer/Counter1 – Input Capture Register High Byte
#define ICR1H *((volatile uint8_t *)0x47)

//-----------------------
// COMMON TIMERS Registers
//-----------------------
// TIMSK: OCIE2 TOIE2 TICIE1 OCIE1A OCIE1B TOIE1 OCIE0 TOIE0
#define TIMSK *((volatile uint8_t *)0x59)
// TIFR: OCF2 TOV2 ICF1 OCF1A OCF1B TOV1 OCF0 TOV0
#define TIFR *((volatile uint8_t *)0x58)
//----------------------------

//*****************************************************************
//****************  TIMER1 MACROS References Define  **************
//*****************************************************************

//----------------------
//@ref PWM1_STATE_Define
//----------------------
#define PWM1_STATE_CHA_Disable ~(uint8_t)(((1 << COM1A1) | (1 << COM1A0)))                                     // Normal port operation, OC1A disconnected.
#define PWM1_STATE_CHB_Disable ~(uint8_t)(((1 << COM1B1) | (1 << COM1B0)))                                     // Normal port operation, OC1B disconnected.
#define PWM1_STATE_CHA_CHB_Disable ~(uint8_t)(((1 << COM1A1) | (1 << COM1A0)) | (1 << COM1B1) | (1 << COM1B0)) // Normal port operation,OC1A and OC1B disconnected.

#define PWM1_STATE_CHA_INVERTING (uint8_t)(1 << COM1A1 | 1 << COM1A0) //(CHA is inverting mode)
#define PWM1_STATE_CHB_INVERTING (uint8_t)(1 << COM1B1 | 1 << COM1B0) //(CHB is inverting mode)

#define PWM1_STATE_CHA_NON_INVERTING (uint8_t)(1 << COM1A1) //(CHA is non-inverting mode)
#define PWM1_STATE_CHB_NON_INVERTING (uint8_t)(1 << COM1B1) //(CHB is non-inverting mode)

#define PWM1_STATE_CHA_CHB_INVERTING (uint8_t)((1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0)) //(CHA and CHB are inverting mode)
#define PWM1_STATE_CHA_CHB_NON_INVERTING (uint8_t)((1 << COM1B1) | (1 << COM1A1))                             //(CHA and CHB are non-inverting mode)
#define PWM1_STATE_CHA_INVERTING_CHB_NON_INVERTING (uint8_t)((1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1))   //(CHA>>inverting mode,CHB>>non-inverting mode)
#define PWM1_STATE_CHB_INVERTING_CHA_NON_INVERTING (uint8_t)((1 << COM1B1) | (1 << COM1B0) | (1 << COM1A1))   //(CHB>>inverting mode,CHA>>non-inverting mode)

#define PWM1_Toggle_OC1A_On_Compare_Match (uint8_t)(1 << COM1A0) // WGM13:0 = 15: Toggle OC1A on Compare Match,
// OC1B disconnected (normal port operation).
//--------------------------------------------------------------------------------------

//--------------------------
//@ref TIMER1_OCMIE1A_Define
//--------------------------
// TIMSK: Bit 4 – OCIE1A: Timer/Counter1, Output Compare A Match Interrupt Enable
#define OCMIE1A_Disable ~((uint8_t)(1 << OCIE1A))
#define OCMIE1A_Enable (uint8_t)(1 << OCIE1A)

//--------------------------
//@ref TIMER1_OCMIE1B_Define
//--------------------------
// TIMSK: Bit 3 – OCIE1B: Timer/Counter1, Output Compare B Match Interrupt Enable
#define OCMIE1B_Disable ~((uint8_t)(1 << OCIE1B))
#define OCMIE1B_Enable (uint8_t)(1 << OCIE1B)

//-------------------------
//@ref TIMER1_TOVIE1_Define
//-------------------------
// TIMSK:  Bit 2 – TOIE1: Timer/Counter1, Overflow Interrupt Enable
#define TOVIE1_Disable ~((uint8_t)(1 << TOIE1))
#define TOVIE1_Enable (uint8_t)(1 << TOIE1)

//-------------------------
//@ref TIMER1_TICIE1_Define
//-------------------------
// NOT SUPPORTED!!!!!
// Bit 5 – TICIE1: Timer/Counter1, Input Capture Interrupt Enable
#define TICIE1_Disable ~((uint8_t)(1 << TICIE1)) // NOT SUPPORTED!!!!!
#define TICIE1_Enable (uint8_t)(1 << TICIE1)     // NOT SUPPORTED!!!!!

//*****************************************************************
//********************  TIMER1 ENUMS References Define  ***********
//*****************************************************************

//--------------------------
//@ref TIMER1_MODE_Define
//--------------------------
// enum for bits(WGM13,WGM12,WGM11,WGM10)
// WGM13,WGM12 @ TCCR1B bit(4,3)
// WGM11,WGM10 @ TCCR1A bit(1,0)
typedef enum
{
    TIMER1_NORMAL_MODE = 0, // TOP = (0xFFFF) ,Update of OCR1x = (Immediate) ,TOV1 Flag Set on (MAX)
    PWM1_Phase_Correct_8B_MODE,
    PWM1_Phase_Correct_9B_MODE,
    PWM1_Phase_Correct_10B_MODE,
    TIMER1_CTC_OCR1_MODE, // TOP = (OCR1) ,Update of OCR1x = (Immediate) ,TOV1 Flag Set on (MAX)
    PWM1_Fast_8B_MODE,
    PWM1_Fast_9B_MODE,
    PWM1_Fast_10B_MODE,
    PWM1_PhaseAndFreqCorrect_ICR1_MODE,
    PWM1_PhaseAndFreqCorrect_OCR1_MODE,
    PWM1_Phase_Correct_ICR1_MODE,
    PWM1_Phase_Correct_OCR1_MODE, // TOP = (OCR1) ,Update of OCR1x = (TOP) ,TOV1 Flag Set on (BOTTOM)
    TIMER1_CTC_ICR1_MODE,
    PWM1_Fast_ICR1_MODE = 14, // TOP = (ICR1) ,Update of OCR1x = (BOTTOM) ,TOV1 Flag Set on (TOP)
    PWM1_Fast_OCR1A_MODE,     // TOP = (OCR1A) ,Update of OCR1x = (BOTTOM) ,TOV1 Flag Set on (TOP)

} TIMER1_MODE_Define_E;

//-----------------------------------
//@ref TIMER1_SELECT_CHANNEL_Define
//-----------------------------------
typedef enum
{
    TIMER1_SELECT_CHANNEL_A,
    TIMER1_SELECT_CHANNEL_B,
    TIMER1_SELECT_CHANNEL_A_B,
    TIMER1_SELECT_NO_CHANNEL
} TIMER1_SELECT_UNIT_Define_E;
//---------------------------------
// BIT Positions in TIMER1 Registers
//---------------------------------
// TCCR1A: COM1A1 COM1A0 COM1B1 COM1B0 FOC1A FOC1B WGM11 WGM10
typedef enum
{
    WGM10 = 0, // Bit 0 – WGM10: Waveform Generation Mode
    WGM11,     // Bit 1 – WGM11: Waveform Generation Mode
    FOC1B,     // Bit 2 – FOC1B: Force Output Compare for Compare unit B
    FOC1A,     // Bit 3 – FOC1A: Force Output Compare for Compare unit A
    COM1B0,    // Bit 4 – COM1B0: Compare Output Mode for Compare unit B
    COM1B1,    // Bit 5 – COM1B1: Compare Output Mode for Compare unit B
    COM1A0,    // Bit 6 – COM1A0: Compare Output Mode for Compare unit A
    COM1A1 = 7 // Bit 7 – COM1A1: Compare Output Mode for Compare unit A
} TIMER1_TCCR1A_PINs_E;

// TCCR1B: ICNC1 ICES1 – WGM13 WGM12 CS12 CS11 CS10
typedef enum
{
    CS10 = 0,  // Bit 0 – CS10: Clock Select
    CS11,      // Bit 1 – CS11: Clock Select
    CS12,      // Bit 2 – CS12: Clock Select
    WGM12,     // Bit 3 – WGM12: Waveform Generation Mode
    WGM13,     // Bit 4 – WGM13: Waveform Generation Mode
    ICES1 = 6, // Bit 6 – ICES1: Input Capture Edge Select
    ICNC1 = 7  // Bit 7 – ICNC1: Input Capture Noise Canceler
} TIMER1_TCCR1B_PINs_E;
/******************************************************************************
 *									APIS				                      *
 *******************************************************************************/

E_STATUS_t TIMER0_Init(Timer_Config_t *configuartion);
E_STATUS_t TIMER0_Stop();
E_STATUS_t TIMER0_Start();

E_STATUS_t TIMER0_GetCompare(unsigned char *TicksNumber);
E_STATUS_t TIMER0_SetCompare(uint8_t uint8_t_TicksNumber);

E_STATUS_t TIMER0_GetCounter(unsigned char *TicksNumber);
E_STATUS_t TIMER0_SetCounter(uint8_t uint8_t_TicksNumber);

E_STATUS_t TIMER0_GetOverflow(unsigned char *TicksNumber);
E_STATUS_t TIMER0_SetOverflow(uint64_t uint8_t_TicksNumber);

void TIMER0_CALLBACK_Overflow_INTERRUPT(Ptr_Func callback);

void TIMER0_CALLBACK_CompareMatch_INTERRUPT(Ptr_Func callback);

void TIMER0_Init_PWM(Timer_Config_t *TIM0_Config);
void PWM0_SetDutyCycleValue(uint8_t DutyCycle);

//-----------
// TIMER1 APIs
//-----------
E_STATUS_t TIMER1_Init(TIMER1_Config_t *TIM1_Config);

// Function to stop Timer1
E_STATUS_t TIMER1_Stop(void);
E_STATUS_t TIMER1_Start();
// Function to set the compare value of Timer1
E_STATUS_t TIMER1_SetICR1Value(uint16_t uint16_t_ticks);
E_STATUS_t TIMER1_SetCompareValue(uint16_t uint16_t_ticks, uint8_t CHANNEL);
// Function to get the current counter value of Timer1
E_STATUS_t TIMER1_GetCounterValue(uint16_t *ptr_uint16_t_ticks);
// Function to set the counter value of Timer1
E_STATUS_t TIMER1_SetCounterValue(uint16_t uint16_t_ticks);
// Function to get the overflow counter value of Timer1
E_STATUS_t TIMER1_GetOverflowValue(uint16_t *ptr_uint16_t_ticks);
// Function to set the overflow counter value of Timer1
E_STATUS_t TIMER1_SetOverflowValue(uint16_t uint16_t_ticks);
// Function to set the overflow interrupt callback function for Timer1
void TIMER1_SetCallBack_Overflow_Interrupt(Ptr_Func CallBack);
// Function to set the compare match interrupt callback function for Timer1
E_STATUS_t TIMER1_SetCallBack_CompareMatch_Interrupt(Ptr_Func CallBack, uint8_t CHANNEL);

// calculate OCR1A/B from the Duty Cycle
E_STATUS_t PWM1_SetDutyCycleAndICR1Value(uint8_t DutyCycle, uint8_t CHANNEL, uint16_t TOP);

// Function to initialize PWM settings for Timer1 (internal use)
void TIMER1_InitPWM();

void WDT_ON(uint8_t time);
void WDT_OFF(void);

void delay_us(uint64_t x);
void delay_ms(uint64_t x);

#endif