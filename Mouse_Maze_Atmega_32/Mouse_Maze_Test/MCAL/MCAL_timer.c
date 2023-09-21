
/******************************************************************************
 *                              INCLUDES                                       *
 *******************************************************************************/
#include "MCAL_timer.h"
#include "MCAL_interrupt.h"

#define NULL (void *)0
#define Motor_A 2
#define Motor_B 3
/******************************************************************************
 *                               GLOBAL & EXTERN VARIABLES                     *
 *******************************************************************************/
Ptr_Func CALLBACK_OVER_FLOW = NULL;
Ptr_Func CALLBACK_COMPARE;
uint8_t Over_Flow_Number = 0, PWM_Over_Flow_Number = 0;
uint8_t CMP_Number = 0, PWM_Compare_Number = 0;
Timer_Config_t *Global_configuartion;
uint8_t TIMER0_u8_OVF_Number = 0;
uint8_t TIMER0_u8_CTC_Number = 0;
/******************************************************************************
 *								 APIS IMPLEMENTATION			              *
 *******************************************************************************/

E_STATUS_t TIMER0_Init(Timer_Config_t *configuartion)
{
	E_STATUS_t Value = E_OK;
	TCNT0 = 0; // Clear Timer0
	if (NULL == configuartion)
	{
		Value = E_NOK;
	}
	else
	{
		Global_configuartion = configuartion;
		if ((Global_configuartion->mode == Normal) || (Global_configuartion->mode == CTC))
		{
			TCCR0 |= Global_configuartion->mode;
		}
		else
		{
			Value = E_NOK;
		}
		if ((Global_configuartion->CLK >= NO_CLK_SRC) && (Global_configuartion->CLK <= EXT_CLK_RISING))
		{
			TCCR0 |= Global_configuartion->CLK;
		}
		else
		{
			Value = E_NOK;
		}
		if ((Global_configuartion->Compare == OCIE_ENABLE))
		{
			TIMSK |= Global_configuartion->Compare;
		}
		else
		{
			CLR_BIT(TIMSK, OCIE_DISABLE);
			Value = E_NOK;
		}
		if ((Global_configuartion->Overflow == TOIE_ENABLE))
		{
			TIMSK |= Global_configuartion->Overflow;
		}
		else
		{
			CLR_BIT(TIMSK, TOIE_DISABLE);
		}

		// Initialize PWM if applicable
		if ((Global_configuartion->PWM0_MODE == Phase_Correct_PWM) || (Global_configuartion->PWM0_MODE == Fast_PWM))
		{
			TIMER0_Init_PWM(Global_configuartion);
		}
		else
		{
			Value = E_NOK;
		}
	}

	return Value;
}

E_STATUS_t TIMER0_Start()
{
	E_STATUS_t Value = E_OK;
	TCCR0 |= Global_configuartion->CLK;
	return Value;
}

E_STATUS_t TIMER0_Stop()
{
	E_STATUS_t Value = E_OK;
	TCCR0 &= ~(1 << CS02 | 1 << CS01 | 1 << CS00);
	return Value;
}

/*Does the TCNT0 counter start from TCNT0's value and count up to the overflow value?
Yes.
Does the Timer/Counter 0 count from 0 to OCR0 value in CTC mode?
Yes.*/

E_STATUS_t TIMER0_GetCompare(unsigned char *TicksNumber)
{
	/*The Output Compare Register contains an 8-bit value that is continuously compared with the
	counter value (TCNT0). A match can be used to generate an output compare interrupt, or to
	generate a waveform output on the OC0 pin*/
	E_STATUS_t Value = E_OK;
	*TicksNumber = OCR0;
	return Value;
}

E_STATUS_t TIMER0_SetCompare(uint8_t uint8_t_TicksNumber)
{
	E_STATUS_t Value = E_OK;
	GIE_disable();
	OCR0 = uint8_t_TicksNumber;
    GIE_enable();
	return Value;
}

E_STATUS_t TIMER0_GetCounter(unsigned char *TicksNumber)
{
	E_STATUS_t Value = E_OK;
	*TicksNumber = TCNT0;
	return Value;
}
E_STATUS_t TIMER0_SetCounter(uint8_t uint8_t_TicksNumber)
{
	E_STATUS_t Value = E_OK;
	GIE_disable();
	TCNT0 = uint8_t_TicksNumber;
    GIE_enable();
	return Value;
}

E_STATUS_t TIMER0_GetOverflow(unsigned char *TicksNumber)
{
	E_STATUS_t Value = E_OK;
	*TicksNumber = Over_Flow_Number;
	return Value;
}

E_STATUS_t TIMER0_SetOverflow(uint64_t uint8_t_TicksNumber)
{
	E_STATUS_t Value = E_OK;
	GIE_disable();
	Over_Flow_Number = uint8_t_TicksNumber;
    GIE_enable();
	return Value;
}

void TIMER0_CALLBACK_Overflow_INTERRUPT(Ptr_Func callback)
{
	CALLBACK_OVER_FLOW = callback;
}

void TIMER0_CALLBACK_CompareMatch_INTERRUPT(Ptr_Func callback)
{
	CALLBACK_COMPARE = callback;
}

// Initialize Timer0 PWM settings
void TIMER0_Init_PWM(Timer_Config_t *TIM0_Config)
{
	// Set Fast PWM mode
	if (TIM0_Config->PWM0_MODE == Fast_PWM)
	{
		// Set WGM00 and WGM01 bits to enable Fast PWM mode
		SET_BIT(TCCR0, WGM00);
		SET_BIT(TCCR0, WGM01);
	}
	// Set Phase Correct PWM mode
	else if (TIM0_Config->PWM0_MODE == Phase_Correct_PWM)
	{
		// Clear WGM00 and set WGM01 bit to enable Phase Correct PWM mode
		CLR_BIT(TCCR0, WGM00);
		SET_BIT(TCCR0, WGM01);
	}

	// Set PWM output state
	if (TIM0_Config->PWM0_STATE == PWM0_STATE_NON_INVERTING)
	{
		SET_BIT(TCCR0, COM00);
		CLR_BIT(TCCR0, COM01);
	}
	else if (TIM0_Config->PWM0_STATE == PWM0_STATE_INVERTING)
	{
		SET_BIT(TCCR0, COM01);
		SET_BIT(TCCR0, COM00);
	}
}

void PWM0_SetDutyCycleValue(uint8_t DutyCycle)
{
	uint8_t Compare_Value = (DutyCycle / 100.0) * 255;
	TIMER0_SetCompare(Compare_Value);
}

//-------
// TIMER1
//-------
// Callback function pointers
Ptr_Func TIMER1_CaLLBack_OVF = NULL;
Ptr_Func TIMER1_CaLLBack_CM[2] = {NULL, NULL};

// Generic Variables
TIMER1_Config_t G_TIMER1_Config;
uint16_t TIMER1_uint16_t_OVF_Number;
uint16_t TIMER1_uint16_t_CTC_Number[2];

//---------------------------------------------------------------
//---------------------     TIMER1 APIs   -----------------------
//---------------------------------------------------------------

E_STATUS_t TIMER1_Init(TIMER1_Config_t *TIM1_Config)
{
	// Variables
	E_STATUS_t RetValue = E_OK;
	uint8_t Temp_TCCR1A = 0, Temp_TCCR1B = 0;

	// Initialize the counter (TCNT1)
	TCNT1 = 0;
	// Copy the configuration
	G_TIMER1_Config = *TIM1_Config;

	// Configure Timer mode
	// for bits(WGM13,WGM12,WGM11,WGM10)
	// WGM11,WGM10 @ TCCR1A bit(1,0)
	Temp_TCCR1A |= ((G_TIMER1_Config.MODE) & 0x03);
	// WGM13,WGM12 @ TCCR1B bit(4,3)
	Temp_TCCR1B |= ((((G_TIMER1_Config.MODE) & 0x0C) >> 2) << 3);

	// Configure Prescaler
	if ((G_TIMER1_Config.PRESCALER_CLK >= NO_CLK_SRC) && (G_TIMER1_Config.PRESCALER_CLK <= EXT_CLK_RISING))
	{
		Temp_TCCR1B |= G_TIMER1_Config.PRESCALER_CLK;
	}
	else
	{
		RetValue = E_NOK;
	}

	// Bit 2 – TOIE1: Timer/Counter1, Overflow Interrupt Enable
	if (TOVIE1_Disable == G_TIMER1_Config.OVF_Interrupt)
	{
		TIMSK &= TOVIE1_Disable;
	}
	else if (TOVIE1_Enable == G_TIMER1_Config.OVF_Interrupt)
	{
		TIMSK |= TOVIE1_Enable;
	}
	else
	{
		RetValue = E_NOK;
	}

	if (TIMER1_SELECT_CHANNEL_A == G_TIMER1_Config.SELECT_CHANNEL)
	{
		// Configure Output Compare A Match Interrupt Enable
		// TIMSK: Bit 4 – OCIE1A: Timer/Counter1
		if ((G_TIMER1_Config.OCAM_Interrupt == OCMIE1A_Enable))
		{
			TIMSK |= G_TIMER1_Config.OCAM_Interrupt;
		}
		else if (G_TIMER1_Config.OCAM_Interrupt == OCMIE1A_Disable)
		{
			TIMSK &= G_TIMER1_Config.OCAM_Interrupt;
		}
		else
		{
			RetValue = E_NOK;
		}
	}
	else if (TIMER1_SELECT_CHANNEL_B == G_TIMER1_Config.SELECT_CHANNEL)
	{

		// Configure  Output Compare B Match Interrupt Enable
		// TIMSK: Bit 3 – OCIE1B: Timer/Counter1
		if ((G_TIMER1_Config.OCBM_Interrupt == OCMIE1B_Enable))
		{
			TIMSK |= G_TIMER1_Config.OCBM_Interrupt;
		}
		else if (G_TIMER1_Config.OCBM_Interrupt == OCMIE1B_Disable)
		{
			TIMSK &= G_TIMER1_Config.OCBM_Interrupt;
		}
		else
		{
			RetValue = E_NOK;
		}
	}
	else if (TIMER1_SELECT_CHANNEL_A_B == G_TIMER1_Config.SELECT_CHANNEL)
	{
		// Configure Output Compare A Match Interrupt Enable
		// TIMSK: Bit 4 – OCIE1A: Timer/Counter1
		if ((G_TIMER1_Config.OCAM_Interrupt == OCMIE1A_Enable))
		{
			TIMSK |= G_TIMER1_Config.OCAM_Interrupt;
		}
		else if (G_TIMER1_Config.OCAM_Interrupt == OCMIE1A_Disable)
		{
			TIMSK &= G_TIMER1_Config.OCAM_Interrupt;
		}
		else
		{
			RetValue = E_NOK;
		}

		// Configure  Output Compare B Match Interrupt Enable
		// TIMSK: Bit 3 – OCIE1B: Timer/Counter1
		if ((G_TIMER1_Config.OCBM_Interrupt == OCMIE1B_Enable))
		{
			TIMSK |= G_TIMER1_Config.OCBM_Interrupt;
		}
		else if (G_TIMER1_Config.OCBM_Interrupt == OCMIE1B_Disable)
		{
			TIMSK &= G_TIMER1_Config.OCBM_Interrupt;
		}
		else
		{
			RetValue = E_NOK;
		}
	}
	else
	{
		RetValue = E_NOK;
	}

	// Function to initialize PWM settings for Timer1 (internal use) if applicable
	if ((G_TIMER1_Config.MODE != TIMER1_NORMAL_MODE) &&
		(G_TIMER1_Config.MODE != TIMER1_CTC_OCR1_MODE) &&
		(G_TIMER1_Config.MODE != TIMER1_CTC_ICR1_MODE))
	{
		Temp_TCCR1A |= G_TIMER1_Config.PWM1_STATE;
	}

	// Copy Temp_TCCR1A value to TCCR1A
	TCCR1A |= Temp_TCCR1A;
	// Copy Temp_TCCR1B value to TCCR1B
	TCCR1B |= Temp_TCCR1B;

	return RetValue;
}
E_STATUS_t TIMER1_Start(void)
{
	E_STATUS_t RetValue = E_OK;
	TCCR1B |= G_TIMER1_Config.PRESCALER_CLK;
	return RetValue;
}
// Function to stop Timer1
E_STATUS_t TIMER1_Stop(void)
{
	E_STATUS_t RetValue = E_OK;
	TCCR1B &= NO_CLK_SRC;
	return RetValue;
}
// Function to set the Input capture value of Timer1
E_STATUS_t TIMER1_SetICR1Value(uint16_t uint16_t_ticks)
{
	E_STATUS_t RetValue = E_OK;
	GIE_disable();
	ICR1 = uint16_t_ticks;
	GIE_enable();
	return RetValue;
}
// Function to set the compare value of Timer1
E_STATUS_t TIMER1_SetCompareValue(uint16_t uint16_t_ticks, uint8_t CHANNEL)
{
	E_STATUS_t RetValue = E_OK;
	GIE_disable();
	if (CHANNEL == TIMER1_SELECT_CHANNEL_A_B)
	{
		OCR1A = uint16_t_ticks;
		OCR1B = uint16_t_ticks;
	}
	else if (CHANNEL == TIMER1_SELECT_CHANNEL_A)
	{
		OCR1A = uint16_t_ticks;
		// OCR1AH = (uint8_t)((uint16_t_ticks & 0xFF00) >> 8);
		// OCR1AL = (uint8_t)(uint16_t_ticks  & 0x00FF);
	}
	else if (CHANNEL == TIMER1_SELECT_CHANNEL_B)
	{
		OCR1B = uint16_t_ticks;
		// OCR1BH = (uint8_t)((uint16_t_ticks & 0xFF00) >> 8);
		// OCR1BL = (uint8_t)(uint16_t_ticks  & 0x00FF);
	}
	else
	{
		RetValue = E_NOK;
	}
	GIE_enable();

	return RetValue;
}
// Function to get the current counter value of Timer1
E_STATUS_t TIMER1_GetCounterValue(uint16_t *ptr_uint16_t_ticks)
{
	E_STATUS_t RetValue = E_OK;

	*ptr_uint16_t_ticks = TCNT1;
	// 	*ptr_uint16_t_ticks =  (((uint16_t)TCNT1H) << 8);
	// 	*ptr_uint16_t_ticks |= ((uint16_t)TCNT1L);

	return RetValue;
}
// Function to set the counter value of Timer1
E_STATUS_t TIMER1_SetCounterValue(uint16_t uint16_t_ticks)
{
	E_STATUS_t RetValue = E_OK;

	// Reset encoder counts for the next control iteration
	// Disable interrupts while resetting to ensure atomicity
	GIE_disable();
	TCNT1 = uint16_t_ticks;
	GIE_enable();

	return RetValue;
}
// Function to get the overflow counter value of Timer1
E_STATUS_t TIMER1_GetOverflowValue(uint16_t *ptr_uint16_t_ticks)
{
	E_STATUS_t RetValue = E_OK;

	*ptr_uint16_t_ticks = TIMER1_uint16_t_OVF_Number;

	return RetValue;
}
// Function to set the overflow counter value of Timer1
E_STATUS_t TIMER1_SetOverflowValue(uint16_t uint16_t_ticks)
{
	E_STATUS_t RetValue = E_OK;
	GIE_disable();
	TIMER1_uint16_t_OVF_Number = uint16_t_ticks;
	GIE_enable();

	return RetValue;
}
// Function to set the overflow interrupt callback function for Timer1
void TIMER1_SetCallBack_Overflow_Interrupt(Ptr_Func CallBack)
{
	TIMER1_CaLLBack_OVF = CallBack;
}
// Function to set the compare match interrupt callback function for Timer1
E_STATUS_t TIMER1_SetCallBack_CompareMatch_Interrupt(Ptr_Func CallBack, uint8_t CHANNEL)
{
	E_STATUS_t RetValue = E_OK;
	TIMER1_CaLLBack_CM[CHANNEL] = CallBack;
	return RetValue;
}

// calculate OCR1AL/OCR1AH OR OCR1BL/OCR1BH from the Duty Cycle
E_STATUS_t PWM1_SetDutyCycleAndICR1Value(uint8_t DutyCycle, uint8_t CHANNEL, uint16_t TOP)
{
	E_STATUS_t RetValue = E_OK;

	uint16_t Compare_Value = (uint16_t)((DutyCycle / 100.0) * TOP);
	RetValue = TIMER1_SetCompareValue(Compare_Value, CHANNEL);

	return RetValue;
}

//==================================================================
//==================INTERRUPT VECTOR TABLE HANDLER==================
//==================================================================
// to be modified in the latest code

//-----------
// TIMER1 IVT
//-----------

// (7)TIMER1 COMPA Timer/Counter1 Compare Match A
// void __vector_7(void) __attribute__((signal));
// void __vector_7(void)
// {
// 	TIMER1_uint16_t_CTC_Number[TIMER1_SELECT_CHANNEL_A]++;

// 	if (TIMER1_CaLLBack_CM[TIMER1_SELECT_CHANNEL_A] != NULL)
// 	{
// 		TIMER1_CaLLBack_CM[TIMER1_SELECT_CHANNEL_A]();
// 	}
// }

// //	(8)TIMER1 COMPB Timer/Counter1 Compare Match B
// void __vector_8(void) __attribute__((signal));
// void __vector_8(void)
// {
// 	TIMER1_uint16_t_CTC_Number[TIMER1_SELECT_CHANNEL_B]++;

// 	if (TIMER1_CaLLBack_CM[TIMER1_SELECT_CHANNEL_B] != NULL)
// 	{
// 		TIMER1_CaLLBack_CM[TIMER1_SELECT_CHANNEL_B]();
// 	}
// }
// // (9)TIMER1 OVF Timer/Counter1 Overflow
// void __vector_9(void) __attribute__((signal));
// void __vector_9(void)
// {
// 	Over_Flow_Number++;

// 	if (TIMER1_CaLLBack_OVF != NULL)
// 	{
// 		TIMER1_CaLLBack_OVF();
// 	}
// }

// //-----------
// // TIMER0 IVT
//-----------

// void __vector_10(void) __attribute__((signal));
// void __vector_10(void)
// {
// 	TIMER0_u8_CTC_Number++;
// CALLBACK_COMPARE();
// }

/////////////////////////////////////
// we will only use  this  for now
/////////////////////////////////////

// void __vector_11(void) __attribute__((signal));
// void __vector_11(void)
// {
// 	TIMER0_u8_OVF_Number++;
// 	if (CALLBACK_OVER_FLOW != NULL)
// 		CALLBACK_OVER_FLOW();
// }

void WDT_OFF(void)
{
	WDTCR |= (1 << WDTOE) | (1 << WDE);
	WDTCR = 0x00;
}
// use WDT_TimeOut_t
void WDT_ON(uint8_t time)
{
	SET_BIT(WDTCR, time);
}
// Function to delay in microseconds
void delay_us(uint64_t us)
{
	us *= 8; // 8 cycles per microsecond for 8 MHz clock
	while (us--)
	{
		asm volatile("nop"); // No operation assembly instruction
	}
}
// Function to delay in milliseconds
void delay_ms(uint64_t ms)
{
	while (ms--)
	{
		delay_us(1000); // Delay for 1000 microseconds (1 ms)
	}
}
