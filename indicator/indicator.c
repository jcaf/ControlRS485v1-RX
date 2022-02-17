/*
 * buzzer.c
 *
 *  Created on: Aug 30, 2021
 *      Author: jcaf
 */
#include "indicator.h"

#include "../main.h"

struct _indicator indicator;
void indicator_setPortPin(volatile unsigned char *Port8bits, int8_t pin)
{
	indicator.Port8bits = Port8bits;
	indicator.pin = pin;
}
#ifdef TIMEON_TIMEOFF_SEPARADOS
void indicator_setKSysTickTimeOn_ms(uint16_t KSysTickTimeOn_ms)///SYSTICK_MS
{
	indicator.KOn_MAX = KSysTickTimeOn_ms;
}
void indicator_setKSysTickTimeOff_ms(uint16_t KSysTickTimeOff_ms)///SYSTICK_MS
{
	indicator.KOff_MAX = KSysTickTimeOff_ms;
}
#else
void indicator_setKSysTickTime_ms(uint16_t KSysTickTime_ms)// div by SYSTICK_MS
{
	indicator.KOn_MAX = KSysTickTime_ms;
}
#endif

void indicator_On(void)
{
	indicator.sm0 = 1;
}
void indicator_Off(void)
{
	PinTo0(*indicator.Port8bits, indicator.pin);
	indicator.sm0 = 0;
}
//
void indicator_cycle_start(void)
{
	indicator.sm0 = 3;
}
void indicator_job(void)
{
	//1 ON, run and stop automatically
	if (indicator.sm0 == 1)
	{
		PinTo1(*indicator.Port8bits, indicator.pin);
		indicator.counter0 = 0;
		indicator.sm0++;
	}
	else if (indicator.sm0 == 2)
	{
		if (mainflag.sysTickMs)
		{
			if (++indicator.counter0 >= indicator.KOn_MAX)
			{
				indicator_Off();
			}
		}
	}
	//Cycle, stop by user
	if (indicator.sm0 == 3)
	{
		PinToggle(*indicator.Port8bits, indicator.pin);
		indicator.counter0 = 0;
		indicator.sm0++;
	}
	else if (indicator.sm0 == 4)
	{
		if (mainflag.sysTickMs)
		{
			if (++indicator.counter0 >= indicator.KOn_MAX)
			{
				indicator.sm0--;
			}
		}
	}

}
