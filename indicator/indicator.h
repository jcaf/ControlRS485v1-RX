/*
 * indicator.h
 *
 *  Created on: Aug 30, 2021
 *      Author: jcaf
 */

#ifndef INDICATOR_INDICATOR_H_
#define INDICATOR_INDICATOR_H_
#include "system.h"
struct _indicator
{
		int8_t sm0;
		uint16_t counter0;
		//
		uint16_t KOn_MAX;
		uint16_t KOff_MAX;
		//

		volatile unsigned char *Port8bits;
		int8_t pin;
};

extern struct _indicator indicator;

	void indicator_setPortPin(volatile unsigned char *Port8bits, int8_t pin);
	#ifdef TIMEON_TIMEOFF_SEPARADOS
	void indicator_setKSysTickTimeOn_ms(uint16_t KSysTickTimeOn_ms);//div by SYSTICK_MS
	void indicator_setKSysTickTimeOff_ms(uint16_t KSysTickTimeOff_ms);//div by SYSTICK_MS
	#else
	void indicator_setKSysTickTime_ms(uint16_t KSysTickTime_ms);//div by SYSTICK_MS
	#endif

	void indicator_On(void);
	void indicator_Off(void);
	void indicator_cycle_start(void);
	void indicator_job(void);


#endif /* INDICATOR_INDICATOR_H_ */
