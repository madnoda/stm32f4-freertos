/********************************************************************************/
/*!
	@file			systick.h
	@author         Nemui Trinomius (http://nemuisan.blog.bai.ne.jp)
    @version        1.00
    @date           2011.06.12
	@brief          Based on ST Microelectronics's Sample Thanks!

    @section HISTORY
		2011.06.12	V1.00	Start Here.

    @section LICENSE
		BSD License. See Copyright.txt
*/
/********************************************************************************/
#ifndef __SYSTICK_H
#define __SYSTICK_H 0x0300

#ifdef __cplusplus
 extern "C" {
#endif

/* General Inclusion */
#include "stm32f4xx_it.h"
#include "stm32f4xx_conf.h"
#include "platform_config.h"

/* Systick Interval MilliSecond order */
#define INTERVAL		1000UL
/* Timer Interval MicroSecond order */
#define USEC_INTERVAL 	1000000UL

/* Inline Static Function */
extern __IO uint32_t TimingDelay;
static inline void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

/* Externals */
extern void _delay_ms(__IO uint32_t mSec);
extern void _delay_us(__IO uint32_t uSec);
extern void Delay(__IO uint32_t uCount);
extern void SysTickInit(__IO uint32_t interval);

#ifdef __cplusplus
}
#endif

#endif	/* __SYSTICK_H */
