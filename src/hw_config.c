/********************************************************************************/
/*!
	@file			hw_config.c
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "hw_config.h"

/* Defines -------------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/;

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/**************************************************************************/
/*! 
    @brief	Configures Main system clocks & power.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void Set_System(void)
{
	/* SystemInit was already executed in asm startup */

	/* NVIC configuration */
	NVIC_Configuration();

}

/**************************************************************************/
/*! 
    @brief	Configures Vector Table base location.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_SetPriority(SysTick_IRQn, 0x0);	
}

/* End Of File ---------------------------------------------------------------*/
