/********************************************************************************/
/*!
	@file			stm32f4xx_it.c
	@author         Nemui Trinomius (http://nemuisan.blog.bai.ne.jp)
    @version        1.00
    @date           2011.10.16
	@brief          Cortex-M4 Processor Exceptions Handlers.				@n
					And STM32F4xx Peripherals Interrupt Handlers.			@n
					Device Dependent Section.

    @section HISTORY
		2011.10.16	V1.00	Start Here.

    @section LICENSE
		BSD License. See Copyright.txt
*/
/********************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "hw_config.h"
#include "FreeRTOS.h"
#include "task.h"

/* Defines -------------------------------------------------------------------*/#define portNVIC_SYSTICK_CTRL		( ( volatile unsigned long * ) 0xe000e010 )
#define portNVIC_SYSTICK_LOAD		( ( volatile unsigned long * ) 0xe000e014 )
#define portNVIC_INT_CTRL			( ( volatile unsigned long * ) 0xe000ed04 )
#define portNVIC_SYSPRI2			( ( volatile unsigned long * ) 0xe000ed20 )
#define portNVIC_SYSTICK_CLK		0x00000004
#define portNVIC_SYSTICK_INT		0x00000002
#define portNVIC_SYSTICK_ENABLE		0x00000001
#define portNVIC_PENDSVSET			0x10000000
#define portNVIC_PENDSV_PRI			( ( ( unsigned long ) configKERNEL_INTERRUPT_PRIORITY ) << 16 )
#define portNVIC_SYSTICK_PRI		( ( ( unsigned long ) configKERNEL_INTERRUPT_PRIORITY ) << 24 )


/* Variables -----------------------------------------------------------------*/

/* Constants -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**************************************************************************/
/*! 
    @brief	Handles NMI exception.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void NMI_Handler(void)
{
}


/**************************************************************************/
/*! 
    @brief	Handles Hard Fault exception.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}


/**************************************************************************/
/*! 
    @brief	Handles Memory Manage exception.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}


/**************************************************************************/
/*! 
    @brief	Handles Bus Fault exception.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}


/**************************************************************************/
/*! 
    @brief	Handles Usage Fault exception.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}


/**************************************************************************/
/*! 
    @brief	Handles SVCall exception.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void SVC_Handler(void)
{
}


/**************************************************************************/
/*! 
    @brief	Handles Debug Monitor exception.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void DebugMon_Handler(void)
{
}


/**************************************************************************/
/*! 
    @brief	Handles PendSVC exception.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void PendSV_Handler(void)
{
}


/**************************************************************************/
/*! 
    @brief	Handles SysTick Handler.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
void SysTick_Handler(void)
{
//	static uint8_t  cntdiskio=0;
	unsigned long ulDummy;
	
	#if configUSE_PREEMPTION == 1
		*(portNVIC_INT_CTRL) = portNVIC_PENDSVSET;	
	#endif

	ulDummy = portSET_INTERRUPT_MASK_FROM_ISR();
	{
		vTaskIncrementTick();
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR( ulDummy );

	 /* every 10mSec */
/*
	if ( cntdiskio++ >= 10 ) {
		cntdiskio = 0;
		disk_timerproc();
	}
*/
	/* every 1mSec */
	TimingDelay_Decrement();
}
/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32F4xx_xx.s).                                            */
/******************************************************************************/
#if (IRQ_WWDG_ENABLE==1)
void WWDG_IRQHandler(void)
{
}
#endif

#if (IRQ_PVD_ENABLE==1)
void PVD_IRQHandler(void)
{
}
#endif

#if (IRQ_TAMPER_ENABLE==1)
void TAMPER_IRQHandler(void)
{
}
#endif


#if (IRQ_FLASH_ENABLE==1)
void FLASH_IRQHandler(void)
{
}
#endif

#if (IRQ_RCC_ENABLE==1)
void RCC_IRQHandler(void)
{
}
#endif

#if (IRQ_EXTI0_ENABLE==1)
void EXTI0_IRQHandler(void)
{
}
#endif

#if (IRQ_EXTI1_ENABLE==1)
void EXTI1_IRQHandler(void)
{
}
#endif

#if (IRQ_EXTI2_ENABLE==1)
void EXTI2_IRQHandler(void)
{
}
#endif

#if (IRQ_EXTI3_ENABLE==1)
void EXTI3_IRQHandler(void)
{
}
#endif

#if (IRQ_EXTI4_ENABLE==1)
void EXTI4_IRQHandler(void)
{
}
#endif

#if (IRQ_DMA1_Channel1_ENABLE==1)
void DMA1_Channel1_IRQHandler(void)
{
}
#endif

#if (IRQ_DMA1_Channel2_ENABLE==1)
void DMA1_Channel2_IRQHandler(void)
{
}
#endif

#if (IRQ_DMA1_Channel3_ENABLE==1)
void DMA1_Channel3_IRQHandler(void)
{
}
#endif

#if (IRQ_DMA1_Channel4_ENABLE==1)
void DMA1_Channel4_IRQHandler(void)
{
}
#endif

#if (IRQ_DMA1_Channel5_ENABLE==1)
void DMA1_Channel5_IRQHandler(void)
{
}
#endif

#if (IRQ_DMA1_Channel6_ENABLE==1)
void DMA1_Channel6_IRQHandler(void)
{
}
#endif

#if (IRQ_DMA1_Channel7_ENABLE==1)
void DMA1_Channel7_IRQHandler(void)
{
}
#endif

#if (IRQ_ADC1_2_ENABLE==1)
void ADC1_2_IRQHandler(void)
{
}
#endif

#if (IRQ_USB_HP_CAN_TX_ENABLE==1)
void USB_HP_CAN_TX_IRQHandler(void)
{
}
#endif

#if (IRQ_USB_LP_CAN_RX0_ENABLE==1)
void USB_LP_CAN_RX0_IRQHandler(void)
{
}
#endif

#if (IRQ_CAN_RX1_ENABLE==1)
void CAN_RX1_IRQHandler(void)
{
}
#endif

#if (IRQ_CAN_SCE_ENABLE==1)
void CAN_SCE_IRQHandler(void)
{
}
#endif

#if (IRQ_EXTI9_5_ENABLE==1)
void EXTI9_5_IRQHandler(void)
{
}
#endif

#if (IRQ_TIM1_BRK_ENABLE==1)
void TIM1_BRK_IRQHandler(void)
{
}
#endif

#if (IRQ_TIM1_UP_ENABLE==1)
void TIM1_UP_IRQHandler(void)
{
}
#endif

#if (IRQ_TIM1_TRG_COM_ENABLE==1)
void TIM1_TRG_COM_IRQHandler(void)
{
}
#endif

#if (IRQ_TIM1_CC_ENABLE==1)
void TIM1_CC_IRQHandler(void)
{
}
#endif

#if (IRQ_TIM2_ENABLE==1)
void TIM2_IRQHandler(void)
{
}
#endif

#if (IRQ_TIM3_ENABLE==1)
void TIM3_IRQHandler(void)
{
}
#endif

#if (IRQ_TIM4_ENABLE==1)
void TIM4_IRQHandler(void)
{
}
#endif


#if (IRQ_I2C1_EV_ENABLE==1)
void I2C1_EV_IRQHandler(void)
{
}
#endif

#if (IRQ_I2C1_ER_ENABLE==1)
void I2C1_ER_IRQHandler(void)
{
}
#endif

#if (IRQ_I2C2_EV_ENABLE==1)
void I2C2_EV_IRQHandler(void)
{
//  fmI2C_IRQ_EV();
}
#endif

#if (IRQ_I2C2_ER_ENABLE==1)
void I2C2_ER_IRQHandler(void)
{
//  fmI2C_IRQ_ER();
}
#endif

#if (IRQ_SPI1_ENABLE==1)
void SPI1_IRQHandler(void)
{
}
#endif

#if (IRQ_SPI2_ENABLE==1)
void SPI2_IRQHandler(void)
{
}
#endif

#if (IRQ_USART3_ENABLE==1)
void USART3_IRQHandler(void)
{

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		/* Advance buffer head. */
		uint16_t tempRX_Head = ((&USART3_Buf)->RX_Head + 1) & (UART_BUFSIZE-1);

		/* Check for overflow. */
		uint16_t tempRX_Tail = (&USART3_Buf)->RX_Tail;
		uint8_t data =  USART_ReceiveData(USART3);

		if (tempRX_Head == tempRX_Tail) {
			/* Disable the USART3 Receive interrupt */
			USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
		}else{
			(&USART3_Buf)->RX[(&USART3_Buf)->RX_Head] = data;
			(&USART3_Buf)->RX_Head = tempRX_Head;
		}
	
	}

	if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
	{   
		/* Check if all data is transmitted. */
		uint16_t tempTX_Tail = (&USART3_Buf)->TX_Tail;
		if ((&USART3_Buf)->TX_Head == tempTX_Tail){
			/* Overflow MAX size Situation */
			/* Disable the USART3 Transmit interrupt */
			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
		}else{
			/* Start transmitting. */
			uint8_t data = (&USART3_Buf)->TX[(&USART3_Buf)->TX_Tail];
			USART3->DR = data;

			/* Advance buffer tail. */
			(&USART3_Buf)->TX_Tail = ((&USART3_Buf)->TX_Tail + 1) & (UART_BUFSIZE-1);
		}

	}
}
#endif

#if (IRQ_EXTI15_10_ENABLE==1)
void EXTI15_10_IRQHandler(void)
{
}
#endif

#if (IRQ_RTCAlarm_ENABLE==1)
void RTCAlarm_IRQHandler(void)
{
}
#endif

#if (IRQ_USBWakeUp_ENABLE==1)
void USBWakeUp_IRQHandler(void)
{
}
#endif

#if (IRQ_TIM8_BRK_ENABLE==1)
void TIM8_BRK_IRQHandler(void)
{
}
#endif

#if (IRQ_TIM8_UP_ENABLE==1)
void TIM8_UP_IRQHandler(void)
{
}
#endif

#if (IRQ_TIM8_TRG_COM_ENABLE==1)
void TIM8_TRG_COM_IRQHandler(void)
{
}
#endif

#if (IRQ_TIM8_CC_ENABLE==1)
void TIM8_CC_IRQHandler(void)
{
}
#endif

#if (IRQ_ADC3_ENABLE==1)
void ADC3_IRQHandler(void)
{
}
#endif

#if (IRQ_FSMC_ENABLE==1)
void FSMC_IRQHandler(void)
{
}
#endif

#if (IRQ_SDIO_ENABLE==1)
void SDIO_IRQHandler(void)
{
}
#endif

#if (IRQ_TIM5_ENABLE==1)
void TIM5_IRQHandler(void)
{
}
#endif

#if (IRQ_SPI3_ENABLE==1)
void SPI3_IRQHandler(void)
{
}
#endif

#if (IRQ_UART4_ENABLE==1)
void UART4_IRQHandler(void)
{

	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		/* Advance buffer head. */
		uint16_t tempRX_Head = ((&UART4_Buf)->RX_Head + 1) & (UART_BUFSIZE-1);

		/* Check for overflow. */
		uint16_t tempRX_Tail = (&UART4_Buf)->RX_Tail;
		uint8_t data =  USART_ReceiveData(UART4);

		if (tempRX_Head == tempRX_Tail) {
			/* Disable the UART4 Receive interrupt */
			USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
		}else{
			(&UART4_Buf)->RX[(&UART4_Buf)->RX_Head] = data;
			(&UART4_Buf)->RX_Head = tempRX_Head;
		}
	
	}

	if(USART_GetITStatus(UART4, USART_IT_TXE) != RESET)
	{   
		/* Check if all data is transmitted. */
		uint16_t tempTX_Tail = (&UART4_Buf)->TX_Tail;
		if ((&UART4_Buf)->TX_Head == tempTX_Tail){
			/* Overflow MAX size Situation */
			/* Disable the UART4 Transmit interrupt */
			USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
		}else{
			/* Start transmitting. */
			uint8_t data = (&UART4_Buf)->TX[(&UART4_Buf)->TX_Tail];
			UART4->DR = data;

			/* Advance buffer tail. */
			(&UART4_Buf)->TX_Tail = ((&UART4_Buf)->TX_Tail + 1) & (UART_BUFSIZE-1);
		}

	}
}
#endif

#if (IRQ_UART5_ENABLE==1)
void UART5_IRQHandler(void)
{
}
#endif

#if (IRQ_TIM6_ENABLE==1)
void TIM6_IRQHandler(void)
{
}
#endif

#if (IRQ_TIM7_ENABLE==1)
void TIM7_IRQHandler(void)
{
}
#endif

#if (IRQ_DMA2_Channel1_ENABLE==1)
void DMA2_Channel1_IRQHandler(void)
{
}
#endif

#if (IRQ_DMA2_Channel2_ENABLE==1)
void DMA2_Channel2_IRQHandler(void)
{
}
#endif

#if (IRQ_DMA2_Channel3_ENABLE==1)
void DMA2_Channel3_IRQHandler(void)
{
}
#endif

#if (IRQ_DMA2_Channel4_5_ENABLE==1)
void DMA2_Channel4_5_IRQHandler(void)
{
}
#endif




/**************************************************************************/
/*! 
    @brief	Handles RealTimeClock interrupts requests.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
#if (IRQ_RTC_ENABLE==1)
void RTC_IRQHandler(void)
{

}
#endif


/**************************************************************************/
/*! 
    @brief	Handles USART2 global interrupt request.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
#if (IRQ_USART2_ENABLE==1)
void USART2_IRQHandler(void)
{

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		/* Advance buffer head. */
		uint16_t tempRX_Head = ((&USART2_Buf)->RX_Head + 1) & (UART_BUFSIZE-1);

		/* Check for overflow. */
		uint16_t tempRX_Tail = (&USART2_Buf)->RX_Tail;
		uint8_t data =  USART_ReceiveData(USART2);

		if (tempRX_Head == tempRX_Tail) {
			/* Disable the USART2 Receive interrupt */
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
		}else{
			(&USART2_Buf)->RX[(&USART2_Buf)->RX_Head] = data;
			(&USART2_Buf)->RX_Head = tempRX_Head;
		}
	
	}

	if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
	{   
		/* Check if all data is transmitted. */
		uint16_t tempTX_Tail = (&USART2_Buf)->TX_Tail;
		if ((&USART2_Buf)->TX_Head == tempTX_Tail){
			/* Overflow MAX size Situation */
			/* Disable the USART2 Transmit interrupt */
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		}else{
			/* Start transmitting. */
			uint8_t data = (&USART2_Buf)->TX[(&USART2_Buf)->TX_Tail];
			USART2->DR = data;

			/* Advance buffer tail. */
			(&USART2_Buf)->TX_Tail = ((&USART2_Buf)->TX_Tail + 1) & (UART_BUFSIZE-1);
		}

	}
}
#endif
/**************************************************************************/
/*! 
    @brief	Handles USART1 global interrupt request.
	@param	None.
    @retval	None.
*/
/**************************************************************************/
#if (IRQ_USART1_ENABLE==1)
void USART1_IRQHandler(void)
{

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		/* Advance buffer head. */
		uint16_t tempRX_Head = ((&USART1_Buf)->RX_Head + 1) & (UART_BUFSIZE-1);

		/* Check for overflow. */
		uint16_t tempRX_Tail = (&USART1_Buf)->RX_Tail;
		uint8_t data =  USART_ReceiveData(USART1);

		if (tempRX_Head == tempRX_Tail) {
			/* Disable the USART1 Receive interrupt */
			USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		}else{
			(&USART1_Buf)->RX[(&USART1_Buf)->RX_Head] = data;
			(&USART1_Buf)->RX_Head = tempRX_Head;
		}
	
	}

	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{   
		/* Check if all data is transmitted. */
		uint16_t tempTX_Tail = (&USART1_Buf)->TX_Tail;
		if ((&USART1_Buf)->TX_Head == tempTX_Tail){
			/* Overflow MAX size Situation */
			/* Disable the USART1 Transmit interrupt */
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		}else{
			/* Start transmitting. */
			uint8_t data = (&USART1_Buf)->TX[(&USART1_Buf)->TX_Tail];
			USART1->DR = data;

			/* Advance buffer tail. */
			(&USART1_Buf)->TX_Tail = ((&USART1_Buf)->TX_Tail + 1) & (UART_BUFSIZE-1);
		}

	}
}
#endif

/**
  * @brief  This function handles DMA Stream interrupt request.
  * @param  None
  * @retval None
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* Test on DMA Stream Transfer Complete interrupt */
  if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
  {
    /* Clear DMA Stream Transfer Complete interrupt pending bit */
    DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);  
    
  }
}

void DMA1_Stream0_IRQHandler(void)
{
  /* Test on DMA Stream Transfer Complete interrupt */
  if(DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF0))
  {
    /* Clear DMA Stream Transfer Complete interrupt pending bit */
    DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF0);  
    
  }
}

/**************************************************************************/
/*! 
    @brief	Handles PPP interrupt request.								@n
			Function Templates
	@param	None.
    @retval	None.
*/
/**************************************************************************/
/*void PPP_IRQHandler(void)
{
}*/


/* End Of File ---------------------------------------------------------------*/
