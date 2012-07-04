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
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_it_setting.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/* ---------------------------------------------- */
/* --- see stm32f10x_it_setting.h */
#if (IRQ_WWDG_ENABLE==1)
void WWDG_IRQHandler(void);
#endif
#if (IRQ_PVD_ENABLE==1)
void PVD_IRQHandler(void);
#endif
#if (IRQ_TAMPER_ENABLE==1)
void TAMPER_IRQHandler(void);
#endif
#if (IRQ_RTC_ENABLE==1)
void RTC_IRQHandler(void);
#endif
#if (IRQ_FLASH_ENABLE==1)
void FLASH_IRQHandler(void);
#endif
#if (IRQ_RCC_ENABLE==1)
void RCC_IRQHandler(void);
#endif
#if (IRQ_EXTI0_ENABLE==1)
void EXTI0_IRQHandler(void);
#endif
#if (IRQ_EXTI1_ENABLE==1)
void EXTI1_IRQHandler(void);
#endif
#if (IRQ_EXTI2_ENABLE==1)
void EXTI2_IRQHandler(void);
#endif
#if (IRQ_EXTI3_ENABLE==1)
void EXTI3_IRQHandler(void);
#endif
#if (IRQ_EXTI4_ENABLE==1)
void EXTI4_IRQHandler(void);
#endif
#if (IRQ_DMA1_Channel1_ENABLE==1)
void DMA1_Channel1_IRQHandler(void);
#endif
#if (IRQ_DMA1_Channel2_ENABLE==1)
void DMA1_Channel2_IRQHandler(void);
#endif
#if (IRQ_DMA1_Channel3_ENABLE==1)
void DMA1_Channel3_IRQHandler(void);
#endif
#if (IRQ_DMA1_Channel4_ENABLE==1)
void DMA1_Channel4_IRQHandler(void);
#endif
#if (IRQ_DMA1_Channel5_ENABLE==1)
void DMA1_Channel5_IRQHandler(void);
#endif
#if (IRQ_DMA1_Channel6_ENABLE==1)
void DMA1_Channel6_IRQHandler(void);
#endif
#if (IRQ_DMA1_Channel7_ENABLE==1)
void DMA1_Channel7_IRQHandler(void);
#endif
#if (IRQ_ADC1_2_ENABLE==1)
void ADC1_2_IRQHandler(void);
#endif
#if (IRQ_USB_HP_CAN_TX_ENABLE==1)
void USB_HP_CAN_TX_IRQHandler(void);
#endif
#if (IRQ_USB_LP_CAN_RX0_ENABLE==1)
void USB_LP_CAN_RX0_IRQHandler(void);
#endif
#if (IRQ_CAN_RX1_ENABLE==1)
void CAN_RX1_IRQHandler(void);
#endif
#if (IRQ_CAN_SCE_ENABLE==1)
void CAN_SCE_IRQHandler(void);
#endif
#if (IRQ_EXTI9_5_ENABLE==1)
void EXTI9_5_IRQHandler(void);
#endif
#if (IRQ_TIM1_BRK_ENABLE==1)
void TIM1_BRK_IRQHandler(void);
#endif
#if (IRQ_TIM1_UP_ENABLE==1)
void TIM1_UP_IRQHandler(void);
#endif
#if (IRQ_TIM1_TRG_COM_ENABLE==1)
void TIM1_TRG_COM_IRQHandler(void);
#endif
#if (IRQ_TIM1_CC_ENABLE==1)
void TIM1_CC_IRQHandler(void);
#endif
#if (IRQ_TIM2_ENABLE==1)
void TIM2_IRQHandler(void);
#endif
#if (IRQ_TIM3_ENABLE==1)
void TIM3_IRQHandler(void);
#endif
#if (IRQ_TIM4_ENABLE==1)
void TIM4_IRQHandler(void);
#endif
#if (IRQ_I2C1_EV_ENABLE==1)
void I2C1_EV_IRQHandler(void);
#endif
#if (IRQ_I2C1_ER_ENABLE==1)
void I2C1_ER_IRQHandler(void);
#endif
#if (IRQ_I2C2_EV_ENABLE==1)
void I2C2_EV_IRQHandler(void);
#endif
#if (IRQ_I2C2_ER_ENABLE==1)
void I2C2_ER_IRQHandler(void);
#endif
#if (IRQ_SPI1_ENABLE==1)
void SPI1_IRQHandler(void);
#endif
#if (IRQ_SPI2_ENABLE==1)
void SPI2_IRQHandler(void);
#endif
#if (IRQ_USART1_ENABLE==1)
void USART1_IRQHandler(void);
#endif
#if (IRQ_USART2_ENABLE==1)
void USART2_IRQHandler(void);
#endif
#if (IRQ_USART3_ENABLE==1)
void USART3_IRQHandler(void);
#endif
#if (IRQ_EXTI15_10_ENABLE==1)
void EXTI15_10_IRQHandler(void);
#endif
#if (IRQ_RTCAlarm_ENABLE==1)
void RTCAlarm_IRQHandler(void);
#endif
#if (IRQ_USBWakeUp_ENABLE==1)
void USBWakeUp_IRQHandler(void);
#endif
#if (IRQ_TIM8_BRK_ENABLE==1)
void TIM8_BRK_IRQHandler(void);
#endif
#if (IRQ_TIM8_UP_ENABLE==1)
void TIM8_UP_IRQHandler(void);
#endif
#if (IRQ_TIM8_TRG_COM_ENABLE==1)
void TIM8_TRG_COM_IRQHandler(void);
#endif
#if (IRQ_TIM8_CC_ENABLE==1)
void TIM8_CC_IRQHandler(void);
#endif
#if (IRQ_ADC3_ENABLE==1)
void ADC3_IRQHandler(void);
#endif
#if (IRQ_FSMC_ENABLE==1)
void FSMC_IRQHandler(void);
#endif
#if (IRQ_SDIO_ENABLE==1)
void SDIO_IRQHandler(void);
#endif
#if (IRQ_TIM5_ENABLE==1)
void TIM5_IRQHandler(void);
#endif
#if (IRQ_SPI3_ENABLE==1)
void SPI3_IRQHandler(void);
#endif
#if (IRQ_UART4_ENABLE==1)
void UART4_IRQHandler(void);
#endif
#if (IRQ_UART5_ENABLE==1)
void UART5_IRQHandler(void);
#endif
#if (IRQ_TIM6_ENABLE==1)
void TIM6_IRQHandler(void);
#endif
#if (IRQ_TIM7_ENABLE==1)
void TIM7_IRQHandler(void);
#endif
#if (IRQ_DMA2_Channel1_ENABLE==1)
void DMA2_Channel1_IRQHandler(void);
#endif
#if (IRQ_DMA2_Channel2_ENABLE==1)
void DMA2_Channel2_IRQHandler(void);
#endif
#if (IRQ_DMA2_Channel3_ENABLE==1)
void DMA2_Channel3_IRQHandler(void);
#endif
#if (IRQ_DMA2_Channel4_5_ENABLE==1)
void DMA2_Channel4_5_IRQHandler(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
