/* Includes ------------------------------------------------------------------*/
#include "math.h"
#include "stm32f4xx.h"
#include "platform_config.h"
#include "hw_config.h"
#include "stm32f4xx_conf.h"
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Demo app includes. */
#include "BlockQ.h"
#include "blocktim.h"
#include "GenQTest.h"
#include "partest.h"
#include "QPeek.h"

/* Defines -------------------------------------------------------------------*/

typedef enum { APPSTATE_FF_TERM, APPSTATE_TESTMENU } AppState;
/* Constants used by the vMemCheckTask() task. */
#define mainCOUNT_INITIAL_VALUE		( ( unsigned long ) 0 )
#define mainNO_TASK					( 0 )
/* Variables -----------------------------------------------------------------*/

xTaskHandle xCreatedTask;
unsigned long ulMemCheckTaskRunningCount;

static void vHogeTask( void *pvParameters );

/**************************************************************************/
/*! 
    @brief  Main Program.
	@param  None.
    @retval None.
*/
/**************************************************************************/
int main(void)
{
	/* Set Basis System */
	Set_System();
	
	/* GPIOE Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB
						  |RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE
						  |RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOG,
						   ENABLE);
    
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );
	/* Set SysTickCounter for _delay_ms(); */
	SysTickInit(INTERVAL);

//	disk_timerproc();

	/* RTC configuration */
#if 0
	RTC_Configuration();
#endif
	/* Set UART and redirect to stdio 		*/
	conio_init(UART_DEFAULT_NUM,UART_BAUDLATE);
	conio_init(3,4800);	// for GPS
	conio_init(4,9600);	// for TNC
	printf("\n");
	printf("Welcome to %s test program !!\n",MPU_SUBMODEL);
	printf("Version %s!!\n",APP_VERSION);
	printf("Build Date : %s\n",__DATE__);

	ulMemCheckTaskRunningCount = mainCOUNT_INITIAL_VALUE;
	xCreatedTask = mainNO_TASK;

	xTaskCreate( vHogeTask, ( signed char * ) "CHECK", 2048, NULL,1,NULL);
	vTaskStartScheduler();

	/* Main Loop */
	while (1)
	{

	}
}

// ワーク用変数
uint32_t workc,workb,worka;

static void vHogeTask( void *pvParameters )
{
	portTickType xLastWakeTime;
	uint32_t tStep = 1000;
	xLastWakeTime = xTaskGetTickCount ();
	for( ;; ){
		vTaskDelayUntil(&xLastWakeTime, tStep/portTICK_RATE_MS);
//		指定した時間までタスクを遅延させる。
//		一定周期でタスクを起動するのに使う
//		単なる Delay なら vTaskDelay(1000 / portTICK_RATE_MS);


	//	xTaskResumeAll();
/*
		fnk1 = 0.0f;
		for (i = 1;i < 50000000;i += 4) {
			fnk1 += 4.0f / (float)(i) - 4.0f / (float)(i + 2);
		}
		printf("pi: %f\n",fnk1);
*/
	}

}

