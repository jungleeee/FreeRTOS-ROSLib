/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

#include "stm32f10x.h"

#include <ros_test.h>

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    /* Infinite */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x08000000);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    __set_FAULTMASK(1);                 /* close all interrupt */

    /*!< At this stage the microcontroller clock setting is already configured,
      this is done through SystemInit() function which is called from startup
      file (startup_stm32f10x_xx.s) before to branch to application main.
      To reconfigure the default setting of SystemInit() function, refer to
      system_stm32f10x.c file
    */
        
#if 0
    RCC_GetClocksFreq(&RCC_Clock);

    printf("\r\n * SYSCLK:%d HCLK:%d\r\n   PCLK1:%d PCLK2:%d \r\n   ADCCLK:%d ",
    RCC_Clock.SYSCLK_Frequency, RCC_Clock.HCLK_Frequency, RCC_Clock.PCLK1_Frequency, RCC_Clock.PCLK2_Frequency, RCC_Clock.ADCCLK_Frequency);
#endif

    /* Add your application code here
     */
    ros_task_create();

    vTaskStartScheduler();                   /* systerm run to start */

    /* Infinite loop */
    while (1)
    {

    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
