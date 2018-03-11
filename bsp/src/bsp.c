
/**
date: 2012-03-21
author: wzh
e-mail: wangzhihai_138@163.com

note: bsp source file.

<for STM32F4 discovery board.> */

/* includes----------------------------------------------*/
#include "bsp.h"


void BSP_Init(void)
{
	BSP_GPIO_Config();	
	BSP_NVIC_Config();
	
}

/**
  * @brief  Configure GPIOD.
  * @param  None.
  * @retval None.
  */
void BSP_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //open GPIOD's clock.
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &GPIO_InitStructure); //push-pull_pull-down_50MHz output.
}

/* NVIC Group set */
void BSP_NVIC_Config(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //16 pre-emption, 0 subpriority.	
	
}

/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
void BSP_Delay(__IO uint32_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}

#ifdef  USE_FULL_ASSERT
#include <stdio.h>
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
	  printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  }
}
#endif
