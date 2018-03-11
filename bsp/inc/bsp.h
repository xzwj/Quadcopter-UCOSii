/**
date: 2012-03-21
author: wzh
e-mail: wangzhihai_138@163.com

note: bsp header file.

<for STM32F4 discovery board.> */

#ifndef BSP_H
#define BSP_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* includes----------------------------------------------*/
#include "stm32f4xx.h"


   
/*function prototypes------------------------------------*/ 
void BSP_Init(void);
void BSP_GPIO_Config(void);
void BSP_NVIC_Config(void);
void BSP_Delay (uint32_t nCount); 

#ifdef  USE_FULL_ASSERT   
void assert_failed(uint8_t* file, uint32_t line);
#endif



#ifdef __cplusplus
 }
#endif /* C++ */

#endif /* BSP_H */

/* end of file */
