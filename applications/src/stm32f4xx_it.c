/**
  ******************************************************************************
  * @file    RCC/RCC_Example/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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
#include "stm32f4xx_it.h"
#include "ucos_ii.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup RCC_Example
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
  /* This interrupt is generated when HSE clock fails */

  if (RCC_GetITStatus(RCC_IT_CSS) != RESET)
  {
    /* At this stage: HSE, PLL are disabled (but no change on PLL config) and HSI
       is selected as system clock source */

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Enable HSE Ready and PLL Ready interrupts */
    RCC_ITConfig(RCC_IT_HSERDY | RCC_IT_PLLRDY, ENABLE);

    /* Clear Clock Security System interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_CSS);

    /* Once HSE clock recover, the HSERDY interrupt is generated and in the RCC ISR
       routine the system clock will be reconfigured to its previous state (before
       HSE clock failure) */
  }
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
	
	
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}


	unsigned char TIM3CH1_CAPTURE_STA,TIM3CH2_CAPTURE_STA,TIM3CH3_CAPTURE_STA,TIM3CH4_CAPTURE_STA;
		uint16_t TIM3CH1_Rise,TIM3CH2_Rise,TIM3CH3_Rise,TIM3CH4_Rise;
		uint16_t TIM3CH1_Fall,TIM3CH2_Fall,TIM3CH3_Fall,TIM3CH4_Fall;
		uint16_t TIM3_T;
		uint16_t   PWMInCh1,PWMInCh2,PWMInCh3,PWMInCh4;
	

void  TIM3_IRQHandler(void){

	  //通道1
		if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET) //开始捕获
  {	
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1); //清空中断开控制位

		if(TIM3CH1_CAPTURE_STA == 1) //捕获到一次上升沿
		{ 
			TIM3CH1_Rise = TIM_GetCapture1(TIM3); //得到捕获的值 
			TIM3CH1_CAPTURE_STA = 0; //标志位设为0
			TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Falling); //变为下降沿捕获	  			    			  
		}
		else  						    //捕获的是下降沿
		{
			TIM3CH1_Fall = TIM_GetCapture1(TIM3);      //捕获到的下降沿的值
			TIM3CH1_CAPTURE_STA = 1; //标志位设为1
			if(TIM3CH1_Fall < TIM3CH1_Rise) //如果下降沿的值小于1上升沿的值
			{
				TIM3_T = 65535;
			}
			else //上升沿大于下降沿
			{
				TIM3_T = 0;
			}	
			PWMInCh1 = TIM3CH1_Fall - TIM3CH1_Rise + TIM3_T; //得到PWM的值
			TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising); //重新设为上升沿捕获	
		}		    
  }
	
	
	
	  //通道2
		if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET) //开始捕获
  {	
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2); //清空中断开控制位

		if(TIM3CH2_CAPTURE_STA == 1) //捕获到一次上升沿
		{ 
			TIM3CH2_Rise = TIM_GetCapture2(TIM3); //得到捕获的值 
			TIM3CH2_CAPTURE_STA = 0; //标志位设为0
			TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Falling); //变为下降沿捕获	  			    			  
		}
		else  						    //捕获的是下降沿
		{
			TIM3CH2_Fall = TIM_GetCapture2(TIM3);      //捕获到的下降沿的值
			TIM3CH2_CAPTURE_STA = 1; //标志位设为1
			if(TIM3CH2_Fall < TIM3CH2_Rise) //如果下降沿的值小于1上升沿的值
			{
				TIM3_T = 65535;
			}
			else //上升沿大于下降沿
			{
				TIM3_T = 0;
			}	
			PWMInCh2 = TIM3CH2_Fall - TIM3CH2_Rise + TIM3_T; //得到PWM的值
			TIM_OC2PolarityConfig(TIM3,TIM_ICPolarity_Rising); //重新设为上升沿捕获	
		}		    
  }
	
	
	  //通道3
		if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET) //开始捕获
  {	
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3); //清空中断开控制位

		if(TIM3CH3_CAPTURE_STA == 1) //捕获到一次上升沿
		{ 
			TIM3CH3_Rise = TIM_GetCapture3(TIM3); //得到捕获的值 
			TIM3CH3_CAPTURE_STA = 0; //标志位设为0
			TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Falling); //变为下降沿捕获	  			    			  
		}
		else  						    //捕获的是下降沿
		{
			TIM3CH3_Fall = TIM_GetCapture3(TIM3);      //捕获到的下降沿的值
			TIM3CH3_CAPTURE_STA = 1; //标志位设为1
			if(TIM3CH3_Fall < TIM3CH3_Rise) //如果下降沿的值小于1上升沿的值
			{
				TIM3_T = 65535;
			}
			else //上升沿大于下降沿
			{
				TIM3_T = 0;
			}	
			PWMInCh3 = TIM3CH3_Fall - TIM3CH3_Rise + TIM3_T; //得到PWM的值
			TIM_OC3PolarityConfig(TIM3,TIM_ICPolarity_Rising); //重新设为上升沿捕获	
		}		    
  }
	
	
	
	  //通道4
		if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET) //开始捕获
  {	
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4); //清空中断开控制位

		if(TIM3CH4_CAPTURE_STA == 1) //捕获到一次上升沿
		{ 
			TIM3CH4_Rise = TIM_GetCapture4(TIM3); //得到捕获的值 
			TIM3CH4_CAPTURE_STA = 0; //标志位设为0
			TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Falling); //变为下降沿捕获	  			    			  
		}
		else  						    //捕获的是下降沿
		{
			TIM3CH4_Fall = TIM_GetCapture4(TIM3);      //捕获到的下降沿的值
			TIM3CH4_CAPTURE_STA = 1; //标志位设为1
			if(TIM3CH4_Fall < TIM3CH4_Rise) //如果下降沿的值小于1上升沿的值
			{
				TIM3_T = 65535;
			}
			else //上升沿大于下降沿
			{
				TIM3_T = 0;
			}	
			PWMInCh4 = TIM3CH4_Fall - TIM3CH4_Rise + TIM3_T; //得到PWM的值
			TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising); //重新设为上升沿捕获	
		}		    
  }
	
	
}
/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

//void SysTick_Handler(void)
//{
//#if OS_CRITICAL_METHOD == 3 /* Allocate storage for CPU status register */
//    OS_CPU_SR  cpu_sr = 0;
//#endif

//    OS_ENTER_CRITICAL();  /* Tell uC/OS-II that we are starting an ISR */
//    OSIntNesting++;
//    OS_EXIT_CRITICAL();

//    OSTimeTick();  /* Call uC/OS-II's OSTimeTick() */

//    OSIntExit();  /* Tell uC/OS-II that we are leaving the ISR */	
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
#if OS_CRITICAL_METHOD == 3 // Allocate storage for CPU status register
    OS_CPU_SR  cpu_sr = 0;
#endif

    OS_ENTER_CRITICAL();  // Tell uC/OS-II that we are starting an ISR 
    OSIntNesting++;
    OS_EXIT_CRITICAL();

	//用户程序..

    OSIntExit();
}*/


/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
