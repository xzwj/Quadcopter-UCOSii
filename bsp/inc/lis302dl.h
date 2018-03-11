
/**
date: 2012-03-21
author: wzh
e-mail: wangzhihai_138@163.com

note: accelerometer MEMS sensor--LIS302DL. header file

<for STM32F4 discovery board.> */

#ifndef LIS302DL_H
#define LIS302DL_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* includes----------------------------------------------*/
#include "stm32f4xx.h"

/* defines-----------------------------------------------*/
#define LIS302DL_CSL						GPIOE->BSRRH = GPIO_Pin_3
#define LIS302DL_CSH						GPIOE->BSRRL = GPIO_Pin_3

	 
#define LIS302DL_WHO_AM_I_R					0x0F
#define LIS302DL_CTRL_REG1_RW				0x20
#define LIS302DL_CTRL_REG2_RW				0x21
#define LIS302DL_CTRL_REG3_RW				0x22
#define LIS302DL_STATUS_REG_R				0x27
#define LIS302DL_OUTX_R						0x29
#define LIS302DL_OUTY_R						0x2B
#define LIS302DL_OUTZ_R						0x2D
  
#define LIS302DL_CLICK_CFG_RW				0x38
#define LIS302DL_CLICK_SRC_R				0x39
#define LIS302DL_CLICK_THSXY_RW				0x3B
#define LIS302DL_CLICK_THSZ_RW				0x3C
#define	LIS302DL_CLICK_TIMELIMIT_RW			0x3D
#define LIS302DL_CLICK_LATENCY_RW			0x3E	 
#define LIS302DL_CLICK_WINDOW_RW			0x3F
	 
	 
	 
/* function prototypes-----------------------------------*/
void Lis302dl_Read(uint8_t addr, uint8_t *dat, uint8_t num);
void Lis302dl_Write(uint8_t addr, uint8_t dat);
void Lis302dl_Init(void); 
void Lis302dl_ReadXYZ(uint8_t *dat);   

   
#ifdef __cplusplus
 }
#endif /* C++ */

#endif /* LIS302DL_H */
