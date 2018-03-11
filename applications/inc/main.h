/**
date: 2012-08-14
author: wzh
e-mail: wangzhihai_138@163.com

note: main header file.

<for STM32F4 discovery board.> */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* includes----------------------------------------------*/
#include "ucos_ii.h"
#include "sys.h"		
#include "stdio.h"
#include "delay.h"
#include "Serial.h"
#include "temp.h"
#include "IIC.h"
#include "mpu6050.h"
#include "tim.h"
#include "ppm.h"
#include "flash.h"
#include "PIDcontral.h"
#include "com.h"	
#include "IMU.h"

   
#ifdef __cplusplus
 }
#endif /* C++ */

#endif /* MAIN_H */

/* end of file */
