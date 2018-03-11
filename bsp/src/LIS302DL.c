
/**
date: 2012-03-21
author: wzh
e-mail: wangzhihai_138@163.com

note: accelerometer MEMS sensor--LIS302DL. source file

<for STM32F4 discovery board.> */

/* includes----------------------------------------------*/
#include "lis302dl.h"
#include "ucos_ii.h"

/* Private function prototypes --------------------------*/
static uint8_t Lis302dl_SPI_ReadWrite(uint8_t dat);

/* Private functions ------------------------------------*/
/* Read&Write LIS302DL register */
static uint8_t Lis302dl_SPI_ReadWrite(uint8_t dat)
{
	uint16_t dly;	
#if OS_CRITICAL_METHOD == 3 /* Allocate storage for CPU status register */
    OS_CPU_SR  cpu_sr = 0;
#endif

    OS_ENTER_CRITICAL();
	
	dly = 0xffff;
	while(((SPI1->SR & 0x02) != 0x02) && (dly--)); //TXE=1,then write. 
	if(dly != 0)
	{
		SPI1->DR = dat;
	}
	
	dly = 0xffff;
	while(((SPI1->SR & 0x01) != 0x01) && (dly--)); //RXNE=1,then read.
	
    OS_EXIT_CRITICAL();	
	
	if(dly != 0)
	{
		return SPI1->DR;
	}
	else
	{
		return 0;
	}
}

/* Read lis302dl */
void Lis302dl_Read(uint8_t addr, uint8_t *dat, uint8_t num)
{	
	if(num>1)
	{
		addr |= 0xc0;
	}
	else
	{
		addr |= 0x80;
	}
	
	LIS302DL_CSL;
	Lis302dl_SPI_ReadWrite(addr); 
	while(num>0)
	{
		*dat = Lis302dl_SPI_ReadWrite(0x00);
		dat++;
		num--;
	}
	LIS302DL_CSH;	
}

/* Write lis302dl */
void Lis302dl_Write(uint8_t addr, uint8_t dat)
{
	LIS302DL_CSL;
	Lis302dl_SPI_ReadWrite(addr);
	Lis302dl_SPI_ReadWrite(dat);
	LIS302DL_CSH;	
}

/* config LIS302DL MEMS sensor */
void Lis302dl_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //open spi1 clock.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE, ENABLE); //open PA&PE IO clock.
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1); //alternate function,
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1); //AF5(spi1) mapping to
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1); //PA5,6,7 pin.
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd =  GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure); //config SPI1's GPIO.
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure); //spi1 chip select.
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure); //lis302dl interrupt pin1&2.
	
	SPI_I2S_DeInit(SPI1); //reset spi1.
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure); //config spi1.
	
	SPI_Cmd(SPI1, ENABLE); //start spi1.

	Lis302dl_Write(LIS302DL_CTRL_REG1_RW, 0x47); //power-up lis302dl.	
}

/* Read X-Y-Z axis data */
void Lis302dl_ReadXYZ(uint8_t *dat)
{
	uint8_t stat;
	
	Lis302dl_Read(LIS302DL_STATUS_REG_R, &stat, 1); //read lis302dl's status.
	//X-axis.
	if((stat & 0x01) == 0x01)
	{
		Lis302dl_Read(LIS302DL_OUTX_R, &dat[0], 1); //read lis302dl's X-axis.
	}
	//Y-axis.
	if((stat & 0x02) == 0x02)
	{
		Lis302dl_Read(LIS302DL_OUTY_R, &dat[1], 1); //read lis302dl's Y-axis.
	}
	//Z-axis.
	if((stat & 0x04) == 0x04)
	{
		Lis302dl_Read(LIS302DL_OUTZ_R, &dat[2], 1); //read lis302dl's Z-axis.
	}
}

