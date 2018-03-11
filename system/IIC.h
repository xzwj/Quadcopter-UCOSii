#include "stm32f4xx.h"
#include "sys.h"



//通过位带操作设置SDA和SCL
//其中SCL可以设置输入也可以设置输出

//IO方向设置
#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9输入模式
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出模式
//IO操作函数	 
#define IIC_SCL    PBout(8) //SCL
#define IIC_SDA    PBout(9) //SDA	 
#define READ_SDA   PBin(9)  //输入SDA 



void IIC_Ini(void);//初始化IIC
void IIC_start(void);//起始信号
void  IIC_stop(void);//结束信号	
int Wait_ACK_Come(void);//等待ACK到达
void IIC_ACK(void);//确认信号
void IIC_NACK(void);//否认信号
void IIC_SendAByte(u8 data);//发送一个字节	
u8 IIC_ReadAByte(unsigned char ack);//读取一个字节

