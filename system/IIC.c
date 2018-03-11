
#include "IIC.h"
#include "delay.h"

//模拟IIC的实现
//其中PB8=SCL,PB9=SDA


//引脚的初始化
void IIC_Ini(void){

     GPIO_InitTypeDef GPIO_Inistructure;
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	GPIO_Inistructure.GPIO_Mode=GPIO_Mode_OUT;//设置为输出模式
	GPIO_Inistructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9;
	GPIO_Inistructure.GPIO_PuPd=GPIO_PuPd_UP;//根据IIC协议上拉
	GPIO_Inistructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Inistructure.GPIO_OType=GPIO_OType_PP;
     
	GPIO_Init(GPIOB,&GPIO_Inistructure);
	
	//根据协议，都为高电平时为空闲状态
	IIC_SDA=1;
	IIC_SCL=1;

}



//产生IIC的起始信号

void IIC_start(void){

	SDA_OUT();
	
	IIC_SDA=1;
	IIC_SCL=1;
	delay_us(5);
	IIC_SDA=0;//此时已经是起始状态了
	
	delay_us(5);
	IIC_SCL=0;//此时已经是发送数据状态了
	

}

//结束信号的产生
void  IIC_stop(void){

      SDA_OUT();
	
	IIC_SCL=0;
	IIC_SDA=0;
	delay_us(5);
	IIC_SCL=1;
	IIC_SDA=1;//根据协议产生结束信号
     delay_us(5);

}


//等待应答信号的到来
//应答信号为1表示接受失败
//为0则表示接受成功

int Wait_ACK_Come(){

	u8 wait_time=0;//允许等待一段时间
	
	SDA_IN();
	IIC_SDA=1;delay_us(1);
	IIC_SCL=1;delay_us(1);
	
	while(READ_SDA){    //如果数据线是高电平就等待它255个数，255后如果仍然是高电平，就接受失败
		
		wait_time++;
		if(wait_time>255)
		{	
			IIC_stop();
			return 1;
		}
	}
	IIC_SCL=0;
		return 0;
}


//发送ACK信号

void IIC_ACK(){
		
	
		IIC_SCL=0;
		SDA_OUT();
	
		IIC_SDA=0;
		delay_us(2);
		IIC_SCL=1;
		delay_us(2);
		IIC_SCL=0;
		
}

//发送NACK信号

void IIC_NACK(){
		
	
		IIC_SCL=0;
		SDA_OUT();
	
		IIC_SDA=1;
		delay_us(2);
		IIC_SCL=1;
		delay_us(2);
		IIC_SCL=0;	
}

//发送一个字节
//从机返回应答
//0为成功
//1为失败
void IIC_SendAByte(u8 data){
	u8 t;
	
	SDA_OUT();
	IIC_SCL=0;//时钟变低准备传输
	
	for(t=0;t<8;t++){
	IIC_SDA=(data&0x80)>>7;
		data<<=1;
		delay_us(5);

		IIC_SCL=1;//此时的数据为有效的数据
		delay_us(2); 
		IIC_SCL=0;	//传输结束
		delay_us(2);
		}
	
}



//接收一个字节
//成功返回ACK
//不成功返回NACK


//读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
u8 IIC_ReadAByte(unsigned char ack){

	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NACK();//发送nACK
    else
        IIC_ACK(); //发送ACK   
    return receive;

}

