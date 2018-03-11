
#include "IIC.h"
#include "delay.h"

//ģ��IIC��ʵ��
//����PB8=SCL,PB9=SDA


//���ŵĳ�ʼ��
void IIC_Ini(void){

     GPIO_InitTypeDef GPIO_Inistructure;
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	GPIO_Inistructure.GPIO_Mode=GPIO_Mode_OUT;//����Ϊ���ģʽ
	GPIO_Inistructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9;
	GPIO_Inistructure.GPIO_PuPd=GPIO_PuPd_UP;//����IICЭ������
	GPIO_Inistructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Inistructure.GPIO_OType=GPIO_OType_PP;
     
	GPIO_Init(GPIOB,&GPIO_Inistructure);
	
	//����Э�飬��Ϊ�ߵ�ƽʱΪ����״̬
	IIC_SDA=1;
	IIC_SCL=1;

}



//����IIC����ʼ�ź�

void IIC_start(void){

	SDA_OUT();
	
	IIC_SDA=1;
	IIC_SCL=1;
	delay_us(5);
	IIC_SDA=0;//��ʱ�Ѿ�����ʼ״̬��
	
	delay_us(5);
	IIC_SCL=0;//��ʱ�Ѿ��Ƿ�������״̬��
	

}

//�����źŵĲ���
void  IIC_stop(void){

      SDA_OUT();
	
	IIC_SCL=0;
	IIC_SDA=0;
	delay_us(5);
	IIC_SCL=1;
	IIC_SDA=1;//����Э����������ź�
     delay_us(5);

}


//�ȴ�Ӧ���źŵĵ���
//Ӧ���ź�Ϊ1��ʾ����ʧ��
//Ϊ0���ʾ���ܳɹ�

int Wait_ACK_Come(){

	u8 wait_time=0;//����ȴ�һ��ʱ��
	
	SDA_IN();
	IIC_SDA=1;delay_us(1);
	IIC_SCL=1;delay_us(1);
	
	while(READ_SDA){    //����������Ǹߵ�ƽ�͵ȴ���255������255�������Ȼ�Ǹߵ�ƽ���ͽ���ʧ��
		
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


//����ACK�ź�

void IIC_ACK(){
		
	
		IIC_SCL=0;
		SDA_OUT();
	
		IIC_SDA=0;
		delay_us(2);
		IIC_SCL=1;
		delay_us(2);
		IIC_SCL=0;
		
}

//����NACK�ź�

void IIC_NACK(){
		
	
		IIC_SCL=0;
		SDA_OUT();
	
		IIC_SDA=1;
		delay_us(2);
		IIC_SCL=1;
		delay_us(2);
		IIC_SCL=0;	
}

//����һ���ֽ�
//�ӻ�����Ӧ��
//0Ϊ�ɹ�
//1Ϊʧ��
void IIC_SendAByte(u8 data){
	u8 t;
	
	SDA_OUT();
	IIC_SCL=0;//ʱ�ӱ��׼������
	
	for(t=0;t<8;t++){
	IIC_SDA=(data&0x80)>>7;
		data<<=1;
		delay_us(5);

		IIC_SCL=1;//��ʱ������Ϊ��Ч������
		delay_us(2); 
		IIC_SCL=0;	//�������
		delay_us(2);
		}
	
}



//����һ���ֽ�
//�ɹ�����ACK
//���ɹ�����NACK


//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
u8 IIC_ReadAByte(unsigned char ack){

	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
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
        IIC_NACK();//����nACK
    else
        IIC_ACK(); //����ACK   
    return receive;

}

