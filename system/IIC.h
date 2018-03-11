#include "stm32f4xx.h"
#include "sys.h"



//ͨ��λ����������SDA��SCL
//����SCL������������Ҳ�����������

//IO��������
#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9���ģʽ
//IO��������	 
#define IIC_SCL    PBout(8) //SCL
#define IIC_SDA    PBout(9) //SDA	 
#define READ_SDA   PBin(9)  //����SDA 



void IIC_Ini(void);//��ʼ��IIC
void IIC_start(void);//��ʼ�ź�
void  IIC_stop(void);//�����ź�	
int Wait_ACK_Come(void);//�ȴ�ACK����
void IIC_ACK(void);//ȷ���ź�
void IIC_NACK(void);//�����ź�
void IIC_SendAByte(u8 data);//����һ���ֽ�	
u8 IIC_ReadAByte(unsigned char ack);//��ȡһ���ֽ�

