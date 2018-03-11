#include "mpu6050.h"
#include "IIC.h"
#include "delay.h"

//MPU6050�ĳ�ʼ��
//���������ǵ�����Ϊ��2000��/S
//���ü��ٶȵķ�Χ��2g
//����0��ʾ�ɹ�
//����1��ʾʧ��
u8 MPU_Init(void){
	
	u8 res;
	IIC_Ini();
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);//��λMPU6050
	delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0x00);//����MPU6050
	MPU_Set_Accel_fsr(1);
	MPU_Set_Gyro_fsr(1);
	MPU_Set_Sampel_Rate(1000);
	MPU_Write_Byte(MPU_INT_EN_REG,0x00);
//	MPU_Write_Byte(MPU_USER_CTRL_REG,0x00);
	MPU_Write_Byte(MPU_FIFO_EN_REG,0x00);
	
//	MPU_Write_Byte(MPU_INTBP_CFG_REG,0x80);//����Ϊ�͵�ƽ��Ч
	
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0x02);//������·IIC
	delay_ms(10);
	MPU_Write_Byte(MPU_USER_CTRL_REG,0x00);//ʹ�ܴ�·IIC
	delay_ms(10);
	res=MPU_Read_byte(MPU_DEVICE_ID_REG);
	
	HMC5883_Write_Byte(0x00,0x14);//�����������ã���׼���35HZ,����ƽ����1
		HMC5883_Write_Byte(0x01,0x20);//Ĭ������1090COUNTS/��˹
		HMC5883_Write_Byte(0X02,0X00);//��������ģʽ
		
	if(res==MPU_ADDR){
		
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0x01);
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0x00);
		MPU_Set_Sampel_Rate(1000);
	}else return 1;
	return 0;
	
}


//���������ǵ����̷�Χ
//fsr=0����250��/S��fsr=1����500��/S��fsr=2����1000��/S��fsr=3����2000��/S
//һ������Ϊfsr=3
//����0��ʾ�ɹ�
//����1��ʾʧ��
u8 MPU_Set_Gyro_fsr(u8 fsr){
	
	return MPU_Write_Byte(MPU_GYRO_CONFIG_REG,(fsr<<3));
}

//���ü��ٶȴ����������̷�Χ
//fsr=0����2g��fsr=1����4g��fsr=2����8g��fsr=3����16g��
//һ������ΪFSR=0
//����0��ʾ�ɹ�
//����1��ʾʧ��

u8 MPU_Set_Accel_fsr(u8 fsr){
	
	return MPU_Write_Byte(MPU_ACCEL_CONFIG_REG,(fsr<<3));

}

//�������ֵ�ͨ�˲���
//����1��ʾʧ��
//����0��ʾ�ɹ�

u8 MPU_Set_LPF(u16 lpf){
	
	u8 data=0;
	if(lpf>188)data=1;
	else if(lpf>98)data=2;
	else if(lpf>42)data=3;
	else if(lpf>20)data=4;
	else if(lpf>10)data=5;
	else data=6;
	return MPU_Write_Byte(MPU_CFG_REG,data);

	} 

//���ò�����
//�����ʷ�Χ��4-1000
//����1��ʾʧ��
//����0��ʾ�ɹ�	
	
	
u8 MPU_Set_Sampel_Rate(u16 rate){

	u8 data=0;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);
	return  MPU_Set_LPF(rate/2);

}

//��MPU6050дһbyte����
//arguement reg:Ŀ�ļĴ���
//data��Ҫд�������
//����1��ʾʧ��
//����0��ʾ����

u8 MPU_Write_Byte(u8 reg,u8 data){
	
	IIC_start();
	IIC_SendAByte((MPU_ADDR<<1)|0);				//���һλΪ0��ʾ��д����
	if(Wait_ACK_Come()){
		IIC_stop();
		return 1;
	}
	IIC_SendAByte(reg);							//���ͼĴ����ĵ�ַ
	Wait_ACK_Come();
	IIC_SendAByte(data);						//��������
	if(Wait_ACK_Come()){
		IIC_stop();
		return 1;
	}
	IIC_stop();									//ֹͣ
	return 0;

}

//�������������������

u8 HMC5883_Write_Byte(u8 reg,u8 data){

		IIC_start();
	IIC_SendAByte((HMC5883L_Addr<<1)|0);				//���һλΪ0��ʾ��д����
	if(Wait_ACK_Come()){
		IIC_stop();
		return 1;
	}
	IIC_SendAByte(reg);							//���ͼĴ����ĵ�ַ
	Wait_ACK_Come();
	IIC_SendAByte(data);						//��������
	if(Wait_ACK_Come()){
		IIC_stop();
		return 1;
	}
	IIC_stop();									//ֹͣ
	return 0;


}


//HMC5883������
u8 HMC5883_Read_Byte(u8 reg){

		u8 result;
	IIC_start();
	IIC_SendAByte((HMC5883L_Addr<<1)|0);				//��SDA����дҪ��ȡ������
	Wait_ACK_Come();
	IIC_SendAByte(reg);							//��SDA��дҪ��ȡ�üĴ����ĵ�ַ
	Wait_ACK_Come();
	
	IIC_start();
	IIC_SendAByte((HMC5883L_Addr<<1)|1);				//���Ͷ�������
	Wait_ACK_Come();
	
	result=IIC_ReadAByte(0);					//�õ�������ҷ���NACK
	IIC_NACK();
	IIC_stop();
	
	return result;	
	
	
}

u8 MS5661_Read_Byte(u8 reg){

		u8 result;
	IIC_start();
	IIC_SendAByte((MS5661_ADDR<<1)|0);				//��SDA����дҪ��ȡ������
	Wait_ACK_Come();
	IIC_SendAByte(reg);							//��SDA��дҪ��ȡ�üĴ����ĵ�ַ
	Wait_ACK_Come();
	
	IIC_start();
	IIC_SendAByte((MS5661_ADDR<<1)|1);				//���Ͷ�������
	Wait_ACK_Come();
	
	result=IIC_ReadAByte(0);					//�õ�������ҷ���NACK
	IIC_NACK();
	IIC_stop();
	
	return result;	
	
	
}

//��MPU6050�ж�һ���Ĵ�����ֵ
//argue:  reg:��Ҫ��ȡ�ļĴ����ĵ�ַ
//����ֵ:��ȡ��������

u8 MPU_Read_byte(u8 reg){
	
	u8 result;
	IIC_start();
	IIC_SendAByte((MPU_ADDR<<1)|0);				//��SDA����дҪ��ȡ������
	Wait_ACK_Come();
	IIC_SendAByte(reg);							//��SDA��дҪ��ȡ�üĴ����ĵ�ַ
	Wait_ACK_Come();
	
	IIC_start();
	IIC_SendAByte((MPU_ADDR<<1)|1);				//���Ͷ�������
	Wait_ACK_Come();
	
	result=IIC_ReadAByte(0);					//�õ�������ҷ���NACK
	IIC_NACK();
	IIC_stop();
	
	return result;	
}


//��MPU6050����д�����ֽ�
//argue��addr:�����ĵ�ַ
//reg:�Ĵ����ĵ�ַ
//len��Ҫд������ݵĳ���
//buf:Ҫд������ݵ�����
//����0��ʾ�������
//����1��ʾ������



u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf){
	
	u8 i=0;
	IIC_start();
	IIC_SendAByte((addr<<1)|0);					//���������ĵ�ַ
	if(Wait_ACK_Come()){
		IIC_stop();
		return 1;
	}
	IIC_SendAByte(reg);							//���ͼĴ����ĵ�ַ
	Wait_ACK_Come();
	for(;i<len;i++){
		IIC_SendAByte(buf[i]);					//��ѭ���з�������
			if(Wait_ACK_Come()){
				IIC_stop();
				return 1;						//����Ļ��ͷ���1
								}
					}
	IIC_stop();
	return 0;									//û�д���ͷ���0

	
}

//������ȡ����ֽ�
//argue��addr:�����ĵ�ַ
//reg���Ĵ����ĵ�ַ
//len:��ȡ���ֽڵĸ���
//buf:��ȡ���ֽڱ���ĵط�
//����0��ʾ����
//����1��ʾ����

u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf){
			
		IIC_start();
		IIC_SendAByte((addr<<1)|0);
		if(Wait_ACK_Come()){
		IIC_stop();
		return 1;
		}
		
		IIC_SendAByte(reg);
		Wait_ACK_Come();
		
		IIC_start();
		IIC_SendAByte((addr<<1)|1);
		Wait_ACK_Come();
		
		while(len)
		{
			if(len==1)
			*buf=IIC_ReadAByte(0);
			else *buf=IIC_ReadAByte(1);
			len--;
			buf++;
		}
	
	IIC_stop();
	
	return 0;
}



//�����¶�
float MPU_Get_Temperature(){
		
		u8 temp[2];
	    short raw;
	    float temperature;
		//MPU_Read_Len(MPU_ADDR,TEMP_OUTH_REG,2,temp);
	   temp[0]=MPU_Read_byte(TEMP_OUTH_REG);
			   temp[1]=MPU_Read_byte(TEMP_OUTL_REG);
	    raw=((u16)temp[0]<<8)|temp[1];
		temperature=(double)raw/340+36.53;
	    return temperature;
}

//�õ����ٶȵ�ֵ
//����0��ʾ�������
//����1��ʾ�д���
u8  MPU_Get_Gyro(short *gx,short *gy,short *gz){

	u8 result[6],res;
	
//	result[0]=MPU_Read_byte(GYRO_XOUTH_REG);
//	result[1]=MPU_Read_byte(GYRO_XOUTL_REG);
//	result[2]=MPU_Read_byte(GYRO_YOUTH_REG);
//	result[3]=MPU_Read_byte(GYRO_YOUTL_REG);
//	result[4]=MPU_Read_byte(GYRO_ZOUTH_REG);
//	result[5]=MPU_Read_byte(GYRO_ZOUTL_REG);
	res=MPU_Read_Len(MPU_ADDR,GYRO_XOUTH_REG,6,result);
	
	if(res==0){
		
	 *gx=((u16)result[0]<<8)|result[1];
	 *gy=((u16)result[2]<<8)|result[3];
	 *gz=((u16)result[4]<<8)|result[5];
	}
	
	return res;

}


//�õ������ǵ�ֵ
//����0��ʾ�������
//����1��ʾ�д���
u8  MPU_Get_Accel(short *gx,short *gy,short *gz){

	u8 result[6],res;
	
	res=MPU_Read_Len(0x68,ACCEL_XOUTH_REG,6,result);
	if(res==0){
		
	 *gx=((u16)result[0]<<8)|result[1];
	 *gy=((u16)result[2]<<8)|result[3];
	 *gz=((u16)result[4]<<8)|result[5];
	}
	
	return res;

}

//�õ�����������X��y,z�������
//argu:*hx,*hy,*hz�ֱ��ʾ�����������
void Get_Hmc5883_Data(short *hx,short *hy,short *hz)
{
		u8 buf[6];
	
		
		MPU_Read_Len(0x1E,0x03,6,buf);
			
	
	*hx=(buf[0]<<8)|buf[1];
		if(*hx>0x7fff)
			*hx-=0xffff;
	
		*hz=(buf[2]<<8)|buf[3];
		if(*hz>0x7fff)
			*hz-=0xffff;
	

		*hy=(buf[4]<<8)|buf[5];
			if(*hy>0x7fff)
			*hy-=0xffff;
			return;
}

void Get_Ms5611_Data(){

		
	
	
	
}


u16 C[7];
void Get_Ms5661_PROM(void){
			
			u8 c11,c12,c21,c22,c31,c32,c41,c42,c51,c52,c61,c62,c71,c72;
			c11=MS5661_Read_Byte(0XA0);
			c12=MS5661_Read_Byte(0XA1);
			C[0]=(c11>>8)|c12;
	
			c21=MS5661_Read_Byte(0XA2);
			c22=MS5661_Read_Byte(0XA3);
			C[1]=(c21>>8)|c22;
	
			c31=MS5661_Read_Byte(0XA4);
			c32=MS5661_Read_Byte(0XA5);
			C[2]=(c31>>8)|c32;
	
			c41=MS5661_Read_Byte(0XA6);
			c42=MS5661_Read_Byte(0XA7);
			C[3]=(c41>>8)|c42;
		
			c51=MS5661_Read_Byte(0XA8);
			c52=MS5661_Read_Byte(0XA9);
			C[4]=(c51>>8)|c52;
			
			c61=MS5661_Read_Byte(0XAA);
			c62=MS5661_Read_Byte(0XAB);
			C[5]=(c61>>8)|c62;
			
			c71=MS5661_Read_Byte(0XAC);
			c72=MS5661_Read_Byte(0XAD);
				C[6]=(c71>>8)|c72;
			MS5661_Read_Byte(0XAE);
		
			

}

