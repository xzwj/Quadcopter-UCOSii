#include "stm32f4xx.h"
#include "IIC.h"


#define    MPU_ADDR    0X68

#define	HMC5883L_Addr   0x1E	//�ų�������������ַ    
#define	MS5661_ADDR     0x77	//��ѹ������������ַ     ����

//���ڼ��ٶȵļĴ�������
#define    ACCEL_XOUTH_REG       0X3B            //X����ٶ�ֵ�ĸ�8λ	
#define    ACCEL_XOUTL_REG       0X3C			 //X����ٶ�ֵ�ĵ�8λ
#define    ACCEL_YOUTH_REG       0X3D            //Y����ٶ�ֵ�ĸ�8λ	
#define    ACCEL_YOUTL_REG       0X3E			 //Y����ٶ�ֵ�ĵ�8λ
#define    ACCEL_ZOUTH_REG       0X3F            //Z����ٶ�ֵ�ĸ�8λ	
#define    ACCEL_ZOUTL_REG       0X40			 //Z����ٶ�ֵ�ĵ�8λ

//�¶ȴ������ļĴ�������
#define    TEMP_OUTH_REG         0X41			 //�¶ȴ������ĸ�8λ     
#define    TEMP_OUTL_REG         0X42			 //�¶ȴ������ĵ�8λ

//���ٶȴ������Ķ���
#define    GYRO_XOUTH_REG  		 0X43            //X����ٶ�ֵ�ĸ�8λ	
#define    GYRO_XOUTL_REG  		 0X44            //X����ٶ�ֵ�ĸ�8λ
#define    GYRO_YOUTH_REG  		 0X45            //Y����ٶ�ֵ�ĸ�8λ
#define    GYRO_YOUTL_REG  		 0X46            //Y����ٶ�ֵ�ĸ�8λ
#define    GYRO_ZOUTH_REG  		 0X47            //Z����ٶ�ֵ�ĸ�8λ
#define    GYRO_ZOUTL_REG  		 0X48            //Z����ٶ�ֵ�ĸ�8λ

//��Դ����Ĵ���
#define    MPU_PWR_MGMT1_REG			 0X6B			 //��Դ����Ĵ���1
#define    MPU_PWR_MGMT2_REG			 0X6C			 //��Դ����Ĵ���1

//�û������Ǽ��ٶȴ������Ķ���
#define    MPU_CONFIG_REG                0X1A            //�û������üĴ���
#define    MPU_GYRO_CONFIG_REG			 0X1B            //�����Ǽ����ٶȴ������Ķ���
#define    MPU_ACCEL_CONFIG_REG 		 0X1C			 //���ٶȴ������Ķ���

#define    MPU_SAMPLE_RATE_REG		   	 0X19			//����Ƶ�ʷ�Ƶ��
#define    MPU_CFG_REG					 0X1A			//���üĴ���

#define    MPU_FIFO_EN_REG               0X23            //FIFOʹ�ܼĴ���

#define    MPU_SMPLRT_DIV_REG			 0X19    		 //�����ǲ����ʷ�Ƶ�Ĵ���

#define    MPU_DEVICE_ID_REG		     0X75	         //����ID�Ĵ���

#define    MPU_INTBP_CFG_REG	  		 0X37		     //�ж�/��·���üĴ���

#define    MPU_INT_EN_REG			     0X38			 //�ж�ʹ�ܼĴ���

#define    MPU_USER_CTRL_REG			 0X6A			 //�û����ƼĴ���


#define    MS5661_Reset     0x1e      //��ѹ�Ƹ�λ

//��ͬD1�̶��µ�ַ

#define    MS5661_D1_0SR256   0X40     
#define    MS5661_D1_0SR512   0X42   
#define    MS5661_D1_0SR1024   0X44   
#define    MS5661_D1_0SR2048  0X46   
#define    MS5661_D1_0SR4096  0X48

//��ͬD2�̶��µ�ַ 
#define    MS5661_D2_0SR256   0X50     
#define    MS5661_D2_0SR512   0X52   
#define    MS5661_D2_0SR1024   0X54   
#define    MS5661_D2_0SR2048  0X56   
#define    MS5661_D2_0SR4096  0X58
#define    MS5661_ADC_READ    0X00
#define    MS5661_PROM_ADDRESS  0XA0






u8 MPU_Write_Byte(u8 reg,u8 data);
u8 MPU_Read_byte(u8 reg);
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 MPU_Set_Gyro_fsr(u8 fsr);
u8 MPU_Set_Accel_fsr(u8 fsr);
u8 MPU_Set_Sampel_Rate(u16 rate);
u8 MPU_Set_LPF(u16 lpf);
u8 MPU_Init(void);
u8  MPU_Get_Accel(short *gx,short *gy,short *gz);
u8  MPU_Get_Gyro(short *gx,short *gy,short *gz);
float	MPU_Get_Temperature(void);
u8 HMC5883_Read_Byte(u8 reg);
u8 HMC5883_Write_Byte(u8 reg,u8 data);
void Get_Hmc5883_Data(short *hx,short *hy,short *hz);
void Get_Ms5661_PROM(void);
u8 MS5661_Read_Byte(u8 reg);

