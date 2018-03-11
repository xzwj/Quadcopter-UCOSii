#include "stm32f4xx.h"
#include "IIC.h"


#define    MPU_ADDR    0X68

#define	HMC5883L_Addr   0x1E	//磁场传感器器件地址    
#define	MS5661_ADDR     0x77	//气压传感器器件地址     对了

//关于加速度的寄存器定义
#define    ACCEL_XOUTH_REG       0X3B            //X轴加速度值的高8位	
#define    ACCEL_XOUTL_REG       0X3C			 //X轴加速度值的低8位
#define    ACCEL_YOUTH_REG       0X3D            //Y轴加速度值的高8位	
#define    ACCEL_YOUTL_REG       0X3E			 //Y轴加速度值的低8位
#define    ACCEL_ZOUTH_REG       0X3F            //Z轴加速度值的高8位	
#define    ACCEL_ZOUTL_REG       0X40			 //Z轴加速度值的低8位

//温度传感器的寄存器定义
#define    TEMP_OUTH_REG         0X41			 //温度传感器的高8位     
#define    TEMP_OUTL_REG         0X42			 //温度传感器的低8位

//角速度传感器的定义
#define    GYRO_XOUTH_REG  		 0X43            //X轴角速度值的高8位	
#define    GYRO_XOUTL_REG  		 0X44            //X轴角速度值的高8位
#define    GYRO_YOUTH_REG  		 0X45            //Y轴角速度值的高8位
#define    GYRO_YOUTL_REG  		 0X46            //Y轴角速度值的高8位
#define    GYRO_ZOUTH_REG  		 0X47            //Z轴角速度值的高8位
#define    GYRO_ZOUTL_REG  		 0X48            //Z轴角速度值的高8位

//电源管理寄存器
#define    MPU_PWR_MGMT1_REG			 0X6B			 //电源管理寄存器1
#define    MPU_PWR_MGMT2_REG			 0X6C			 //电源管理寄存器1

//用户陀螺仪加速度传感器的定义
#define    MPU_CONFIG_REG                0X1A            //用户的配置寄存器
#define    MPU_GYRO_CONFIG_REG			 0X1B            //陀螺仪即角速度传感器的定义
#define    MPU_ACCEL_CONFIG_REG 		 0X1C			 //加速度传感器的定义

#define    MPU_SAMPLE_RATE_REG		   	 0X19			//采样频率分频器
#define    MPU_CFG_REG					 0X1A			//配置寄存器

#define    MPU_FIFO_EN_REG               0X23            //FIFO使能寄存器

#define    MPU_SMPLRT_DIV_REG			 0X19    		 //陀螺仪采样率分频寄存器

#define    MPU_DEVICE_ID_REG		     0X75	         //器件ID寄存器

#define    MPU_INTBP_CFG_REG	  		 0X37		     //中断/旁路设置寄存器

#define    MPU_INT_EN_REG			     0X38			 //中断使能寄存器

#define    MPU_USER_CTRL_REG			 0X6A			 //用户控制寄存器


#define    MS5661_Reset     0x1e      //气压计复位

//不同D1刻度下地址

#define    MS5661_D1_0SR256   0X40     
#define    MS5661_D1_0SR512   0X42   
#define    MS5661_D1_0SR1024   0X44   
#define    MS5661_D1_0SR2048  0X46   
#define    MS5661_D1_0SR4096  0X48

//不同D2刻度下地址 
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

