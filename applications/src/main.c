 
#include "main.h"
#include "bsp.h"


void usart1_send_char(u8 c)
{

	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); 
    USART_SendData(USART2,c);   

} 

//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
}
//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart1_niming_report(0XA1,tbuf,12);//自定义帧,0XA1
}	
//通过串口2上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//清0
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;	
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_report(0XAF,tbuf,28);//飞控显示帧,0XAF
} 
extern uint16_t   PWMInCh1,PWMInCh2,PWMInCh3,PWMInCh4;
extern float Pitch,Yaw,Roll;
extern PID_Struct PID_Pitch_Struct,PID_Roll_Struct,PID_Yaw_Struct;
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			10 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				512
//任务堆栈	
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数


//获取姿态任务
//设置任务优先级
#define ATTITUDE_TASK_PRIO      			4
//任务堆栈	
OS_STK ATTITUDE_TASK_STK[START_STK_SIZE];
//任务函数

//输出油门任务
#define PWM_TASK_PRIO      			5

//任务堆栈	
OS_STK PWM_TASK_STK[START_STK_SIZE];
//任务函数


//三个任务执行函数
void Start_Task(void *pdata);
void GET_Attitude_TASK(void *pdata);
void PWM_OUTPUT_TASK(void *pdata);
#define led PAout(5)	
		
	void GPIO_Iint()   
{   
    GPIO_InitTypeDef  GPIO_InitStructure;   
       
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);       
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;   
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;   
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   
       
    GPIO_Init(GPIOA,&GPIO_InitStructure);   
       
}  
//主频84HZ   AHB1=84MHZ  APB1=42MHZ APB2=84MHZ
void SystemClcok(){
	
    RCC->CR |= ((uint32_t)RCC_CR_HSION);                     // Enable HSI
  while ((RCC->CR & RCC_CR_HSIRDY) == 0);                  // Wait for HSI Ready

  RCC->CFGR = RCC_CFGR_SW_HSI;                             // HSI is system clock
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);  // Wait for HSI used as system clock

  FLASH->ACR  = FLASH_ACR_PRFTEN;                          // Enable Prefetch Buffer
  FLASH->ACR |= FLASH_ACR_ICEN;                            // Instruction cache enable
  FLASH->ACR |= FLASH_ACR_DCEN;                            // Data cache enable
  FLASH->ACR |= FLASH_ACR_LATENCY_5WS;                     // Flash 5 wait state

  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                         // HCLK = SYSCLK
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;                        // APB1 = HCLK/2          
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;                        // APB2 = HCLK

  RCC->CR &= ~RCC_CR_PLLON;                                // Disable PLL

  // PLL configuration:  VCO = HSI/M * N,  Sysclk = VCO/P
  RCC->PLLCFGR = ( 16ul                   |                // PLL_M =  16
                 (320ul <<  6)            |                // PLL_N = 384
                 (  1ul << 16)            |                // PLL_P =   4
                 (RCC_PLLCFGR_PLLSRC_HSI) |                // PLL_SRC = HSI
                 (  8ul << 24)             );              // PLL_Q =   8

  RCC->CR |= RCC_CR_PLLON;                                 // Enable PLL
  while((RCC->CR & RCC_CR_PLLRDY) == 0) __NOP();           // Wait till PLL is ready

  RCC->CFGR &= ~RCC_CFGR_SW;                               // Select PLL as system clock source
  RCC->CFGR |=  RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait till PLL is system clock src


}



int main(void){
	
		INT8U A;
	 SystemClcok(); 
	delay_init(80);
	SystemCoreClockUpdate();
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SER_Initialize();
	delay_ms(10);
	Tim_Init();//TIM1 来输出PWM
	TIM2_Ini();//计算PID时间
	TIM5_Ini();//计算姿态计算时间
	Cap_Config();//捕获PWM tim3
	
	 GPIO_Iint();  
	MPU_Init();
	get_gyro_bias();
	//get_accel_bias();
	get_compass_bias();
	compass_calibration();
	init_quaternion(); 
	PID_Ini(&PID_Pitch_Struct);
	PID_Ini(&PID_Roll_Struct);
	PID_Ini(&PID_Yaw_Struct);	
		PID_Pitch_Struct.Kp=0.0;
	PID_Roll_Struct.Kp=0.0;
		led=1;
		
	
	OSInit();
	
	 A=OSTaskCreate(Start_Task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
		
	OSStart();
}



void Start_Task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;
	 pdata = pdata; 
  	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)    
	
		OSTaskCreate(GET_Attitude_TASK,(void *)0,(OS_STK *)&ATTITUDE_TASK_STK[START_STK_SIZE-1],ATTITUDE_TASK_PRIO);
		OSTaskCreate(PWM_OUTPUT_TASK,(void *)0,(OS_STK *)&PWM_TASK_STK[START_STK_SIZE-1],PWM_TASK_PRIO);
	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();				//退出临界区(可以被中断打断)
	
} 


void GET_Attitude_TASK(void *pdata){
	
		 
		pdata=pdata;
	while(1){
			
	 get_gy86_data();
		Get_Attitude();
		usart1_report_imu(0,0,0,0,0,0,(Pitch*100),(Roll*100),(Yaw*10));
		OSTimeDly(1);
	};
}

void PWM_OUTPUT_TASK(void *pdata){
		
		pdata=pdata;
	
			while(1){
		Change_PWMTOExcept(PWMInCh1,PWMInCh2,PWMInCh3,PWMInCh4);
			MOTOR_CAL();

			TIM1_PWMOUTPUT(MOTOR1,MOTOR2,MOTOR3,MOTOR4);
				
			OSTimeDly(2);
			};
}

