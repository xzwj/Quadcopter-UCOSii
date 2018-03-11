 
#include "main.h"
#include "bsp.h"


void usart1_send_char(u8 c)
{

	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); 
    USART_SendData(USART2,c);   

} 

//�������ݸ�����������λ�����(V2.6�汾)
//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//���28�ֽ����� 
	send_buf[len+3]=0;	//У��������
	send_buf[0]=0X88;	//֡ͷ
	send_buf[1]=fun;	//������
	send_buf[2]=len;	//���ݳ���
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//��������
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//����У���	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1 
}
//���ͼ��ٶȴ��������ݺ�����������
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
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
	usart1_niming_report(0XA1,tbuf,12);//�Զ���֡,0XA1
}	
//ͨ������2�ϱ���������̬���ݸ�����
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//��0
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
	usart1_niming_report(0XAF,tbuf,28);//�ɿ���ʾ֡,0XAF
} 
extern uint16_t   PWMInCh1,PWMInCh2,PWMInCh3,PWMInCh4;
extern float Pitch,Yaw,Roll;
extern PID_Struct PID_Pitch_Struct,PID_Roll_Struct,PID_Yaw_Struct;
//START ����
//�����������ȼ�
#define START_TASK_PRIO      			10 //��ʼ��������ȼ�����Ϊ���
//���������ջ��С
#define START_STK_SIZE  				512
//�����ջ	
OS_STK START_TASK_STK[START_STK_SIZE];
//������


//��ȡ��̬����
//�����������ȼ�
#define ATTITUDE_TASK_PRIO      			4
//�����ջ	
OS_STK ATTITUDE_TASK_STK[START_STK_SIZE];
//������

//�����������
#define PWM_TASK_PRIO      			5

//�����ջ	
OS_STK PWM_TASK_STK[START_STK_SIZE];
//������


//��������ִ�к���
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
//��Ƶ84HZ   AHB1=84MHZ  APB1=42MHZ APB2=84MHZ
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
	Tim_Init();//TIM1 �����PWM
	TIM2_Ini();//����PIDʱ��
	TIM5_Ini();//������̬����ʱ��
	Cap_Config();//����PWM tim3
	
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
	
	 A=OSTaskCreate(Start_Task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
		
	OSStart();
}



void Start_Task(void *pdata)
{
    OS_CPU_SR cpu_sr=0;
	 pdata = pdata; 
  	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��)    
	
		OSTaskCreate(GET_Attitude_TASK,(void *)0,(OS_STK *)&ATTITUDE_TASK_STK[START_STK_SIZE-1],ATTITUDE_TASK_PRIO);
		OSTaskCreate(PWM_OUTPUT_TASK,(void *)0,(OS_STK *)&PWM_TASK_STK[START_STK_SIZE-1],PWM_TASK_PRIO);
	OSTaskSuspend(START_TASK_PRIO);	//������ʼ����.
	OS_EXIT_CRITICAL();				//�˳��ٽ���(���Ա��жϴ��)
	
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

