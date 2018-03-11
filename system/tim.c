#include "tim.h"
//ʹ��PA8-PA11
//ʹ��TIM1

void Tim_Init(void){

	    GPIO_InitTypeDef  GPIO_InitStructure;   
		
		TIM_TimeBaseInitTypeDef Tim_TimeBaseStructure;
	    TIM_OCInitTypeDef       Tim_OCIniSureuture;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//ʹ��PA
	
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE); //ʹ��TIM1
	
	//PA8-PA1���ŵĸ���
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
		
		GPIO_Init(GPIOA,&GPIO_InitStructure);
		
		
		//���ĸ��˿ڵĸ���
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_TIM1);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_TIM1);
		
		TIM_DeInit(TIM1);
		
		
	//TIM1�ĳ�ʼ����ʹ��
		Tim_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
		Tim_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
		Tim_TimeBaseStructure.TIM_Period=2500-1;
		Tim_TimeBaseStructure.TIM_Prescaler=84-1;
		Tim_TimeBaseStructure.TIM_RepetitionCounter=0;
		
		TIM_TimeBaseInit(TIM1,&Tim_TimeBaseStructure);
				
		
	//PWMģʽ���趨
		Tim_OCIniSureuture.TIM_OCMode=TIM_OCMode_PWM1;
		Tim_OCIniSureuture.TIM_OCPolarity=TIM_OCPolarity_High;
		Tim_OCIniSureuture.TIM_OutputState=TIM_OutputState_Enable;
		Tim_OCIniSureuture.TIM_Pulse=1000;
		
		//PA8 TIM1-CH1
		TIM_OC1Init(TIM1,&Tim_OCIniSureuture);
		TIM_OC1PreloadConfig(TIM1,ENABLE);
		
		//PA9 TIM1-CH2 
		TIM_OC2Init(TIM1,&Tim_OCIniSureuture);
		TIM_OC2PreloadConfig(TIM1,ENABLE);
		
		//PA10 TIM1-CH3
		TIM_OC3Init(TIM1,&Tim_OCIniSureuture);
		TIM_OC3PreloadConfig(TIM1,ENABLE);
		
		//PA11 TIM1-CH4
		TIM_OC4Init(TIM1,&Tim_OCIniSureuture);
		TIM_OC4PreloadConfig(TIM1,ENABLE);
		
		TIM_ARRPreloadConfig(TIM1, ENABLE);
		
		TIM_CtrlPWMOutputs(TIM1,ENABLE);
		
		TIM_Cmd(TIM1,ENABLE);//TIM1ʹ��


}

//ʹ��һ��32λ�Ķ�ʱ������������PID��ʱ����

void TIM2_Ini(void){

			TIM_TimeBaseInitTypeDef Tim_TimeBaseStructure;
					
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);			
			Tim_TimeBaseStructure.TIM_ClockDivision=0;
			Tim_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
			Tim_TimeBaseStructure.TIM_Period=0XFFFF;
			Tim_TimeBaseStructure.TIM_Prescaler=42-1;
	
			TIM_TimeBaseInit(TIM2,&Tim_TimeBaseStructure);
			TIM_Cmd(TIM2,ENABLE);
}
	

float GET_PIDTIME(void)//���ص�ǰtim2������ֵ,32λ
{
	float temp_PID=0 ;
	static uint32_t now_PID=0; // �������ڼ��� ��λ us

 	now_PID = TIM2->CNT;//����16λʱ��
   	TIM2->CNT=0;
	temp_PID = (float)now_PID / 1000000.0f;          //�������

	return temp_PID;
}
//ʹ��һ��32λ�Ķ�ʱ������������PID��ʱ����

void TIM5_Ini(void){

			TIM_TimeBaseInitTypeDef Tim_TimeBaseStructure;
					
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);			
			Tim_TimeBaseStructure.TIM_ClockDivision=0;
			Tim_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
			Tim_TimeBaseStructure.TIM_Period=0XFFFF;
			Tim_TimeBaseStructure.TIM_Prescaler=42-1;
	
			TIM_TimeBaseInit(TIM5,&Tim_TimeBaseStructure);
			TIM_Cmd(TIM5,ENABLE);
}
	

float GET_TIME(void)//���ص�ǰtim2������ֵ,32λ
{
	float temp_PID=0 ;
	static uint32_t now_PID=0; // �������ڼ��� ��λ us

 	now_PID = TIM5->CNT;//����16λʱ��
   	TIM5->CNT=0;
	temp_PID = (float)now_PID / 1000000.0f;          //�������

	return temp_PID;
}


void Tim4_Ini(void)
{
	
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	//�������ã�ʱ���ͱȽ�������ã���������ֻ�趨ʱ�����Բ���OC�Ƚ����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	TIM_DeInit(TIM4);

	TIM_TimeBaseStructure.TIM_Period=1000;//װ��ֵ
	//prescaler is 1200,that is 42000000/42/1000=1000Hz;
	TIM_TimeBaseStructure.TIM_Prescaler=42-1;//��Ƶϵ��
	//set clock division 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	//count up
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	//clear the TIM4 overflow interrupt flag
	TIM_ClearFlag(TIM4,TIM_FLAG_Update);
	//TIM2 overflow interrupt enable
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	//enable TIM4
	TIM_Cmd(TIM4,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	
}



