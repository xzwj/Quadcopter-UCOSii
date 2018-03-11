#include "PIDcontral.h"
#include "tim.h"

extern float Pitch, Roll, Yaw;

//�����λ��ʽPID�㷨
float Time_dt;							//���ε�PID��ʱ����
float pid_roll;							
float pid_pitch;
float pid_yaw;

float Time_dt;              //����PID������ʱ����

//4�����
uint16_t MOTOR1;						
uint16_t MOTOR2;
uint16_t MOTOR3;	
uint16_t MOTOR4;

//���Ǹ��ݽ���ң������PWM�õ���������
float Motor_Ail;					   //�������
float Motor_Ele;					   //��������
float Motor_Thr;					   //����
float Motor_Rud;					   //��������	

PID_Struct PID_Yaw_Struct;				 //����Yaw��PID�ṹ��
PID_Struct PID_Roll_Struct;			     //����Roll��PID�ṹ��
PID_Struct PID_Pitch_Struct;			 //����Pitch��PID�ṹ��
PID_Struct PID_Para;
//PID�ṹ��ĳ�ʼ��
void PID_Ini(PID_Struct *PID){
		
		PID->Err_k=0.0;                 //���ε����
		PID->Err_k_1=0.0;								//K-1ʱ�̵����
		PID->Err_k_2=0.0;								//K-2ʱ�̵����
		PID->expectation=0.0;					  //ң�صõ���������
		PID->Kp=0.0;										//����
		PID->Td=0.0;										//΢��
		PID->Ti=0.0;										//����
		PID->Ouput_deltaUk=0.0;					//PID�����������
	
//���ʹ������ʽ �������Ҫʹ��
		PID->Ouput_deltaUk_Min=-800.0;  //�����������Сֵ
		PID->Ouput_deltaUk_Max=800.0;		//������������ֵ
		PID->PID_Integral_Max=200.0;    //���ƻ���������ֵ
		PID->PID_Integral_Min=-200.0;		//���ƻ��������Сֵ
		PID->SumErr=0.0;								//����
}


float PID_Cal(PID_Struct *PID,float measured,float expect) {

	float Value_Proportion;   //������
  float Value_Integral;		//������
  float Value_Derivative;	//΢����
	
	 PID->expectation =  expect;	
  PID->Err_k = PID->expectation - measured;
  PID->SumErr = PID->SumErr + PID->Err_k;

  //P I D
  Value_Proportion    = PID->Kp * PID->Err_k;
  //Value_Integral      = PID->Kp * PID->SumErr * Time_dt / PID->Ti;	
  Value_Derivative  = PID->Kp * PID->Td * (PID->Err_k - PID->Err_k_1) / Time_dt;

  if(Value_Integral > PID->PID_Integral_Max)
  {
//    PID->SumErr -= PID->Err_k;
	Value_Integral = PID->PID_Integral_Max;
  }
  if(Value_Integral < PID->PID_Integral_Min)
  {
//  	PID->SumErr -= PID->Err_k;
    Value_Integral = PID->PID_Integral_Min;
  }
  
  PID->Ouput_deltaUk = Value_Proportion + Value_Integral + Value_Derivative;

  if(PID->Ouput_deltaUk > PID->Ouput_deltaUk_Max)
  {PID->Ouput_deltaUk = PID->Ouput_deltaUk_Max;}
  if(PID->Ouput_deltaUk < PID->Ouput_deltaUk_Min)
  {PID->Ouput_deltaUk = PID->Ouput_deltaUk_Min;}

  PID->Err_k_1 = PID->Err_k;	  //����k-1�����ֵ
  
  return PID->Ouput_deltaUk;
}

float Limit_PWMOUT(float MOTOR)
{
   if(MOTOR>PWM_MOTOR_MAX)
   {
      MOTOR=PWM_MOTOR_MAX;
   }
   else if(MOTOR<PWM_MOTOR_MIN)
   {
	  MOTOR=PWM_MOTOR_MIN;
   }
   else 
   {
	  MOTOR=MOTOR;
   }

   return MOTOR;
}


//������ĵ����PWM
void MOTOR_CAL(void)		  
{
    Time_dt = GET_PIDTIME();
  	pid_roll  = PID_Cal(&PID_Roll_Struct,  Roll,  Motor_Ail);
	pid_pitch = PID_Cal(&PID_Pitch_Struct, Pitch, Motor_Ele);

	pid_yaw   = PID_Cal(&PID_Yaw_Struct,   Yaw,   Motor_Rud);
	//xģʽ
	MOTOR1 = (uint16_t)Limit_PWMOUT(Motor_Thr + pid_pitch + pid_roll + pid_yaw);	//��ǰ�����M1��˳ʱ����ת
    MOTOR2 = (uint16_t)Limit_PWMOUT(Motor_Thr + pid_pitch - pid_roll - pid_yaw);	//��ǰ�����M2����ʱ����ת
    MOTOR3 = (uint16_t)Limit_PWMOUT(Motor_Thr - pid_pitch + pid_roll - pid_yaw);	//��󷽵��M3����ʱ����ת
    MOTOR4 = (uint16_t)Limit_PWMOUT(Motor_Thr - pid_pitch - pid_roll + pid_yaw);	//�Һ󷽵��M4��˳ʱ����ת
	
	if(Motor_Thr<=1050)//��ֹδ������ʱ������������б����ת���µ��ת��
	{
		MOTOR1=1000;
		MOTOR2=1000;
		MOTOR3=1000;
		MOTOR4=1000;
	}		  
}


//��PWMֵ�ı�Ϊ������
void Change_PWMTOExcept(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4){

   if(ch1<1000)ch1=1000;   //ROLL ALL
	 if(ch1>2000)ch1=2000;
		          
	 if(ch2<1000)ch2=1000;   //pitch  ELE
	 if(ch2>2000)ch2=2000;
	
	 if(ch3<1000)ch3=1000;   //THR����
	 if(ch3>2000)ch3=2000;
	
	 if(ch4<1000)ch4=1000;   //yaw    RUD
	 if(ch4>2000)ch4=2000;
	
		Motor_Ail=(float)((ch1-AilMiddle)*0.01);    //roll�ǵ�����ֵ  ��ΧΪ+-5��
		Motor_Ele=(float)((ch2-EleMiddle)*0.01);    //Pitch�ǵ�������  ��ΧΪ+-5��
		Motor_Rud=(float)((ch4-RudMiddle)*0.01);		//yaw�ǵ�������  ��ΧΪ+-5��
		Motor_Thr=(float)ch3;                       //���ŵ�������   ֵ��1000-2000
	
}


//����ĸ������PWMֵ
void TIM1_PWMOUTPUT(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4){

		TIM_SetCompare1(TIM1,ch1);
		TIM_SetCompare2(TIM1,ch2);
		TIM_SetCompare3(TIM1,ch3);
		TIM_SetCompare4(TIM1,ch4);
		



}


