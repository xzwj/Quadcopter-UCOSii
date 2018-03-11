#include "PIDcontral.h"
#include "tim.h"

extern float Pitch, Roll, Yaw;

//这次是位置式PID算法
float Time_dt;							//两次调PID的时间间隔
float pid_roll;							
float pid_pitch;
float pid_yaw;

float Time_dt;              //两次PID采样的时间间隔

//4个电机
uint16_t MOTOR1;						
uint16_t MOTOR2;
uint16_t MOTOR3;	
uint16_t MOTOR4;

//这是根据解析遥控器的PWM得到的期望角
float Motor_Ail;					   //横滚期望
float Motor_Ele;					   //俯仰期望
float Motor_Thr;					   //油门
float Motor_Rud;					   //航向期望	

PID_Struct PID_Yaw_Struct;				 //定义Yaw的PID结构体
PID_Struct PID_Roll_Struct;			     //定义Roll的PID结构体
PID_Struct PID_Pitch_Struct;			 //定义Pitch的PID结构体
PID_Struct PID_Para;
//PID结构体的初始化
void PID_Ini(PID_Struct *PID){
		
		PID->Err_k=0.0;                 //本次的误差
		PID->Err_k_1=0.0;								//K-1时刻的误差
		PID->Err_k_2=0.0;								//K-2时刻的误差
		PID->expectation=0.0;					  //遥控得到的期望角
		PID->Kp=0.0;										//比例
		PID->Td=0.0;										//微分
		PID->Ti=0.0;										//积分
		PID->Ouput_deltaUk=0.0;					//PID计算后的输出量
	
//如果使用增量式 以下项不需要使用
		PID->Ouput_deltaUk_Min=-800.0;  //限制输出的最小值
		PID->Ouput_deltaUk_Max=800.0;		//限制输出的最大值
		PID->PID_Integral_Max=200.0;    //限制积分项的最大值
		PID->PID_Integral_Min=-200.0;		//限制积分项的最小值
		PID->SumErr=0.0;								//误差和
}


float PID_Cal(PID_Struct *PID,float measured,float expect) {

	float Value_Proportion;   //比例项
  float Value_Integral;		//积分项
  float Value_Derivative;	//微分项
	
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

  PID->Err_k_1 = PID->Err_k;	  //保存k-1次误差值
  
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


//输出给的电机的PWM
void MOTOR_CAL(void)		  
{
    Time_dt = GET_PIDTIME();
  	pid_roll  = PID_Cal(&PID_Roll_Struct,  Roll,  Motor_Ail);
	pid_pitch = PID_Cal(&PID_Pitch_Struct, Pitch, Motor_Ele);

	pid_yaw   = PID_Cal(&PID_Yaw_Struct,   Yaw,   Motor_Rud);
	//x模式
	MOTOR1 = (uint16_t)Limit_PWMOUT(Motor_Thr + pid_pitch + pid_roll + pid_yaw);	//左前方电机M1，顺时针旋转
    MOTOR2 = (uint16_t)Limit_PWMOUT(Motor_Thr + pid_pitch - pid_roll - pid_yaw);	//右前方电机M2，逆时针旋转
    MOTOR3 = (uint16_t)Limit_PWMOUT(Motor_Thr - pid_pitch + pid_roll - pid_yaw);	//左后方电机M3，逆时针旋转
    MOTOR4 = (uint16_t)Limit_PWMOUT(Motor_Thr - pid_pitch - pid_roll + pid_yaw);	//右后方电机M4，顺时针旋转
	
	if(Motor_Thr<=1050)//防止未加油门时，由于四轴倾斜或旋转导致电机转动
	{
		MOTOR1=1000;
		MOTOR2=1000;
		MOTOR3=1000;
		MOTOR4=1000;
	}		  
}


//把PWM值改变为期望角
void Change_PWMTOExcept(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4){

   if(ch1<1000)ch1=1000;   //ROLL ALL
	 if(ch1>2000)ch1=2000;
		          
	 if(ch2<1000)ch2=1000;   //pitch  ELE
	 if(ch2>2000)ch2=2000;
	
	 if(ch3<1000)ch3=1000;   //THR油门
	 if(ch3>2000)ch3=2000;
	
	 if(ch4<1000)ch4=1000;   //yaw    RUD
	 if(ch4>2000)ch4=2000;
	
		Motor_Ail=(float)((ch1-AilMiddle)*0.01);    //roll角的期望值  范围为+-5度
		Motor_Ele=(float)((ch2-EleMiddle)*0.01);    //Pitch角的期望角  范围为+-5度
		Motor_Rud=(float)((ch4-RudMiddle)*0.01);		//yaw角的期望角  范围为+-5度
		Motor_Thr=(float)ch3;                       //油门的期望角   值域1000-2000
	
}


//输出四个电机的PWM值
void TIM1_PWMOUTPUT(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4){

		TIM_SetCompare1(TIM1,ch1);
		TIM_SetCompare2(TIM1,ch2);
		TIM_SetCompare3(TIM1,ch3);
		TIM_SetCompare4(TIM1,ch4);
		



}


