#include "stm32f4xx.h"

#define AilMiddle        1530         //副翼中点
#define EleMiddle        1528         //升降舵中点
#define RudMiddle        1512         //方向舵中点
#define PWM_MOTOR_MIN    875	      //电调输出最小脉宽0.875ms
#define PWM_MOTOR_MAX    2000		  //电调输出最大脉宽2ms

extern float Time_dt;
extern float pid_roll;
extern float pid_pitch;
extern float pid_yaw;
extern uint16_t MOTOR1;
extern uint16_t MOTOR2;
extern uint16_t MOTOR3;	
extern uint16_t MOTOR4;
extern float Motor_Ail;					   //横滚期望
extern float Motor_Ele;					   //俯仰期望
extern float Motor_Thr;					   //油门
extern float Motor_Rud;					   //航向期望	

typedef struct {
  float expectation;            //遥控给的期望值
  float Err_k;			     //当前误差值e(k)
  float Err_k_1;		     //k-1时刻误差值e(k-1)
  float Err_k_2;		     //k-2时刻误差值e(k-2)
  float SumErr;              //误差的和
  float Kp;				     //比例系数
  float Ti;				     //积分系数
  float Td;				     //微分系数
  float Ouput_deltaUk;		 //PID计算后的输出量U(k) - U(k-1)，增量式
  float Ouput_deltaUk_Max;		 //限制输出量最大值
  float Ouput_deltaUk_Min;		 //限制输出量最小值
  float PID_Integral_Max;				 //限制积分项最大值
  float PID_Integral_Min;				 //限制积分项最小值
} PID_Struct;

extern PID_Struct PID_Yaw_Struct;				 //定义Yaw的PID结构体
extern PID_Struct PID_Roll_Struct;			 //定义Roll的PID结构体
extern PID_Struct PID_Pitch_Struct;			 //定义Pitch的PID结构体
extern PID_Struct PID_Para;

void Change_PWMTOExcept(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4);
void PID_Ini(PID_Struct *PID);
void MOTOR_CAL(void);
float Limit_PWMOUT(float MOTOR);
void TIM1_PWMOUTPUT(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4);


	