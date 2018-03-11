#include "stm32f4xx.h"

#define AilMiddle        1530         //�����е�
#define EleMiddle        1528         //�������е�
#define RudMiddle        1512         //������е�
#define PWM_MOTOR_MIN    875	      //��������С����0.875ms
#define PWM_MOTOR_MAX    2000		  //�������������2ms

extern float Time_dt;
extern float pid_roll;
extern float pid_pitch;
extern float pid_yaw;
extern uint16_t MOTOR1;
extern uint16_t MOTOR2;
extern uint16_t MOTOR3;	
extern uint16_t MOTOR4;
extern float Motor_Ail;					   //�������
extern float Motor_Ele;					   //��������
extern float Motor_Thr;					   //����
extern float Motor_Rud;					   //��������	

typedef struct {
  float expectation;            //ң�ظ�������ֵ
  float Err_k;			     //��ǰ���ֵe(k)
  float Err_k_1;		     //k-1ʱ�����ֵe(k-1)
  float Err_k_2;		     //k-2ʱ�����ֵe(k-2)
  float SumErr;              //���ĺ�
  float Kp;				     //����ϵ��
  float Ti;				     //����ϵ��
  float Td;				     //΢��ϵ��
  float Ouput_deltaUk;		 //PID�����������U(k) - U(k-1)������ʽ
  float Ouput_deltaUk_Max;		 //������������ֵ
  float Ouput_deltaUk_Min;		 //�����������Сֵ
  float PID_Integral_Max;				 //���ƻ��������ֵ
  float PID_Integral_Min;				 //���ƻ�������Сֵ
} PID_Struct;

extern PID_Struct PID_Yaw_Struct;				 //����Yaw��PID�ṹ��
extern PID_Struct PID_Roll_Struct;			 //����Roll��PID�ṹ��
extern PID_Struct PID_Pitch_Struct;			 //����Pitch��PID�ṹ��
extern PID_Struct PID_Para;

void Change_PWMTOExcept(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4);
void PID_Ini(PID_Struct *PID);
void MOTOR_CAL(void);
float Limit_PWMOUT(float MOTOR);
void TIM1_PWMOUTPUT(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4);


	