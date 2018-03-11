#include "stm32f4xx.h"


typedef struct{
	
				int16_t X;
				int16_t Y;
				int16_t Z;} S_INT16_XYZ;


				
				typedef struct{
				float X;
				float Y;
				float Z;}S_FLOAT_XYZ;
				
				
void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);				
float invSqrt(float x);
void GET_COMPASS(void);		
void get_gy86_data(void);
void compass_calibration(void);
void get_compass_bias(void);
int get_gyro_bias(void);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void init_quaternion(void);				
int get_accel_bias(void);
		
				