#include "com.h"
#include "PIDcontral.h"
#include "stdio.h"
#include "flash.h"
#include "ppm.h"
extern u8 ComRxBuffer[4];
extern PID_Struct PID_Para;
union Para TEMP;
extern uint16_t MOTOR1,MOTOR2,MOTOR3,MOTOR4;
extern uint16_t   PWMInCh1,PWMInCh2,PWMInCh3,PWMInCh4;
u8 ComRxBuffer[4];
void serialcom(void){

			if(ComRxBuffer[0]==0XFF&ComRxBuffer[1]==0XAA){
			
			 switch(ComRxBuffer[2]){
				 case 0x01: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Kp+=1.0f;
				  printf("%f",PID_Para.Kp);
			   break;
			 
				 case 0x02: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Kp-=1.0f;
				  printf("%f",PID_Para.Kp);
					break;
			 
				 case 0x03: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Kp+=0.1f;
				  printf("%f",PID_Para.Kp);
					break;
				 
				 case 0x04: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Kp-=0.1f;
				  printf("%f",PID_Para.Kp);
					break;
				
					case 0x05: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Kp+=0.01f;
				  printf("%f",PID_Para.Kp);
					break;
					
					case 0x06: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Kp-=0.01f;
				  printf("%f",PID_Para.Kp);
					break;
				
					case 0x07: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Kp-=0.001f;
				  printf("%f",PID_Para.Kp);
					break;
				
					case 0x08: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Kp+=0.001f;
				  printf("%f",PID_Para.Kp);
					break;
						
				//下面是对I参数的在线调试
					
					case 0x09: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Ti+=100.0f;
				  printf("%f",PID_Para.Ti);
					break;
					
					case 0x0A: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Ti-=100.0f;
				  printf("%f",PID_Para.Ti);
					break;
					
					case 0x0B: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Ti+=10.0f;
				  printf("%f",PID_Para.Ti);
					break;
						
					case 0x0C: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Ti-=10.0f;
				  printf("%f",PID_Para.Ti);
					break;
						
					case 0x0D: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Ti+=1.0f;
				  printf("%f",PID_Para.Ti);
					break;
					
					case 0x0E: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Ti-=1.0f;
				  printf("%f",PID_Para.Ti);
					break;

					case 0x0F: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Ti+=0.1f;
				  printf("%f",PID_Para.Ti);
					break;

					case 0x10: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Ti-=0.1f;
				  printf("%f",PID_Para.Ti);
					break;

					case 0x11: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Ti+=0.01f;
				  printf("%f",PID_Para.Ti);
					break;
	
					case 0x12: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Ti-=0.01f;
				  printf("%f",PID_Para.Ti);
					break;

					case 0x13: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Ti+=0.001f;
				  printf("%f",PID_Para.Ti);
					break;

					case 0x14: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Ti-=0.001f;
				  printf("%f",PID_Para.Ti);
					break;

					//下面是对D参数的调试
					
					 case 0x15: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Ti+=1.0f;
				  printf("%f",PID_Para.Td);
			   break;
			 
				 case 0x16: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Td-=1.0f;
				  printf("%f",PID_Para.Td);
					break;
			 
				 case 0x17: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Td+=0.1f;
				  printf("%f",PID_Para.Td);
					break;
				 
				 case 0x18: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Td-=0.1f;
				  printf("%f",PID_Para.Td);
					break;
				
					case 0x19: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Td+=0.01f;
				  printf("%f",PID_Para.Td);
					break;
					
					case 0x1A: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Td-=0.01f;
				  printf("%f",PID_Para.Td);
					break;
				
					case 0x1B: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Td-=0.001f;
				  printf("%f",PID_Para.Td);
					break;
				
					case 0x1C: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Td+=0.001f;
				  printf("%f",PID_Para.Td);
					break;
					
					case 0x20: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Kp=0.0f;
				  printf("%f",PID_Para.Td);
					break;
					
					case 0x21: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Ti=0.0f;
				  printf("%f",PID_Para.Td);
					break;
					
					case 0x22: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Ti=999999.0f;
				  printf("%f",PID_Para.Td);
					break;
					
					case 0x23: 
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
					(&PID_Para)->Td=0.0f;
				  printf("%f",PID_Para.Td);
					break;
					
					
					//将ROLL角的PID数据写进FLASH
					case 0x30:
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
				
					TEMP.a=PID_Para.Kp;
					STMFLASH_Write(ROLL_FLASH_ADDRESS,&(TEMP.b),1);
						
					TEMP.a=PID_Para.Ti;
					STMFLASH_Write(ROLL_FLASH_ADDRESS+4,&(TEMP.b),1);
					
					TEMP.a=PID_Para.Td;
					STMFLASH_Write(ROLL_FLASH_ADDRESS+8,&(TEMP.b),1);
					break;
					
					//将PITCH角的PID数据写进FLASH
					case 0x31:
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
				
					TEMP.a=PID_Para.Kp;
					STMFLASH_Write(PITCH_FLASH_ADDRESS,&(TEMP.b),1);
						
					TEMP.a=PID_Para.Ti;
					STMFLASH_Write(PITCH_FLASH_ADDRESS+4,&(TEMP.b),1);
					
					TEMP.a=PID_Para.Td;
					STMFLASH_Write(PITCH_FLASH_ADDRESS+8,&(TEMP.b),1);
					break;
					
					//将YAW角的PID数据写进FLASH
					case 0x32:
					ComRxBuffer[0]=0;
					ComRxBuffer[1]=0;
					ComRxBuffer[2]=0;
				
					TEMP.a=PID_Para.Kp;
					STMFLASH_Write(YAW_FLASH_ADDRESS,&(TEMP.b),1);
						
					TEMP.a=PID_Para.Ti;
					STMFLASH_Write(YAW_FLASH_ADDRESS+4,&(TEMP.b),1);
					
					TEMP.a=PID_Para.Td;
					STMFLASH_Write(YAW_FLASH_ADDRESS+8,&(TEMP.b),1);
					break;
					
					//打印出四个油门的当前状态
					case 0X40:
					printf("motor1=%d  motor2=%d  motor3=%d  motor4=%d\r\n",MOTOR1,MOTOR2,MOTOR3,MOTOR4);
					break;
					
					case 0x41:
					printf("pwminch1=%d  pwminch2=%d  pwminch3=%d  pwminch4=%d\r\n",PWMInCh1,PWMInCh2,PWMInCh3,PWMInCh4);	
					break;
			 }
			
			
		
			}


}
