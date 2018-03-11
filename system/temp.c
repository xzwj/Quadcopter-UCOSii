#include "stm32f4xx.h"
#include "delay.h"

void Inter_temp_sensor(){

		GPIO_InitTypeDef GPIO_InitStructure;
		ADC_CommonInitTypeDef ADC_CommonInitStructure;
		ADC_InitTypeDef ADC_InitStructure;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);

		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
		GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
		
	
	     GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
		
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);
      
		ADC_TempSensorVrefintCmd(ENABLE);//使能内部温度传感器
	
		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
		ADC_CommonInitStructure.ADC_TwoSamplingDelay =ADC_TwoSamplingDelay_5Cycles;
		ADC_CommonInitStructure.ADC_DMAAccessMode =ADC_DMAAccessMode_Disabled; 
		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
		ADC_CommonInit(&ADC_CommonInitStructure);
		
		
		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12 位模式
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐
		ADC_InitStructure.ADC_NbrOfConversion = 1;//1 个转换在规则序列中
		ADC_Init(ADC1, &ADC_InitStructure);
		ADC_Cmd(ADC1, ENABLE);//开启 AD 转换器

}


		u16 Get_Adc(u8 ch)
		{
		//设置指定 ADC 的规则组通道，一个序列，采样时间
		ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles ); 
		ADC_SoftwareStartConv(ADC1); //使能指定的 ADC1 的软件转换启动功能
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束
		return ADC_GetConversionValue(ADC1); //返回最近一次 ADC1 规则组的转换结果
		}
		
		
		
		u16 Get_Adc_Average(u8 ch,u8 times)
		{
		u32 temp_val=0; u8 t;
		for(t=0;t<times;t++)
		{
		temp_val+=Get_Adc(ch); 
			delay_ms(5);
		}
		return temp_val/times;
		}

		
		short Get_temp(){
				u32 adcx; short result;
			
			double temperate;
			adcx=(float)Get_Adc_Average(ADC_Channel_16,20);
			temperate=(float)adcx*(3.3/4096);
			temperate=(temperate-0.76)/0.0025+25;
			result=temperate*=100;
			
			return result;

		}
