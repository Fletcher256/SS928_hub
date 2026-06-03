#include "stm32f10x.h"

#define AD_NUM 2

uint16_t AD_BUFFER[AD_NUM];

void ADC1_Init(void)
{
	GPIO_InitTypeDef InitGPIOA;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	//分频器配置12MHZ
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	//配浮空电平读取输入
	//ADC需要专门的读取模式来禁用GPIO防止读取干扰
	InitGPIOA.GPIO_Mode = GPIO_Mode_AIN;
	
	InitGPIOA.GPIO_Pin = 0X00000000;

	InitGPIOA.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2;
	
	InitGPIOA.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA,&InitGPIOA);
	
	//多填充菜单只需要多次配置单个通道即可
	//这个转换周期大约3.6us左右。
//	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_41Cycles5);
	
	//以下部分配置一个ADC1口所有通道的模式
	ADC_InitTypeDef ADC_InitStructure;
	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	
	//这个是定时器中断触发(不使用硬件内部自动触发。)
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	
	//这里配置ADC的触发模式,本通道配置单口独立触发。
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_Cmd(ADC1,ENABLE);
	
	//ADC上电之后需要对其进行一次校准。
	
	ADC_ResetCalibration(ADC1);
	//等待是否初始化校准完成
	while(ADC_GetResetCalibrationStatus(ADC1) == SET);
	//开始校准
	ADC_StartCalibration(ADC1);
	//等待是否校准完成
	while(ADC_GetCalibrationStatus(ADC1) == SET);
	
//	使用连续转换非扫描只需要开启扫描一次即可。
//	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}

uint16_t GetADC1Value(void)
{
	//开启转换
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	//等待转换完成
	while(RESET == ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
	
	//读取DR1数据寄存器后即可对EOC标志位进行清除。
	return ADC_GetConversionValue(ADC1);
}

//这个是整活用的通道读取。
uint16_t GetADC1ChxValue(uint8_t ADC_Channel_x)
{
	ADC_RegularChannelConfig(ADC1,ADC_Channel_x,1,ADC_SampleTime_41Cycles5);
	//开启转换
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	//等待转换完成
	while(RESET == ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
	
	//读取DR1数据寄存器后即可对EOC标志位进行清除。
	return ADC_GetConversionValue(ADC1);
}

void ADC1SCAN_Init(void)
{
	GPIO_InitTypeDef InitGPIOA;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	//分频器配置12MHZ
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	//配浮空电平读取输入
	//ADC需要专门的读取模式来禁用GPIO防止读取干扰
	InitGPIOA.GPIO_Mode = GPIO_Mode_AIN;
	
	InitGPIOA.GPIO_Pin = 0X00000000;

	InitGPIOA.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	
	InitGPIOA.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA,&InitGPIOA);
	
	//多填充菜单只需要多次配置单个通道即可
	//这个转换周期大约3.6us左右。
//	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_41Cycles5);
	
	//以下部分配置一个ADC1口所有通道的模式
	ADC_InitTypeDef ADC_InitStructure;
	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	
	//这个是定时器中断触发(不使用硬件内部自动触发。)
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	
	//这里配置ADC的触发模式,本通道配置单口独立触发。
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	
	ADC_InitStructure.ADC_NbrOfChannel = AD_NUM;
	
	//使用扫描模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_41Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1,2,ADC_SampleTime_41Cycles5);
	
		//DMA是AHB系统总线上的外设,而注入GPIO之类的是APB上的外设
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	DMA_InitTypeDef DMA_InitStructure;
	
	//转运目标为缓冲区
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_BUFFER;
	
	//注意不要把储存器的配置和外设的配置混在一起用(会有未定义行为,导致两个同时修改某些数值。。)
	//因为这个操作本质是在做一些未定义行为(相同的寄存器配置导致同步但是异常的行为。。)
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
	
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	
	//不自增的原因是因为这个寄存器是16位的,之所以需要DMA读走数据时因为每次扫描N个通道时都会把值覆盖到这个寄存器里。
	//若DMA在每次转换完后将这个数据读走那么这个数据就不会被覆盖掉了。
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	
	//这个是传输计数器
	DMA_InitStructure.DMA_BufferSize = AD_NUM;
	
	//外设站点(起点)作为SRC还是DST
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	
	//使用ADC1外设触发
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable ;
	
	//使用循环模式触发
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	//DMA开启无妨,需要ADC硬件发出触发信号才可以开始搬运
	DMA_Cmd(DMA1_Channel1,ENABLE);
	
	//ADC与DMA1的交互开启使能
	ADC_DMACmd(ADC1,ENABLE);
	
	ADC_Cmd(ADC1,ENABLE);
	
	//ADC上电之后需要对其进行一次校准。
	
	ADC_ResetCalibration(ADC1);
	//等待是否初始化校准完成
	while(ADC_GetResetCalibrationStatus(ADC1) == SET);
	//开始校准
	ADC_StartCalibration(ADC1);
	//等待是否校准完成
	while(ADC_GetCalibrationStatus(ADC1) == SET);
	
//	使用连续转换非扫描只需要开启扫描一次即可。
//	ADC_SoftwareStartConvCmd(ADC1,ENABLE);

//这里使用连续转换加扫描只需开启一次
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}

void GetADC1SCANValue(void)
{
	//开启转换
	DMA_Cmd(DMA1_Channel1,DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel1, AD_NUM);
	
	//需要DMA等待ADC的传输指令因此DMA要早于ADC开启。
	DMA_Cmd(DMA1_Channel1,ENABLE);
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	while(DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);
	DMA_ClearFlag(DMA1_FLAG_TC1);
}

uint16_t * GetADBuffer()
{
	return AD_BUFFER;
}
