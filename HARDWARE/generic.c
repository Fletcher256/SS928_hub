#include "stm32f10x.h"
#include "system_stm32f10x.h"

#define DWT_CTRL      (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT    (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL_CYCCNTENA_Msk (1UL << 0)

void Delay_100ns(uint32_t n) {
    TIM_SetCounter(TIM2, 0);//��ʼ��ʱ
    while (n > 0) {
        while (TIM_GetCounter(TIM2) < 1);//��ʱ����
        TIM_SetCounter(TIM2, 0);//���¹���
        n--;
    }
}

void Delay_us(uint32_t xus)
{
    uint32_t ticks;
    uint32_t start;

    if(xus == 0) return;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT_CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    ticks = (SystemCoreClock / 1000000U) * xus;
    start = DWT_CYCCNT;
    while((uint32_t)(DWT_CYCCNT - start) < ticks);
}

void Delay_ms(uint32_t xms){
	while(xms--)
		Delay_us(1000);
}
 
void Delay_s(uint32_t xs)
{
	while(xs--)
		Delay_ms(1000);
} 

/*
GPIOx:choose your GPIO A~G
GPIO_Mode: @ref GPIOMode_TypeDef
GPIO_Speed: @ref GPIOSpeed_TypeDef
PINx:GPIO_pin_x    x(0~15)
*/
void Pin_init(GPIO_TypeDef* GPIOx,uint32_t PINx,GPIOMode_TypeDef GPIO_MODE){
    GPIO_InitTypeDef GPIOInitStruct;
    GPIOInitStruct.GPIO_Mode=GPIO_MODE;
    GPIOInitStruct.GPIO_Speed=GPIO_Speed_50MHz;
    switch(PINx){
        case GPIO_Pin_0 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_0;break;
        case GPIO_Pin_1 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_1;break;
        case GPIO_Pin_2 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_2;break;
        case GPIO_Pin_3 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_3;break;
        case GPIO_Pin_4 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_4;break;
        case GPIO_Pin_5 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_5;break;
        case GPIO_Pin_6 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_6;break;
        case GPIO_Pin_7 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_7;break;
        case GPIO_Pin_8 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_8;break;
        case GPIO_Pin_9 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_9;break;
        case GPIO_Pin_10 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_10;break;
        case GPIO_Pin_11 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_11;break;
        case GPIO_Pin_12 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_12;break;
        case GPIO_Pin_13 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_13;break;
        case GPIO_Pin_14 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_14;break;
        case GPIO_Pin_15 :GPIOInitStruct.GPIO_Pin=GPIO_Pin_15;break;
    }
    //open clock
    if(GPIOx==GPIOA)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    else if(GPIOx==GPIOB)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    else if(GPIOx==GPIOC)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    else if(GPIOx==GPIOD)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
    else if(GPIOx==GPIOE)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
    else if(GPIOx==GPIOF)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF,ENABLE);
    else if(GPIOx==GPIOG)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG,ENABLE);
   
    GPIO_Init(GPIOx,&GPIOInitStruct);
}

/*
Pin:choose your pin 0~15
Time:set your delay time /ms
*/
void PIN_invert(uint8_t Pin,uint32_t Time){

}
/*
Pin:choose your pin 0~15
Time:set your delay time of every loop /ms
times:set your blink times 
*/
void PIN_blink(uint8_t Pin,uint32_t Time,uint8_t times){

}
