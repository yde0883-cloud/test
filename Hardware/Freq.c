#include "Freq.h"
#include "stm32f10x.h"

void FREQ_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
//    捕获
    TIM_ICInitTypeDef TIM_ICInitStruct;
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2; //通道2
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising; //上拉
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; //直连
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;   //不分频
    TIM_ICInitStruct.TIM_ICFilter = 0;
    TIM_ICInit(TIM2, &TIM_ICInitStruct);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_TimeBaseStruct.TIM_Period = 0xFFFF;
    TIM_TimeBaseStruct.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
    
    TIM_Cmd(TIM2, ENABLE);
}

uint32_t FREQ_Get(void)
{
    uint32_t cap1, cap2;
    cap1 = TIM_GetCapture2(TIM2);
    while ((cap2 = TIM_GetCapture2(TIM2)) == cap1);
    if (cap2 > cap1) {
        return 1000000 / (cap2 - cap1);
    } else {
        return 1000000 / ((0xFFFF - cap1) + cap2);
    }
}
