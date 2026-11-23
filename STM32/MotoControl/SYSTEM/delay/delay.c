#include "delay.h"

// 使用 STM32F4 标准库的 SystemCoreClock（默认 168MHz）来计算
void delay_init(void)
{
    // 不需要动 SysTick，HAL库已经配置好了
}

// 毫秒延时：直接调用 HAL 库的延时
void delay_ms(uint32_t nms)
{
    HAL_Delay(nms);
}

// 微秒延时：利用 SysTick 的当前计数值进行差值计算，不破坏 HAL 库底层
void delay_us(uint32_t nus)
{
    uint32_t ticks = nus * (SystemCoreClock / 1000000);
    uint32_t told = SysTick->VAL;
    uint32_t tnow, tcnt = 0;
    
    while(1) 
    {
        tnow = SysTick->VAL;
        if(tnow != told) 
        {
            if(tnow < told) 
                tcnt += told - tnow;
            else 
                tcnt += SysTick->LOAD + 1 - tnow + told;
            told = tnow;
            if(tcnt >= ticks) break;
        }
    }
}