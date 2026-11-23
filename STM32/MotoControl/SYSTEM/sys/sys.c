#include "sys.h"

// THUMB指令不支持汇编内联
// 采用如下方法实现执行汇编指令WFI  
void WFI_SET(void)
{
    __ASM volatile ("wfi");
}

//关闭所有中断
void INTX_DISABLE(void)
{
    __ASM volatile ("cpsid i");
}

//开启所有中断
void INTX_ENABLE(void)
{
    __ASM volatile ("cpsie i");
}

//设置栈顶地址
//addr:栈顶地址
void MSR_MSP(u32 addr) 
{
    __ASM volatile ("msr msp, %0" : : "r" (addr));
    __ASM volatile ("bx r14");
}

/**************************************************************************
Function: Set JTAG mode
函数功能：设置JTAG模式
**************************************************************************/
//#define JTAG_SWD_DISABLE   0X02
//#define SWD_ENABLE         0X01
//#define JTAG_SWD_ENABLE    0X00	
void JTAG_Set(u8 mode)
{
    // 你的工程用的是 STM32F4，引脚复用配置在 GPIO 里由 HAL 库接管
    // 这个老函数在 F4 上已经不需要了，留个空函数防止报错
    (void)mode;
}