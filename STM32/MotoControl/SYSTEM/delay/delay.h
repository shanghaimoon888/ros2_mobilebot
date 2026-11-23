#ifndef __DELAY_H
#define __DELAY_H

#include "main.h" // 包含 HAL 库的头文件

void delay_init(void);
void delay_us(uint32_t nus);
void delay_ms(uint32_t nms);

#endif