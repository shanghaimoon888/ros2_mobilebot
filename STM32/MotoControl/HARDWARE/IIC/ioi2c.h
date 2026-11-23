#ifndef __IOI2C_H
#define __IOI2C_H

#include "main.h" // 包含 HAL 库
#include "i2c.h"  // 包含 CubeMX 生成的 I2C 句柄头文件

// 商家库里的基础函数，用 HAL 库实现
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr);
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t IICwriteBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data);
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data);

// 商家代码里不需要的软件 I2C 函数
// 不用再写 IIC_Init，IIC_Start 等函数了

#endif