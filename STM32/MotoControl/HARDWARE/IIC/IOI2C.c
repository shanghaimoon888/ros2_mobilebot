#include "ioi2c.h"

// 确认你在 CubeMX 里生成的 I2C 句柄名叫 hi2c2，如果是 hi2c1 请自行修改

// 硬件 I2C 连续读
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    // HAL_I2C_Mem_Read 内部会自动处理设备地址左移、START、重复START、ACK/NACK 和 STOP
    if (HAL_I2C_Mem_Read(&hi2c2, (dev << 1), reg, I2C_MEMADD_SIZE_8BIT, data, length, 100) == HAL_OK) {
        return 0; // 成功返回 0 (为了兼容商家代码逻辑)
    }
    return 1; // 失败返回 1
}

// 硬件 I2C 连续写
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
    if (HAL_I2C_Mem_Write(&hi2c2, (dev << 1), reg, I2C_MEMADD_SIZE_8BIT, data, length, 100) == HAL_OK) {
        return 1;
    }
    return 0;
}

// 硬件 I2C 写单字节
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}

// 硬件 I2C 读单字节
/*unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr)
{
    unsigned char res = 0;
    HAL_I2C_Mem_Read(&hi2c2, (I2C_Addr << 1), addr, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
    return res;
}*/

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr)
{
    unsigned char res = 0;
    
    // 兼容处理：
    // 如果传入的 I2C_Addr 是 0x68（7位地址），左移1位变成 0xD0。
    // 如果传入的 I2C_Addr 已经是 0xD0（8位地址），则直接使用。
    uint8_t dev_addr_8bit = (I2C_Addr > 0x7F) ? I2C_Addr : (I2C_Addr << 1);
    
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, dev_addr_8bit, addr, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
    
    if (status != HAL_OK) {
        // 如果读取失败，你可以返回 0，或者返回一个特殊错误码（比如 0xFF）
        return 0; 
    }
    
    return res;
}

// 兼容商家的 i2cRead 接口
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (HAL_I2C_Mem_Read(&hi2c2, (addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK) {
        return 0;
    }
    return 1;
}
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	
	 if(HAL_I2C_Mem_Write(&hi2c2, (addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
                         (uint8_t*)data, len, 100) == HAL_OK)
        return 0;
    return 1;
    
    
    uint8_t buffer[20]; // 假设最长写 20 个字节
    if (len > 19) return 1; // 防止溢出
    
    buffer[0] = reg; // 第一个字节是寄存器地址
    for (int i = 0; i < len; i++) {
        buffer[i + 1] = data[i]; // 把数据塞进去
    }

    // 使用底层 Transmit 直接一次性发出去
    if (HAL_I2C_Master_Transmit(&hi2c2, (addr << 1), buffer, len + 1, 100) == HAL_OK) {
        return 0; // 成功返回 0
    }
    return 1; // 失败返回 1
}
// 兼容商家的 i2cWrite 接口
/*int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    if (HAL_I2C_Mem_Write(&hi2c2, (addr << 1), reg, I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK) {
        return 0;
    }
    return 1;
}*/

// 读取-修改-写多个位 (MPU6050 初始化寄存器常用)
uint8_t IICwriteBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t b;
    if (i2cRead(dev, reg, 1, &b) == 0) {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    }
    return 0;
}

// 读取-修改-写单个位
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    i2cRead(dev, reg, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}