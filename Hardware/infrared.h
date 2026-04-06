#ifndef __INFRARED_H
#define __INFRARED_H

#include "headfile.h"

// 引脚索引定义 (0~7)
#define IR_CH0  0   // PB12
#define IR_CH1  1   // PB13
#define IR_CH2  2   // PB14
#define IR_CH3  3   // PB15
#define IR_CH4  4   // PA8
#define IR_CH5  5   // PA9
#define IR_CH6  6   // PA10
#define IR_CH7  7   // PA11

// 函数声明
void Infrared_Init(void);                   // 初始化所有红外引脚为输入
uint8_t Infrared_ReadPin(uint8_t ch);       // 读取指定通道的电平 (0:低电平, 1:高电平)
uint8_t Infrared_ReadAll(void);             // 读取所有通道状态，返回8位掩码 (bit0~bit7对应CH0~CH7)  

#endif
