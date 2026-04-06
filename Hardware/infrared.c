#include "headfile.h"

// 静态函数：将通道索引转换为GPIO端口和引脚
static void _GetPinByChannel(uint8_t ch, GPIO_TypeDef** port, uint16_t* pin)
{
    switch(ch)
    {
        case IR_CH0: *port = GPIOB; *pin = GPIO_Pin_12; break;
        case IR_CH1: *port = GPIOB; *pin = GPIO_Pin_13; break;
        case IR_CH2: *port = GPIOB; *pin = GPIO_Pin_14; break;
        case IR_CH3: *port = GPIOB; *pin = GPIO_Pin_15; break;
        case IR_CH4: *port = GPIOA; *pin = GPIO_Pin_8;  break;
        case IR_CH5: *port = GPIOA; *pin = GPIO_Pin_9;  break;
        case IR_CH6: *port = GPIOA; *pin = GPIO_Pin_10; break;
        case IR_CH7: *port = GPIOA; *pin = GPIO_Pin_11; break;
        default:     *port = GPIOB; *pin = GPIO_Pin_12; break; // 默认返回CH0
    }
}

void Infrared_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    // 使能GPIOA和GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
    // 配置PB12~PB15为输入（浮空）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;   // 浮空输入，若红外接收头输出需上拉可改为 GPIO_Mode_IPU
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 配置PA8~PA11为输入（浮空）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// 读取单个通道电平 (返回: 0=低电平, 1=高电平)
uint8_t Infrared_ReadPin(uint8_t ch)
{
    GPIO_TypeDef* port;
    uint16_t pin;
    _GetPinByChannel(ch, &port, &pin);
    return GPIO_ReadInputDataBit(port, pin);
}

// 读取所有通道电平，返回8位掩码 (bit0=CH0, bit1=CH1, ..., bit7=CH7)
uint8_t Infrared_ReadAll(void)
{
    uint8_t status = 0;
    uint8_t i;
    for(i = 0; i < 8; i++)
    {
        if(Infrared_ReadPin(i) == 1)   // 高电平
        {
            status |= (1 << i);
        }
    }
    return status;
}
