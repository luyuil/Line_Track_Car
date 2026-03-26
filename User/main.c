#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "MPU6050.h"
#include "quaternion.h"  // 添加四元数头文件
#include "Key.h"
#include <math.h>

// 定义π常量
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 全局变量声明
int16_t AX, AY, AZ, GX, GY, GZ;
int16_t keynumber = 0;

// 添加四元数欧拉角变量
EulerAngle quat_euler_filtered;

int main(void)
{ 
   
    OLED_Init();     
    Timer_Init();
	Key_Init();
    MPU6050_Init();
    
    // 初始化四元数算法
    Quaternion_Init();
    
    // 设置零漂校准值（你的实测值）
    Quaternion_SetOffsets(92, 7, 11);
    
    // 设置互补滤波系数（你的参数：0.01）
    Quaternion_SetBeta(0.01f);
    
    // 设置滤波阈值（你的参数：0.08）
    Quaternion_SetFilterThreshold(0.08f);
    
    OLED_Clear();
    
    while (1)
    {
        OLED_Clear();
        //按键测试
		OLED_ShowNum(0,0,keynumber,3,OLED_8X16);
        OLED_Update(); 
		
		if(Key_Check(KEY_1,KEY_DOWN))
		{
			keynumber += 1;
		}
		if(Key_Check(KEY_2,KEY_DOWN))
		{
			keynumber -= 1;
		}
		if(Key_Check(KEY_3,KEY_DOWN))
		{
			keynumber += 10;
		}
		if(Key_Check(KEY_4,KEY_DOWN))
		{
			keynumber -= 10;
		}
    }   
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {       
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        //非阻塞按键
		Key_Tick();
        // 读取MPU6050数据
        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
        
        // 更新四元数（集成了你的算法）
        Quaternion_Update();
        
        // 错误标志处理
        if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
        {
            TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        }
    }
}
