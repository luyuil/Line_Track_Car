#include "headfile.h"

// 定义π常量
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 全局变量声明
int16_t AX, AY, AZ, GX, GY, GZ;
int16_t keynumber = 0;

//// 添加四元数欧拉角变量
//EulerAngle quat_euler_filtered;

// 菜单状态声明
MenuState current_menu = MENU_MAIN;

int main(void)
{ 
   
    OLED_Init();     
    Timer_Init();
	Key_Init();
    MPU6050_Init();
    
    OLED_Clear();    
    while (1)
    {
        // 非阻塞式菜单处理
        switch(current_menu) {
            case MENU_MAIN: {
                int result = menu_MAIN();
                if(result > 0) {
                    current_menu = result;  // 切换到子菜单
                }
                break;
            }
			case MENU_TASK1: {
                int result = menu_TASK1();
                if(result > 0) {
                    current_menu = result;  // 切换到子菜单
                }
                break;
            }
			case MENU_TASK2: {
                int result = menu_TASK2();
                if(result > 0) {
                    current_menu = result;  // 切换到子菜单
                }
                break;
            }
			case MENU_TASK3: {
                int result = menu_TASK3();
                if(result > 0) {
                    current_menu = result;  // 切换到子菜单
                }
                break;
            }
		}
		OLED_Update();
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
        
//        // 更新四元数（集成了你的算法）
//        Quaternion_Update();
        
        // 错误标志处理
        if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
        {
            TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        }
    }
}
