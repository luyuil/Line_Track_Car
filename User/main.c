#include "headfile.h"

// 定义π常量
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 菜单状态声明
MenuState current_menu = MENU_MAIN;

int8_t number = 0;
int16_t left_speed = 0;
int16_t right_speed = 0;
int16_t left_location = 0;
int16_t right_location = 0;

int main(void)
{ 
    OLED_Init();     
    Timer_Init();
	Key_Init();
    MPU6050_Init();
	PWM_Init();
	Motor_Init();
	Encoder1_Init();
	Encoder2_Init();
    
    OLED_Clear();    
    while (1)
    {
        // 非阻塞式菜单处理
//        switch(current_menu) {
//            case MENU_MAIN: {
//                int result = menu_MAIN();
//                if(result > 0) {
//                    current_menu = result;  // 切换到子菜单
//                }
//                break;
//            }
//			case MENU_TASK1: {
//                int result = menu_TASK1();
//                if(result == 0) {
//                    current_menu = MENU_MAIN;  
//                }
//                break;
//            }
//			case MENU_TASK2: {
//                int result = menu_TASK2();
//                if(result == 0) {
//                    current_menu = MENU_MAIN;  
//                }
//                break;
//            }
//			case MENU_TASK3: {
//                int result = menu_TASK3();
//                if(result == 0) {
//                    current_menu = MENU_MAIN;  
//                }
//                break;
//            }
//		}
//		OLED_Update();
		
		// 电机和编码器测试
		OLED_ShowNum(0,0,left_speed,3,OLED_8X16);
		OLED_ShowNum(0,16,right_speed,3,OLED_8X16);
		OLED_ShowNum(0,32,left_location,5,OLED_8X16);
		OLED_ShowNum(0,48,right_location,5,OLED_8X16);
		OLED_Update();
		Motor1_SetPWM(50);
		Motor2_SetPWM(50);
    }   
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {       
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        //非阻塞按键
		Key_Tick();
        
		number ++;
		if(number >= 10)
		{
			number = 0;
			left_speed = Encoder1_Get();
			right_speed = Encoder2_Get();
			left_location += left_speed;
			right_location += right_speed;
		}
		
        // 错误标志处理
        if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
        {
            TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        }
    }
}
