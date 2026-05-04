#include "headfile.h"

// 定义π常量
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// mpu6050六轴数据
int16_t gx,gy,gz,ax,ay,az;

// 菜单状态声明
MenuState current_menu = MENU_MAIN;

// 中断任务
int16_t Count = 0;

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
	Buzzer_Init();
	Infrared_Init();
    
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
                if(result == 0) {
                    current_menu = MENU_MAIN;  
                }
                break;
            }
			case MENU_TASK2: {
                int result = menu_TASK2();
                if(result == 0) {
                    current_menu = MENU_MAIN;  
                }
                break;
            }
			case MENU_TASK3: {
                int result = menu_TASK3();
                if(result == 0) {
                    current_menu = MENU_MAIN;  
                }
                break;
            }
		}
		OLED_ShowString(0,56,"yaw:",OLED_6X8);
		OLED_ShowFloatNum(24,56,imu_angle.yaw,3,2,OLED_6X8);
		OLED_Update();
		
		// 电机和编码器测试
//		Motor1_SetPWM(80);
//		Motor2_SetPWM(80);
    }   
}

// 1ms中断
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {       
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        //非阻塞按键
		Key_Tick();
        
		// mpu6050读取大概要2ms以上的时间，所有定一个5ms来确保读取数据不丢失
		static uint8_t imu_tick = 0;
		if(++imu_tick >= 5)
		{
			imu_tick = 0;
			// 六轴数据读取
			MPU6050_GetData(&ax,&ay,&az,&gx,&gy,&gz);
			// 四元数解算yaw
			gyro_offset_get();
			imu_get_angle();
		}
		
        // 错误标志处理
        if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
        {
            TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        }
    }
}
