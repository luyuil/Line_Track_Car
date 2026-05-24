#include "headfile.h"

// 定义π常量
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// mpu6050六轴数据
int16_t gx,gy,gz,ax,ay,az;

// 电机速度
int32_t speed_l = 0;
int32_t speed_r = 0;

// 菜单状态声明
MenuState current_menu = MENU_MAIN;
// 菜单外部变量申明
extern MENU MAIN;
extern MENU TASK1;
extern MENU TASK2;
extern MENU TASK3;

// 该函数用来放置初始化函数，只是让主函数更美观而已
void system_init(void)
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
}

int main(void)
{ 
    system_init();
    
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
		OLED_ShowString(0,40,"sp_l:",OLED_6X8);
		OLED_ShowSignedNum(30,40,speed_l,4,OLED_6X8);
		OLED_ShowString(0,48,"sp_r:",OLED_6X8);
		OLED_ShowSignedNum(30,48,speed_r,4,OLED_6X8);
		OLED_ShowString(0,56,"yaw:",OLED_6X8);
		OLED_ShowFloatNum(24,56,imu_angle.yaw,3,2,OLED_6X8);
		OLED_Update();
    }   
}

// 中断函数，1ms中断
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {       
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        //非阻塞按键
		Key_Tick();
        
		// mpu6050读取大概要2ms以上的时间，所以定一个5ms来确保读取数据不丢失
		static uint8_t imu_tick = 0;
		if(++imu_tick >= 10)
		{
			imu_tick = 0;
			// 六轴数据读取
			MPU6050_GetData(&ax,&ay,&az,&gx,&gy,&gz);
			// 四元数解算yaw
			gyro_offset_get();
			imu_get_angle();
		}
		
		// 左右轮编码器（速度）读取
		static uint8_t encoder_tick = 0;
		if(++encoder_tick >= 10)
		{
			encoder_tick = 0;
			speed_l = Encoder1_Get();
			speed_r = Encoder2_Get();
		}
		
//===================================================================================================
// 下面放各个任务的代码
//===================================================================================================
		
//------------------------------------------------------------------
// 任务一：走直线
//------------------------------------------------------------------
		static uint8_t task1_tick = 0;
		static uint8_t task1_was_running = 0;
		uint8_t task1_running = (current_menu == MENU_TASK1 && TASK1.last_selection == 4);
		if (task1_running)
		{
			if (!task1_was_running)
			{
				line_running_reset();
			}
			if (++task1_tick >= 20)
			{
				task1_tick = 0;
				line_running();
			}
		}
		task1_was_running = task1_running;
		
//------------------------------------------------------------------
// 任务二：循迹
//------------------------------------------------------------------
		static uint8_t task2_tick = 0;
		if(current_menu == MENU_TASK2 && TASK2.last_selection == 4)
		{
			if(++task2_tick >= 20)
			{
				task2_tick = 0;
				// 这里放置任务二的函数
			}
		}
//------------------------------------------------------------------
// 任务三：绕八字
//------------------------------------------------------------------
		static uint8_t task3_tick = 0;
		if(current_menu == MENU_TASK3 && TASK3.last_selection == 4)
		{
			if(++task3_tick >= 20)
			{
				task3_tick = 0;
				// 这里放置任务三的函数
			}
		}
		
		// 空闲状态
		uint8_t any_task_running = 
            (current_menu == MENU_TASK1 && TASK1.last_selection == 4) ||
            (current_menu == MENU_TASK2 && TASK2.last_selection == 4) ||
            (current_menu == MENU_TASK3 && TASK3.last_selection == 4);
        if (!any_task_running)
        {
            Motor1_SetPWM(0);
            Motor2_SetPWM(0);
        }
		
        // 错误标志处理
        if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
        {
            TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        }
    }
}
