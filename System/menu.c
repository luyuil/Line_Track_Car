#include "stm32f10x.h"
#include "OLED.h"
#include "menu.h"
#include "Key.h"

// 主菜单状态变量
MENU MAIN ={
	.flag = 0,
	.last_flag = 0,
	.initialized = 0,
	.last_selection = 1,
};

// 非阻塞主菜单
int menu_MAIN(void)
{
	if(!MAIN.initialized)
	{
		//显示初始菜单
		OLED_Clear();
		OLED_ShowString(6,0,"task1",OLED_6X8);
		OLED_ShowString(6,8,"task2",OLED_6X8);
		OLED_ShowString(6,16,"task3",OLED_6X8);
		
		//使用上次保存的选择位置
		MAIN.flag = MAIN.last_selection;
		OLED_ShowString(0,7*MAIN.flag,">",OLED_6X8);
		MAIN.initialized = 1;
		MAIN.last_flag = MAIN.flag;
	}
	
	//按键控制光标移动
	if(Key_Check(KEY_1, KEY_DOWN))
	{
		MAIN.flag ++;
		if(MAIN.flag == 3)MAIN.flag = 0;
	}
	if(Key_Check(KEY_2, KEY_DOWN))
	{
		MAIN.flag --;
		if(MAIN.flag < 0)MAIN.flag = 2;
	}
	if(Key_Check(KEY_3, KEY_DOWN))
	{
		MAIN.last_selection = MAIN.flag;
		MAIN.initialized = 0;
		return MAIN.flag;
	}
	
	//更新光标
	if(MAIN.flag != MAIN.last_flag)
	{
		OLED_ShowString(0,MAIN.last_flag," ",OLED_6X8);
		OLED_ShowString(0,MAIN.flag,">",OLED_6X8);
		MAIN.last_flag = MAIN.flag;
	}
	
	return 0;
}
