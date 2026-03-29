#include "headfile.h"

// 主菜单状态变量
MENU MAIN ={
	.flag = 0,
	.last_flag = 0,
	.initialized = 0,
	.last_selection = 1,
};

// 任务1菜单状态变量
MENU TASK1 ={
	.flag = 0,
	.last_flag = 0,
	.initialized = 0,
};

// 任务2菜单状态变量
MENU TASK2 ={
	.flag = 0,
	.last_flag = 0,
	.initialized = 0,
};

// 任务3菜单状态变量
MENU TASK3 ={
	.flag = 0,
	.last_flag = 0,
	.initialized = 0,
};

// 直走pid
PID_t line = {
	.Target = 0.0,
	.Actual = 0.0,
	.Out = 0.0,
	.Kp = 0.0,
	.Ki = 0.0,
	.Kd = 0.0,
};

// 循迹pid
PID_t track = {
	.Target = 0.0,
	.Actual = 0.0,
	.Out = 0.0,
	.Kp = 0.0,
	.Ki = 0.0,
	.Kd = 0.0,
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
		OLED_ShowString(0,8*MAIN.flag,">",OLED_6X8);
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
	//按键3按下进入子菜单
	if(Key_Check(KEY_3, KEY_DOWN))
	{
		MAIN.last_selection = MAIN.flag;
		MAIN.initialized = 0;
		return MAIN.flag;
	}
	
	//更新光标
	if(MAIN.flag != MAIN.last_flag)
	{
		OLED_ShowString(0,8*MAIN.last_flag," ",OLED_6X8);
		OLED_ShowString(0,8*MAIN.flag,">",OLED_6X8);
		MAIN.last_flag = MAIN.flag;
	}
	
	return 0;
}

int menu_TASK1(void)
{
	if(!TASK1.initialized)
	{
		//显示初始菜单
		OLED_Clear();
		OLED_ShowString(0,0,"line_pid",OLED_6X8);
		OLED_ShowString(6,8,"kp",OLED_6X8);
		OLED_ShowString(6,16,"ki",OLED_6X8);
		OLED_ShowString(6,24,"kd",OLED_6X8);
		OLED_ShowString(6,32,"start",OLED_6X8);
		
		TASK1.initialized = 1;
		TASK1.last_selection = 0;
		TASK1.flag = 1;
		TASK1.last_flag = 1;
	}
	
	//未选择状态
	if(TASK1.last_selection == 0)
	{
		OLED_ShowFloatNum(48,8,line.Kp,3,2,OLED_6X8);
		OLED_ShowFloatNum(48,16,line.Ki,3,2,OLED_6X8);
		OLED_ShowFloatNum(48,32,line.Kd,3,2,OLED_6X8);
		
		if(Key_Check(KEY_1,KEY_DOWN))
		{
			TASK1.flag ++;
			if(TASK1.flag >= 5)TASK1.flag = 1;
		}
		if(Key_Check(KEY_2,KEY_DOWN))
		{
			TASK1.flag --;
			if(TASK1.flag <= 0)TASK1.flag = 4;
		}
		if(Key_Check(KEY_3, KEY_DOWN))
		{
			TASK1.last_selection = TASK1.flag;
			//显示编辑状态
			OLED_ShowString(0,80,"E",OLED_6X8);
		}
	}
	//选择状态s
	else
	{
		if(Key_Check(KEY_1, KEY_DOWN))
		{
			if(TASK1.last_selection == 1)
			{
				line.Kp += 0.1;
				OLED_ShowFloatNum(48,8,line.Kp,3,2,OLED_6X8);
			}
			else if(TASK1.last_selection == 2)
			{
				line.Ki += 0.1;
				OLED_ShowFloatNum(48,16,line.Ki,3,2,OLED_6X8);
			}
			else if(TASK1.last_selection == 3)
			{
				line.Kd += 0.1;
				OLED_ShowFloatNum(48,32,line.Kd,3,2,OLED_6X8);
			}
		}
		if(Key_Check(KEY_2, KEY_DOWN))
		{
			if(TASK1.last_selection == 1)
			{
				line.Kp -= 0.1;
				OLED_ShowFloatNum(48,8,line.Kp,3,2,OLED_6X8);
			}
			else if(TASK1.last_selection == 2)
			{
				line.Ki -= 0.1;
				OLED_ShowFloatNum(48,16,line.Ki,3,2,OLED_6X8);
			}
			else if(TASK1.last_selection == 3)
			{
				line.Kd -= 0.1;
				OLED_ShowFloatNum(48,32,line.Kd,3,2,OLED_6X8);
			}
		}
		if(Key_Check(KEY_3, KEY_DOWN))
		{
			TASK1.last_selection = 0;
			OLED_ShowString(0,80," ",OLED_6X8);
		}
	}
	
	if(Key_Check(KEY_4, KEY_DOWN)) {
        TASK1.initialized = 0;
        return 0;  // 返回主菜单
    }
	
	//更新光标
	if(TASK1.flag != TASK1.last_flag && TASK1.last_selection == 0)
	{
		OLED_ShowString(0,8*TASK1.last_flag," ",OLED_6X8);
		OLED_ShowString(0,8*TASK1.flag,">",OLED_6X8);
		TASK1.last_flag = TASK1.flag;
	}
	
	return 1;  // 1表示还在当前菜单
}

int menu_TASK2(void)
{
	
}

int menu_TASK3(void)
{
	
}
