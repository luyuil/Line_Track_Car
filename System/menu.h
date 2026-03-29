#ifndef __MENU_H
#define __MENU_H

#include "stm32f10x.h"

// 菜单元素定义
typedef struct {
	int8_t flag;
	int8_t last_flag;
	int8_t initialized;
	int8_t last_selection;
} MENU;

// 菜单状态定义
typedef enum {
    MENU_MAIN,
    MENU_TASK1,
    MENU_TASK2,
    MENU_TASK3
} MenuState;

//==========函数部分==========
int menu_MAIN(void);
int menu_TASK1(void);
int menu_TASK2(void);
int menu_TASK3(void);

#endif
