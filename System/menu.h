#ifndef __MENU_H
#define __MENU_H

#include "stm32f10x.h"

typedef struct {
	int8_t flag;
	int8_t last_flag;
	int8_t initialized;
	int8_t last_selection;
} MENU;

#endif
