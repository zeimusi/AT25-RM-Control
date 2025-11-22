#ifndef __TASK_REFEREE_UI_H
#define __TASK_REFEREE_UI_H

#include "RM_Cilent_UI.h"

enum{
    INIT = 0,
    INITING = 1,
    MOVEING = 2,
}UI_STATE;

/**
 * @brief UI初始化
 */
void UI_Init(void);

/**
 * @brief 动态UI
 */
void UI_Move(void);

/**
 * @brief UI刷新
 */
void UI_Refresh(void);

#endif
