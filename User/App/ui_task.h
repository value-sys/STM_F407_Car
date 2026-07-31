#ifndef UI_TASK_H
#define UI_TASK_H

#include <stdint.h>

typedef enum
{
    UI_MODE_STANDBY = 0,
    UI_MODE_GRAY_TRACK_TEST,
    UI_MODE_LINE_LAP,
    UI_MODE_AB_CURVE_TEST,
    UI_MODE_STATIC_POSITION,
    UI_MODE_AB_CENTER,
    UI_MODE_LAP_CENTER,
    UI_MODE_LAP_TARGET,
    UI_MODE_COUNT
} eUiModeTdf;

void vUiInit(void);
void vUiTaskUpdate(void);
void vUiStop(void);
uint8_t ucUiRunEnabled(void);
eUiModeTdf eUiGetMode(void);
uint32_t ulUiElapsedTenths(void);

#endif
