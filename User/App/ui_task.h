#ifndef UI_TASK_H
#define UI_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    UI_MODE_STANDBY = 0,
    UI_MODE_LINE_LAP,
    UI_MODE_STATIC_POSITION,
    UI_MODE_AB_CENTER,
    UI_MODE_LAP_CENTER,
    UI_MODE_LAP_TARGET,
    UI_MODE_COUNT
} eUiModeTdf;

void vUiInit(void);
void vUiTaskUpdate(void);
void vUiStop(void);
void vUiFinishRun(void);
uint8_t ucUiRunEnabled(void);
uint8_t ucUiChassisStartAllowed(void);
eUiModeTdf eUiGetMode(void);
uint32_t ulUiElapsedTenths(void);

/* Ozone diagnostics: 1 means the active-low key is detected as pressed. */
extern volatile uint8_t g_ui_debug_key2_raw;
extern volatile uint8_t g_ui_debug_key2_stable;

#ifdef __cplusplus
}
#endif

#endif
