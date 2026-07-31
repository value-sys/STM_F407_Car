#include "ui_task.h"

#include <stdio.h>

#include "cmsis_os2.h"
#include "chassis.h"
#include "i2c.h"
#include "key.h"
#include "motor_control.h"
#include "oled_ssd1306.h"
#include "qd4310_test.h"

static uint8_t g_ui_run_enabled;
static eUiModeTdf g_ui_mode;
static uint8_t g_ui_dirty;
static uint32_t g_ui_start_tick;
static uint32_t g_ui_elapsed_tenths;
static uint32_t g_ui_rendered_tenths;

volatile uint8_t g_ui_debug_key2_raw;
volatile uint8_t g_ui_debug_key2_stable;

static void ui_stop_motion(void)
{
    vChassisImuRotateCancel();
    vChassisStop();
    vMotorControlSetEnable(1U);
    vMotorControlStop();
}

static uint32_t ui_elapsed_tenths_now(void)
{
    const uint32_t tick_frequency = osKernelGetTickFreq();
    const uint32_t elapsed_ticks = osKernelGetTickCount() - g_ui_start_tick;

    if (tick_frequency == 0U) {
        return 0U;
    }
    return (uint32_t)(((uint64_t)elapsed_ticks * 10U) / tick_frequency);
}

static const char *ui_mode_name(eUiModeTdf mode)
{
    switch (mode) {
        case UI_MODE_STANDBY: return "STANDBY";
        case UI_MODE_LINE_LAP: return "LINE 1 CIRCLE";
        case UI_MODE_STATIC_POSITION: return "STATIC POSITION";
        case UI_MODE_AB_CENTER: return "A-B CENTER";
        case UI_MODE_LAP_CENTER: return "1 CIRCLE CENTER";
        case UI_MODE_LAP_TARGET: return "1 CIRCLE TARGET";
        default: return "UNKNOWN";
    }
}

static void ui_render(void)
{
    const char *state = "STOPPED";
    char time_text[20];
    const uint32_t seconds = g_ui_elapsed_tenths / 10U;
    const uint32_t tenths = g_ui_elapsed_tenths % 10U;

    if (g_ui_run_enabled != 0U) {
        if (g_ui_mode == UI_MODE_LINE_LAP) {
            state = "RUNNING";
        } else if (g_ui_mode == UI_MODE_STATIC_POSITION) {
            state = "BALANCE";
        } else {
            state = "NO CTRL";
        }
    } else if (g_ui_mode != UI_MODE_STANDBY) {
        state = "READY";
    }
    (void)snprintf(time_text, sizeof(time_text), "TIME:%lu.%luS",
                   (unsigned long)seconds, (unsigned long)tenths);

    OLED_Clear();
    OLED_SetCursor(0U, 0U);
    OLED_WriteString("NB CAR");
    OLED_SetCursor(0U, 2U);
    OLED_WriteString("MODE:");
    OLED_WriteString(ui_mode_name(g_ui_mode));
    OLED_SetCursor(0U, 4U);
    OLED_WriteString("STATE:");
    OLED_WriteString(state);
    OLED_SetCursor(0U, 6U);
    OLED_WriteString(time_text);
    (void)OLED_UpdateScreen();
}

void vUiInit(void)
{
    Key_Init();
    g_ui_run_enabled = 0U;
    g_ui_mode = UI_MODE_STANDBY;
    g_ui_dirty = 1U;
    g_ui_start_tick = 0U;
    g_ui_elapsed_tenths = 0U;
    g_ui_rendered_tenths = 0U;
    g_ui_debug_key2_raw = 0U;
    g_ui_debug_key2_stable = 0U;
    g_qd4310_test_motion_authorized = 0U;
    if (OLED_Init(&hi2c1, 0x3CU) == HAL_OK) {
        ui_render();
        g_ui_dirty = 0U;
    }
}

void vUiTaskUpdate(void)
{
    const uint8_t events = Key_Scan();
    g_ui_debug_key2_raw = Key_IsDownRaw(1U);
    g_ui_debug_key2_stable = Key_IsDown(1U);

    /*
     * KEY2 has priority while held. This keeps stop behavior reliable even
     * if the first debounced edge is lost during a noisy transition.
     */
    /* The stop key bypasses debounce so motion is removed immediately. */
    if (g_ui_debug_key2_raw != 0U || g_ui_debug_key2_stable != 0U) {
        vUiStop();
    } else if ((events & KEY_EVENT_1) != 0U) {
        if (g_ui_mode == UI_MODE_STANDBY) {
            g_ui_mode = UI_MODE_LINE_LAP;
        }
        if (g_ui_mode == UI_MODE_STATIC_POSITION) {
            /* Static-position mode owns the QD4310 run request. */
            g_qd4310_test_manual_mode = 0U;
            g_qd4310_test_motion_authorized = 1U;
        }
        g_ui_start_tick = osKernelGetTickCount();
        g_ui_elapsed_tenths = 0U;
        g_ui_rendered_tenths = 0U;
        g_ui_run_enabled = 1U;
        g_ui_dirty = 1U;
    } else if ((events & KEY_EVENT_2) != 0U) {
        vUiStop();
    } else if ((events & KEY_EVENT_3) != 0U) {
        vUiStop();
        g_ui_mode = (eUiModeTdf)((g_ui_mode + 1U) % UI_MODE_COUNT);
        g_ui_run_enabled = 0U;
        g_ui_elapsed_tenths = 0U;
        g_ui_rendered_tenths = 0U;
        g_ui_dirty = 1U;
    } else if ((events & KEY_EVENT_4) != 0U) {
        vUiStop();
        g_ui_mode = UI_MODE_STANDBY;
        g_ui_run_enabled = 0U;
        g_ui_elapsed_tenths = 0U;
        g_ui_rendered_tenths = 0U;
        g_ui_dirty = 1U;
    }

    if (g_ui_run_enabled != 0U) {
        g_ui_elapsed_tenths = ui_elapsed_tenths_now();
        if (g_ui_elapsed_tenths != g_ui_rendered_tenths) {
            g_ui_rendered_tenths = g_ui_elapsed_tenths;
            g_ui_dirty = 1U;
        }
    }
    if (g_ui_dirty != 0U && OLED_IsReady() != 0U) {
        ui_render();
        g_ui_dirty = 0U;
    }
}

void vUiStop(void)
{
    vQd4310TestRequestStop();
    if (g_ui_run_enabled != 0U) {
        g_ui_elapsed_tenths = ui_elapsed_tenths_now();
        g_ui_run_enabled = 0U;
        g_ui_rendered_tenths = g_ui_elapsed_tenths;
        g_ui_dirty = 1U;
    }
    ui_stop_motion();
}

uint8_t ucUiRunEnabled(void)
{
    return g_ui_run_enabled;
}

eUiModeTdf eUiGetMode(void)
{
    return g_ui_mode;
}

uint32_t ulUiElapsedTenths(void)
{
    return g_ui_elapsed_tenths;
}
