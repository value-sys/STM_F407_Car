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
static uint8_t g_ui_timer_running;
static uint8_t g_ui_target_key1_previous_raw;
static uint8_t g_ui_target_key3_previous_raw;
static uint8_t g_ui_target_key1_long;
static uint8_t g_ui_target_key3_long;
static uint32_t g_ui_target_key1_down_tick;
static uint32_t g_ui_target_key3_down_tick;
static uint32_t g_ui_target_key1_repeat_tick;
static uint32_t g_ui_target_key3_repeat_tick;

#define UI_TARGET_POSITION_STEP_MM 5.0f
#define UI_TARGET_POSITION_MIN_MM  1.0f
#define UI_TARGET_POSITION_MAX_MM  250.0f
#define UI_LONG_PRESS_START_MS     500U
#define UI_LONG_PRESS_REPEAT_MS    150U

volatile uint8_t g_ui_debug_key2_raw;
volatile uint8_t g_ui_debug_key2_stable;

static uint8_t ui_mode_is_center_hold(void)
{
    return (g_ui_mode == UI_MODE_AB_CENTER ||
            g_ui_mode == UI_MODE_LAP_CENTER) ? 1U : 0U;
}

static void ui_adjust_target_position(float delta_mm)
{
    if (g_ui_run_enabled != 0U) {
        return;
    }

    float target_mm = g_qd4310_test_target_position_mm + delta_mm;
    if (target_mm < UI_TARGET_POSITION_MIN_MM) {
        target_mm = UI_TARGET_POSITION_MIN_MM;
    } else if (target_mm > UI_TARGET_POSITION_MAX_MM) {
        target_mm = UI_TARGET_POSITION_MAX_MM;
    }
    g_qd4310_test_target_position_mm = target_mm;
    g_ui_dirty = 1U;
}

static uint8_t ui_process_target_mode_keys(uint8_t events)
{
    const uint32_t now = osKernelGetTickCount();
    uint8_t short_events = 0U;

    if (g_ui_mode != UI_MODE_LAP_TARGET) {
        g_ui_target_key1_previous_raw = 0U;
        g_ui_target_key3_previous_raw = 0U;
        g_ui_target_key1_long = 0U;
        g_ui_target_key3_long = 0U;
        return events;
    }

    const uint8_t key1_raw = Key_IsDownRaw(0U);
    const uint8_t key3_raw = Key_IsDownRaw(2U);

    /* Key1: long press decreases target; release of a short press starts. */
    if (key1_raw != 0U && g_ui_target_key1_previous_raw == 0U) {
        g_ui_target_key1_down_tick = now;
        g_ui_target_key1_repeat_tick = now + UI_LONG_PRESS_START_MS;
        g_ui_target_key1_long = 0U;
    } else if (key1_raw != 0U && g_ui_target_key1_long == 0U &&
               (uint32_t)(now - g_ui_target_key1_down_tick) >=
                   UI_LONG_PRESS_START_MS) {
        g_ui_target_key1_long = 1U;
        g_ui_target_key1_repeat_tick = now + UI_LONG_PRESS_REPEAT_MS;
        ui_adjust_target_position(-UI_TARGET_POSITION_STEP_MM);
    } else if (key1_raw != 0U && g_ui_target_key1_long != 0U &&
               (int32_t)(now - g_ui_target_key1_repeat_tick) >= 0) {
        g_ui_target_key1_repeat_tick = now + UI_LONG_PRESS_REPEAT_MS;
        ui_adjust_target_position(-UI_TARGET_POSITION_STEP_MM);
    } else if (key1_raw == 0U &&
               g_ui_target_key1_previous_raw != 0U) {
        if (g_ui_target_key1_long == 0U) {
            short_events |= KEY_EVENT_1;
        }
        g_ui_target_key1_long = 0U;
    }

    /* Key3: long press increases target; release of a short press switches. */
    if (key3_raw != 0U && g_ui_target_key3_previous_raw == 0U) {
        g_ui_target_key3_down_tick = now;
        g_ui_target_key3_repeat_tick = now + UI_LONG_PRESS_START_MS;
        g_ui_target_key3_long = 0U;
    } else if (key3_raw != 0U && g_ui_target_key3_long == 0U &&
               (uint32_t)(now - g_ui_target_key3_down_tick) >=
                   UI_LONG_PRESS_START_MS) {
        g_ui_target_key3_long = 1U;
        g_ui_target_key3_repeat_tick = now + UI_LONG_PRESS_REPEAT_MS;
        ui_adjust_target_position(UI_TARGET_POSITION_STEP_MM);
    } else if (key3_raw != 0U && g_ui_target_key3_long != 0U &&
               (int32_t)(now - g_ui_target_key3_repeat_tick) >= 0) {
        g_ui_target_key3_repeat_tick = now + UI_LONG_PRESS_REPEAT_MS;
        ui_adjust_target_position(UI_TARGET_POSITION_STEP_MM);
    } else if (key3_raw == 0U &&
               g_ui_target_key3_previous_raw != 0U) {
        if (g_ui_target_key3_long == 0U) {
            short_events |= KEY_EVENT_3;
        }
        g_ui_target_key3_long = 0U;
    }

    g_ui_target_key1_previous_raw = key1_raw;
    g_ui_target_key3_previous_raw = key3_raw;
    return (uint8_t)((events & (uint8_t)~(KEY_EVENT_1 | KEY_EVENT_3)) |
                     short_events);
}

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

static void ui_update_static_position_timing(void)
{
    if (g_ui_mode != UI_MODE_STATIC_POSITION ||
        g_ui_run_enabled == 0U) {
        return;
    }

    if (g_qd4310_test_position_sequence_timer_started != 0U &&
        g_ui_timer_running == 0U) {
        g_ui_start_tick = osKernelGetTickCount();
        g_ui_elapsed_tenths = 0U;
        g_ui_rendered_tenths = 0U;
        g_ui_timer_running = 1U;
        g_ui_dirty = 1U;
    }

    if (g_qd4310_test_position_sequence_completed != 0U &&
        g_ui_timer_running != 0U) {
        g_ui_elapsed_tenths = ui_elapsed_tenths_now();
        g_ui_timer_running = 0U;
        g_ui_rendered_tenths = g_ui_elapsed_tenths;
        g_ui_dirty = 1U;
    }
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
    char bottom_text[20];
    const uint32_t seconds = g_ui_elapsed_tenths / 10U;
    const uint32_t tenths = g_ui_elapsed_tenths % 10U;

    if (g_ui_run_enabled != 0U) {
        if (g_ui_mode == UI_MODE_LINE_LAP) {
            state = "RUNNING";
        } else if (g_ui_mode == UI_MODE_STATIC_POSITION &&
                   g_qd4310_test_position_sequence_completed != 0U) {
            state = "DONE";
        } else if (g_ui_mode == UI_MODE_STATIC_POSITION) {
            state = "BALANCE";
        } else if (ui_mode_is_center_hold() != 0U) {
            state = "BALANCE";
        } else if (g_ui_mode == UI_MODE_LAP_TARGET) {
            state = "BALANCE";
        } else {
            state = "NO CTRL";
        }
    } else if (g_ui_mode != UI_MODE_STANDBY) {
        state = "READY";
    }
    if (g_ui_mode == UI_MODE_LAP_TARGET) {
        const uint32_t target_mm =
            (uint32_t)(g_qd4310_test_target_position_mm + 0.5f);
        (void)snprintf(bottom_text, sizeof(bottom_text), "TARGET:%3luMM",
                       (unsigned long)target_mm);
    } else {
        (void)snprintf(bottom_text, sizeof(bottom_text), "TIME:%lu.%luS",
                       (unsigned long)seconds, (unsigned long)tenths);
    }

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
    OLED_WriteString(bottom_text);
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
    g_ui_timer_running = 0U;
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
    const uint8_t ui_events = ui_process_target_mode_keys(events);
    g_ui_debug_key2_raw = Key_IsDownRaw(1U);
    g_ui_debug_key2_stable = Key_IsDown(1U);

    /*
     * KEY2 has priority while held. This keeps stop behavior reliable even
     * if the first debounced edge is lost during a noisy transition.
     */
    /* The stop key bypasses debounce so motion is removed immediately. */
    if (g_ui_debug_key2_raw != 0U || g_ui_debug_key2_stable != 0U) {
        vUiStop();
    } else if ((ui_events & KEY_EVENT_1) != 0U) {
        if (g_ui_mode == UI_MODE_STANDBY) {
            g_ui_mode = UI_MODE_LINE_LAP;
        }
        if (g_ui_mode == UI_MODE_STATIC_POSITION) {
            /* Static-position mode owns the QD4310 run request. */
            g_qd4310_test_manual_mode = 0U;
            g_qd4310_test_initial_angle_deg = QD4310_TEST_INITIAL_ANGLE_DEG;
            g_qd4310_test_position_sequence_phase = 0U;
            g_qd4310_test_position_sequence_confirm_count = 0U;
            g_qd4310_test_position_sequence_timer_started = 0U;
            g_qd4310_test_position_sequence_completed = 0U;
            g_qd4310_test_position_sequence_wait_center_enabled = 1U;
            g_qd4310_test_position_sequence_enabled = 1U;
            g_qd4310_test_motion_authorized = 1U;
        }
        if (ui_mode_is_center_hold() != 0U) {
            /* Modes 4 and 5 use the same fixed 125 mm visual balance loop. */
            g_qd4310_test_manual_mode = 0U;
            g_qd4310_test_position_sequence_enabled = 0U;
            g_qd4310_test_position_sequence_wait_center_enabled = 0U;
            g_qd4310_test_position_sequence_phase = 0U;
            g_qd4310_test_position_sequence_confirm_count = 0U;
            g_qd4310_test_position_sequence_timer_started = 0U;
            g_qd4310_test_position_sequence_completed = 0U;
            g_qd4310_test_target_position_mm =
                QD4310_TEST_POSITION_SEQUENCE_CENTER_TARGET_MM;
            g_qd4310_test_motion_authorized = 1U;
        }
        if (g_ui_mode == UI_MODE_LAP_TARGET) {
            /* The selected target is controlled by the vision balance loop. */
            g_qd4310_test_manual_mode = 0U;
            g_qd4310_test_position_sequence_enabled = 0U;
            g_qd4310_test_position_sequence_wait_center_enabled = 0U;
            g_qd4310_test_position_sequence_phase = 0U;
            g_qd4310_test_position_sequence_confirm_count = 0U;
            g_qd4310_test_position_sequence_timer_started = 0U;
            g_qd4310_test_position_sequence_completed = 0U;
            g_qd4310_test_motion_authorized = 1U;
        }
        if (g_ui_mode == UI_MODE_STATIC_POSITION) {
            g_ui_start_tick = 0U;
            g_ui_timer_running = 0U;
        } else {
            g_ui_start_tick = osKernelGetTickCount();
            g_ui_timer_running = 1U;
        }
        g_ui_elapsed_tenths = 0U;
        g_ui_rendered_tenths = 0U;
        g_ui_run_enabled = 1U;
        g_ui_dirty = 1U;
    } else if ((ui_events & KEY_EVENT_2) != 0U) {
        vUiStop();
    } else if ((ui_events & KEY_EVENT_3) != 0U) {
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

    ui_update_static_position_timing();

    if (g_ui_run_enabled != 0U && g_ui_timer_running != 0U) {
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
    g_qd4310_test_position_sequence_enabled = 0U;
    g_qd4310_test_position_sequence_wait_center_enabled = 0U;
    g_qd4310_test_position_sequence_phase = 0U;
    g_qd4310_test_position_sequence_confirm_count = 0U;
    g_qd4310_test_position_sequence_timer_started = 0U;
    g_qd4310_test_position_sequence_completed = 0U;
    if (g_ui_run_enabled != 0U) {
        if (g_ui_timer_running != 0U) {
            g_ui_elapsed_tenths = ui_elapsed_tenths_now();
        }
        g_ui_timer_running = 0U;
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
