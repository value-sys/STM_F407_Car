#ifndef VISION_TASK_H
#define VISION_TASK_H

#include <stdint.h>

#include "vision_protocol.h"

typedef enum
{
    VISION_STATE_WAITING = 0,
    VISION_STATE_READY,
    VISION_STATE_INVALID,
    VISION_STATE_TIMEOUT
} eVisionState;

typedef struct
{
    uint32_t frame_ok_count;
    uint32_t crc_error_count;
    uint32_t format_error_count;
    uint32_t valid_frame_count;
    uint32_t timeout_count;
    uint32_t last_valid_tick_ms;
    eVisionState state;
} stVisionDiagnostics;

/* Ozone-readable live diagnostics. Keep these volatile for debugger watches. */
extern volatile int16_t g_vision_debug_position_01mm;
extern volatile float g_vision_debug_position_mm;
extern volatile float g_vision_debug_speed_mm_per_s;
extern volatile uint8_t g_vision_debug_valid;
extern volatile uint8_t g_vision_debug_confidence;
extern volatile uint16_t g_vision_debug_sequence;
extern volatile uint32_t g_vision_debug_timestamp_ms;
extern volatile uint32_t g_vision_debug_receive_tick_ms;
extern volatile uint32_t g_vision_debug_frame_ok_count;
extern volatile uint32_t g_vision_debug_crc_error_count;
extern volatile uint32_t g_vision_debug_format_error_count;
extern volatile uint32_t g_vision_debug_valid_frame_count;
extern volatile uint32_t g_vision_debug_timeout_count;
extern volatile uint32_t g_vision_debug_state;

void vVisionInit(void);
void vVisionTaskUpdate(void);
uint8_t ucVisionGetLatest(stVisionBallData *data);
float fVisionGetBallSpeedMmPerS(void);
eVisionState eVisionGetState(void);
void vVisionGetDiagnostics(stVisionDiagnostics *diagnostics);

#endif
