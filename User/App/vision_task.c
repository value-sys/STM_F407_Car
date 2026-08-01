#include "vision_task.h"

#include <stddef.h>

#include "main.h"
#include "usart.h"

#define VISION_DMA_BUFFER_SIZE    128U
#define VISION_FAILURE_TIMEOUT_MS 100U

static uint8_t g_vision_dma_buffer[VISION_DMA_BUFFER_SIZE];
static uint16_t g_vision_dma_last_position;
static stVisionParser g_vision_parser;
static volatile stVisionBallData g_vision_pending_data;
static volatile uint8_t g_vision_pending;
static stVisionBallData g_vision_latest_data;
static uint32_t g_vision_receive_count;
static uint8_t g_vision_have_sequence;
static uint16_t g_vision_last_sequence;
static uint8_t g_vision_have_position;
static int16_t g_vision_last_position_mm;
static uint32_t g_vision_last_position_tick_ms;
static uint32_t g_vision_last_valid_tick_ms;
static float g_vision_ball_speed_mm_per_s;
static uint32_t g_vision_valid_frame_count;
static uint32_t g_vision_timeout_count;
static eVisionState g_vision_state;

volatile int16_t g_vision_debug_position_01mm;
volatile float g_vision_debug_position_mm;
volatile float g_vision_debug_speed_mm_per_s;
volatile uint8_t g_vision_debug_valid;
volatile uint8_t g_vision_debug_confidence;
volatile uint16_t g_vision_debug_sequence;
volatile uint32_t g_vision_debug_timestamp_ms;
volatile uint32_t g_vision_debug_receive_tick_ms;
volatile uint32_t g_vision_debug_receive_count;
volatile uint32_t g_vision_debug_frame_ok_count;
volatile uint32_t g_vision_debug_crc_error_count;
volatile uint32_t g_vision_debug_format_error_count;
volatile uint32_t g_vision_debug_valid_frame_count;
volatile uint32_t g_vision_debug_timeout_count;
volatile uint32_t g_vision_debug_state;

static void vVisionStoreParsedData(const stVisionBallData *data)
{
    stVisionBallData copy = *data;
    copy.receive_tick_ms = HAL_GetTick();
    copy.receive_count = ++g_vision_receive_count;
    g_vision_pending_data = copy;
    g_vision_pending = 1U;
}

static void vVisionConsumeDmaData(uint16_t current_position,
                                  uint8_t buffer_full)
{
    if (buffer_full != 0U &&
        g_vision_dma_last_position == current_position)
    {
        for (uint16_t i = 0U; i < VISION_DMA_BUFFER_SIZE; ++i)
        {
            stVisionBallData data;
            if (ucVisionParserInput(&g_vision_parser,
                                    g_vision_dma_buffer[i], &data) != 0U)
            {
                vVisionStoreParsedData(&data);
            }
        }
        return;
    }

    while (g_vision_dma_last_position != current_position)
    {
        stVisionBallData data;
        const uint8_t byte =
            g_vision_dma_buffer[g_vision_dma_last_position];

        if (ucVisionParserInput(&g_vision_parser, byte, &data) != 0U)
        {
            vVisionStoreParsedData(&data);
        }

        ++g_vision_dma_last_position;
        if (g_vision_dma_last_position >= VISION_DMA_BUFFER_SIZE)
        {
            g_vision_dma_last_position = 0U;
        }
    }
}

void vVisionInit(void)
{
    vVisionParserInit(&g_vision_parser);
    g_vision_dma_last_position = 0U;
    g_vision_pending = 0U;
    g_vision_receive_count = 0U;
    g_vision_have_sequence = 0U;
    g_vision_have_position = 0U;
    g_vision_ball_speed_mm_per_s = 0.0f;
    g_vision_valid_frame_count = 0U;
    g_vision_last_valid_tick_ms = HAL_GetTick();
    g_vision_state = VISION_STATE_WAITING;

    g_vision_debug_position_01mm = 0;
    g_vision_debug_position_mm = 0.0f;
    g_vision_debug_speed_mm_per_s = 0.0f;
    g_vision_debug_valid = 0U;
    g_vision_debug_confidence = 0U;
    g_vision_debug_sequence = 0U;
    g_vision_debug_timestamp_ms = 0U;
    g_vision_debug_receive_tick_ms = 0U;
    g_vision_debug_receive_count = 0U;
    g_vision_debug_frame_ok_count = 0U;
    g_vision_debug_crc_error_count = 0U;
    g_vision_debug_format_error_count = 0U;
    g_vision_debug_valid_frame_count = 0U;
    g_vision_debug_timeout_count = 0U;
    g_vision_debug_state = VISION_STATE_WAITING;

    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                       g_vision_dma_buffer,
                                       VISION_DMA_BUFFER_SIZE);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                uint16_t size)
{
    if (huart != &huart1)
    {
        return;
    }

    if (size >= VISION_DMA_BUFFER_SIZE)
    {
        vVisionConsumeDmaData(0U, 1U);
    }
    else
    {
        vVisionConsumeDmaData(size, 0U);
    }
}

void vVisionTaskUpdate(void)
{
    stVisionBallData data;
    uint8_t has_data = 0U;
    const uint32_t now = HAL_GetTick();

    __disable_irq();
    if (g_vision_pending != 0U)
    {
        data = g_vision_pending_data;
        g_vision_pending = 0U;
        has_data = 1U;
    }
    __enable_irq();

    if (has_data != 0U)
    {
        g_vision_latest_data = data;
        g_vision_last_sequence = data.sequence;
        g_vision_have_sequence = 1U;

        /* Keep the legacy raw diagnostic name for Ozone compatibility. */
        g_vision_debug_position_01mm = data.ball_position_mm;
        g_vision_debug_position_mm =
            (float)data.ball_position_mm;
        g_vision_debug_valid = data.valid;
        g_vision_debug_confidence = data.confidence;
        g_vision_debug_sequence = data.sequence;
        g_vision_debug_timestamp_ms = data.timestamp_ms;
        g_vision_debug_receive_tick_ms = data.receive_tick_ms;
        g_vision_debug_receive_count = data.receive_count;

        if (data.valid != 0U &&
            data.confidence >= VISION_CONFIDENCE_MIN)
        {
            const float position_mm =
                (float)data.ball_position_mm;
            const uint32_t sample_tick = data.receive_tick_ms;
            const uint32_t dt = sample_tick -
                                g_vision_last_position_tick_ms;

            if (g_vision_have_position != 0U && dt > 0U)
            {
                const float last_mm =
                    (float)g_vision_last_position_mm;
                const float raw_speed =
                    (position_mm - last_mm) * 1000.0f / (float)dt;

                g_vision_ball_speed_mm_per_s =
                    0.7f * g_vision_ball_speed_mm_per_s +
                    0.3f * raw_speed;
            }

            g_vision_last_position_mm = data.ball_position_mm;
            g_vision_last_position_tick_ms = sample_tick;
            g_vision_have_position = 1U;
            ++g_vision_valid_frame_count;
            g_vision_last_valid_tick_ms = sample_tick;
            g_vision_state = VISION_STATE_READY;
        }
        else
        {
            g_vision_state = VISION_STATE_INVALID;
        }
    }

    if (g_vision_have_sequence != 0U &&
        (now - g_vision_last_valid_tick_ms) >
        VISION_FAILURE_TIMEOUT_MS)
    {
        if (g_vision_state != VISION_STATE_TIMEOUT)
        {
            ++g_vision_timeout_count;
        }
        g_vision_state = VISION_STATE_TIMEOUT;
    }

    g_vision_debug_speed_mm_per_s = g_vision_ball_speed_mm_per_s;
    g_vision_debug_frame_ok_count = g_vision_parser.frame_ok_count;
    g_vision_debug_crc_error_count = g_vision_parser.crc_error_count;
    g_vision_debug_format_error_count =
        g_vision_parser.format_error_count;
    g_vision_debug_valid_frame_count = g_vision_valid_frame_count;
    g_vision_debug_timeout_count = g_vision_timeout_count;
    g_vision_debug_state = (uint32_t)g_vision_state;
}

uint8_t ucVisionGetLatest(stVisionBallData *data)
{
    if (data == NULL || g_vision_have_sequence == 0U)
    {
        return 0U;
    }

    *data = g_vision_latest_data;
    return 1U;
}

float fVisionGetBallSpeedMmPerS(void)
{
    return g_vision_ball_speed_mm_per_s;
}

eVisionState eVisionGetState(void)
{
    return g_vision_state;
}

void vVisionGetDiagnostics(stVisionDiagnostics *diagnostics)
{
    if (diagnostics == NULL)
    {
        return;
    }

    diagnostics->frame_ok_count = g_vision_parser.frame_ok_count;
    diagnostics->crc_error_count = g_vision_parser.crc_error_count;
    diagnostics->format_error_count = g_vision_parser.format_error_count;
    diagnostics->valid_frame_count = g_vision_valid_frame_count;
    diagnostics->timeout_count = g_vision_timeout_count;
    diagnostics->last_valid_tick_ms = g_vision_last_valid_tick_ms;
    diagnostics->state = g_vision_state;
}
