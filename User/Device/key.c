#include "key.h"

#include "main.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} key_pin_t;

static const key_pin_t g_key_pins[4] = {
    {key1_GPIO_Port, key1_Pin}, {key2_GPIO_Port, key2_Pin},
    {key3_GPIO_Port, key3_Pin}, {key4_GPIO_Port, key4_Pin}
};
static uint8_t g_key_raw;
static uint8_t g_key_stable;
static uint8_t g_key_debounce_count;

static uint8_t key_read_raw(void)
{
    uint8_t state = 0U;
    for (uint8_t index = 0U; index < 4U; ++index) {
        if (HAL_GPIO_ReadPin(g_key_pins[index].port, g_key_pins[index].pin) == GPIO_PIN_RESET) {
            state |= (uint8_t)(1U << index);
        }
    }
    return state;
}

void Key_Init(void)
{
    g_key_raw = key_read_raw();
    g_key_stable = g_key_raw;
    g_key_debounce_count = 0U;
}

uint8_t Key_Scan(void)
{
    uint8_t raw = key_read_raw();
    uint8_t pressed = 0U;

    if (raw != g_key_raw) {
        g_key_raw = raw;
        g_key_debounce_count = 0U;
        return 0U;
    }
    if (g_key_debounce_count < 2U) {
        ++g_key_debounce_count;
        return 0U;
    }
    pressed = (uint8_t)(raw & (uint8_t)~g_key_stable);
    g_key_stable = raw;
    return pressed;
}

uint8_t Key_IsDown(uint8_t key_index)
{
    if (key_index >= 4U) return 0U;
    return (uint8_t)((g_key_stable >> key_index) & 1U);
}

uint8_t Key_IsDownRaw(uint8_t key_index)
{
    if (key_index >= 4U) return 0U;
    return HAL_GPIO_ReadPin(g_key_pins[key_index].port,
                            g_key_pins[key_index].pin) == GPIO_PIN_RESET;
}
