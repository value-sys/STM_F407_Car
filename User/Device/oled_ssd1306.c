#include "oled_ssd1306.h"

#include <stddef.h>
#include <string.h>

#define OLED_I2C_TIMEOUT_MS 100U
#define OLED_COMMAND_CONTROL 0x00U
#define OLED_DATA_CONTROL 0x40U

static I2C_HandleTypeDef *g_oled_i2c;
static uint8_t g_oled_address;
static uint8_t g_oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8U];
static uint8_t g_oled_ready;
static uint8_t g_oled_column;
static uint8_t g_oled_page;

static HAL_StatusTypeDef oled_write_commands(const uint8_t *commands, uint8_t count)
{
    uint8_t packet[17];

    if (commands == NULL || count == 0U || count > 16U) {
        return HAL_ERROR;
    }
    packet[0] = OLED_COMMAND_CONTROL;
    memcpy(&packet[1], commands, count);
    return HAL_I2C_Master_Transmit(g_oled_i2c, g_oled_address, packet,
                                   (uint16_t)count + 1U, OLED_I2C_TIMEOUT_MS);
}

static const uint8_t *oled_glyph(char character)
{
    static const uint8_t blank[5] = {0U, 0U, 0U, 0U, 0U};
    static const uint8_t question[5] = {0x02U, 0x01U, 0x51U, 0x09U, 0x06U};
    static const uint8_t digits[10][5] = {
        {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU}, {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
        {0x42U, 0x61U, 0x51U, 0x49U, 0x46U}, {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
        {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U}, {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
        {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U}, {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
        {0x36U, 0x49U, 0x49U, 0x49U, 0x36U}, {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}
    };
    static const uint8_t letters[26][5] = {
        {0x7EU,0x11U,0x11U,0x11U,0x7EU},{0x7FU,0x49U,0x49U,0x49U,0x36U},
        {0x3EU,0x41U,0x41U,0x41U,0x22U},{0x7FU,0x41U,0x41U,0x22U,0x1CU},
        {0x7FU,0x49U,0x49U,0x49U,0x41U},{0x7FU,0x09U,0x09U,0x09U,0x01U},
        {0x3EU,0x41U,0x49U,0x49U,0x7AU},{0x7FU,0x08U,0x08U,0x08U,0x7FU},
        {0x00U,0x41U,0x7FU,0x41U,0x00U},{0x20U,0x40U,0x41U,0x3FU,0x01U},
        {0x7FU,0x08U,0x14U,0x22U,0x41U},{0x7FU,0x40U,0x40U,0x40U,0x40U},
        {0x7FU,0x02U,0x0CU,0x02U,0x7FU},{0x7FU,0x04U,0x08U,0x10U,0x7FU},
        {0x3EU,0x41U,0x41U,0x41U,0x3EU},{0x7FU,0x09U,0x09U,0x09U,0x06U},
        {0x3EU,0x41U,0x51U,0x21U,0x5EU},{0x7FU,0x09U,0x19U,0x29U,0x46U},
        {0x46U,0x49U,0x49U,0x49U,0x31U},{0x01U,0x01U,0x7FU,0x01U,0x01U},
        {0x3FU,0x40U,0x40U,0x40U,0x3FU},{0x1FU,0x20U,0x40U,0x20U,0x1FU},
        {0x7FU,0x20U,0x18U,0x20U,0x7FU},{0x63U,0x14U,0x08U,0x14U,0x63U},
        {0x07U,0x08U,0x70U,0x08U,0x07U},{0x61U,0x51U,0x49U,0x45U,0x43U}
    };
    static const uint8_t colon[5] = {0U, 0x36U, 0x36U, 0U, 0U};
    static const uint8_t dash[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
    static const uint8_t dot[5] = {0U, 0x60U, 0x60U, 0U, 0U};

    if (character == ' ') return blank;
    if (character >= '0' && character <= '9') return digits[(uint8_t)character - '0'];
    if (character >= 'A' && character <= 'Z') return letters[(uint8_t)character - 'A'];
    if (character == ':') return colon;
    if (character == '-') return dash;
    if (character == '.') return dot;
    return question;
}

HAL_StatusTypeDef OLED_Init(I2C_HandleTypeDef *hi2c, uint8_t address_7bit)
{
    static const uint8_t init_1[] = {
        0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
        0x8DU, 0x14U, 0x20U, 0x00U, 0xA1U, 0xC8U, 0xDAU, 0x12U
    };
    static const uint8_t init_2[] = {
        0x81U, 0xCFU, 0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U, 0xA6U, 0xAFU
    };
    HAL_StatusTypeDef status;

    if (hi2c == NULL || address_7bit > 0x7FU) return HAL_ERROR;
    g_oled_i2c = hi2c;
    g_oled_address = (uint8_t)(address_7bit << 1U);
    g_oled_ready = 0U;
    status = HAL_I2C_IsDeviceReady(g_oled_i2c, g_oled_address, 2U, OLED_I2C_TIMEOUT_MS);
    if (status != HAL_OK) return status;
    status = oled_write_commands(init_1, sizeof(init_1));
    if (status != HAL_OK) return status;
    status = oled_write_commands(init_2, sizeof(init_2));
    if (status != HAL_OK) return status;
    g_oled_ready = 1U;
    OLED_Clear();
    return OLED_UpdateScreen();
}

HAL_StatusTypeDef OLED_UpdateScreen(void)
{
    uint8_t packet[OLED_WIDTH + 1U];

    if (g_oled_i2c == NULL || g_oled_ready == 0U) return HAL_ERROR;
    packet[0] = OLED_DATA_CONTROL;
    for (uint8_t page = 0U; page < OLED_HEIGHT / 8U; ++page) {
        const uint8_t commands[3] = {(uint8_t)(0xB0U + page), 0x00U, 0x10U};
        HAL_StatusTypeDef status = oled_write_commands(commands, sizeof(commands));
        if (status != HAL_OK) return status;
        memcpy(&packet[1], &g_oled_buffer[page * OLED_WIDTH], OLED_WIDTH);
        status = HAL_I2C_Master_Transmit(g_oled_i2c, g_oled_address, packet,
                                         sizeof(packet), OLED_I2C_TIMEOUT_MS);
        if (status != HAL_OK) return status;
    }
    return HAL_OK;
}

void OLED_Clear(void)
{
    memset(g_oled_buffer, 0, sizeof(g_oled_buffer));
    g_oled_column = 0U;
    g_oled_page = 0U;
}

void OLED_SetCursor(uint8_t column, uint8_t page)
{
    g_oled_column = column < OLED_WIDTH ? column : OLED_WIDTH - 1U;
    g_oled_page = page < OLED_HEIGHT / 8U ? page : OLED_HEIGHT / 8U - 1U;
}

void OLED_WriteChar(char character)
{
    const uint8_t *glyph = oled_glyph(character);
    if (g_oled_column > OLED_WIDTH - 6U) {
        g_oled_column = 0U;
        g_oled_page = (uint8_t)((g_oled_page + 1U) % (OLED_HEIGHT / 8U));
    }
    memcpy(&g_oled_buffer[g_oled_page * OLED_WIDTH + g_oled_column], glyph, 5U);
    g_oled_buffer[g_oled_page * OLED_WIDTH + g_oled_column + 5U] = 0U;
    g_oled_column = (uint8_t)(g_oled_column + 6U);
}

void OLED_WriteString(const char *text)
{
    if (text == NULL) return;
    while (*text != '\0') OLED_WriteChar(*text++);
}

uint8_t OLED_IsReady(void)
{
    return g_oled_ready;
}
