#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define OLED_WIDTH 128U
#define OLED_HEIGHT 64U

HAL_StatusTypeDef OLED_Init(I2C_HandleTypeDef *hi2c, uint8_t address_7bit);
HAL_StatusTypeDef OLED_UpdateScreen(void);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t column, uint8_t page);
void OLED_WriteChar(char character);
void OLED_WriteString(const char *text);
uint8_t OLED_IsReady(void);

#endif
