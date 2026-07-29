#ifndef KEY_H
#define KEY_H

#include <stdint.h>

#define KEY_EVENT_1 (1U << 0)
#define KEY_EVENT_2 (1U << 1)
#define KEY_EVENT_3 (1U << 2)
#define KEY_EVENT_4 (1U << 3)

void Key_Init(void);
uint8_t Key_Scan(void);
uint8_t Key_IsDown(uint8_t key_index);
uint8_t Key_IsDownRaw(uint8_t key_index);

#endif
