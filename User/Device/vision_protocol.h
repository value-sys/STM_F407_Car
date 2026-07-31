#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdint.h>

#define VISION_FRAME_HEAD_1       0xAAU
#define VISION_FRAME_HEAD_2       0x55U
#define VISION_PROTOCOL_VERSION   0x01U
#define VISION_TYPE_BALL_POSITION 0x01U

#define VISION_FRAME_LENGTH       17U
#define VISION_PAYLOAD_LENGTH     10U
#define VISION_CONFIDENCE_MIN     10U
/* Absolute pipe coordinate: 1 mm to 250 mm, unit 1 mm. */
#define VISION_POSITION_MIN_MM    (1)
#define VISION_POSITION_MAX_MM    (250)

typedef struct
{
    uint16_t sequence;
    int16_t ball_position_mm;
    uint8_t valid;
    uint8_t confidence;
    uint32_t timestamp_ms;
    uint32_t receive_tick_ms;
    uint32_t receive_count;
} stVisionBallData;

typedef struct
{
    uint8_t frame[VISION_FRAME_LENGTH];
    uint8_t index;
    uint8_t expected_length;
    uint32_t frame_ok_count;
    uint32_t crc_error_count;
    uint32_t format_error_count;
} stVisionParser;

uint16_t usVisionCrc16Ccitt(const uint8_t *data, uint16_t length);
void vVisionParserInit(stVisionParser *parser);
uint8_t ucVisionParserInput(stVisionParser *parser,
                            uint8_t byte,
                            stVisionBallData *output);

#endif
