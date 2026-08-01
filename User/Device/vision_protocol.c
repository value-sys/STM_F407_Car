#include "vision_protocol.h"

#include <stddef.h>

static void vVisionParserReset(stVisionParser *parser)
{
    parser->index = 0U;
    parser->expected_length = 0U;
}

uint16_t usVisionCrc16Ccitt(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    if (data == NULL)
    {
        return 0U;
    }

    for (uint16_t i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1U) ^ 0x1021U)
                : (uint16_t)(crc << 1U);
        }
    }

    return crc;
}

void vVisionParserInit(stVisionParser *parser)
{
    if (parser != NULL)
    {
        *parser = (stVisionParser){0};
    }
}

uint8_t ucVisionParserInput(stVisionParser *parser,
                            uint8_t byte,
                            stVisionBallData *output)
{
    uint16_t received_crc;
    uint16_t calculated_crc;
    int16_t position;

    if (parser == NULL || output == NULL)
    {
        return 0U;
    }

    if (parser->index == 0U)
    {
        if (byte == VISION_FRAME_HEAD_1)
        {
            parser->frame[0] = byte;
            parser->index = 1U;
        }
        return 0U;
    }

    if (parser->index == 1U)
    {
        if (byte == VISION_FRAME_HEAD_2)
        {
            parser->frame[1] = byte;
            parser->index = 2U;
        }
        else if (byte != VISION_FRAME_HEAD_1)
        {
            vVisionParserReset(parser);
        }
        return 0U;
    }

    parser->frame[parser->index++] = byte;

    if (parser->index == 5U)
    {
        if (parser->frame[2] != VISION_PROTOCOL_VERSION ||
            parser->frame[3] != VISION_TYPE_BALL_POSITION ||
            parser->frame[4] != VISION_PAYLOAD_LENGTH)
        {
            ++parser->format_error_count;
            vVisionParserReset(parser);
            return 0U;
        }
        parser->expected_length = VISION_FRAME_LENGTH;
    }

    if (parser->expected_length == 0U ||
        parser->index < parser->expected_length)
    {
        return 0U;
    }

    received_crc = (uint16_t)parser->frame[15] |
                   ((uint16_t)parser->frame[16] << 8U);
    calculated_crc = usVisionCrc16Ccitt(&parser->frame[2], 13U);
    if (received_crc != calculated_crc)
    {
        ++parser->crc_error_count;
        vVisionParserReset(parser);
        return 0U;
    }

    position = (int16_t)((uint16_t)parser->frame[7] |
                         ((uint16_t)parser->frame[8] << 8U));
    if (parser->frame[9] > 1U || parser->frame[10] > 100U ||
        position < VISION_POSITION_MIN_MM ||
        position > VISION_POSITION_MAX_MM)
    {
        ++parser->format_error_count;
        vVisionParserReset(parser);
        return 0U;
    }

    output->sequence = (uint16_t)parser->frame[5] |
                       ((uint16_t)parser->frame[6] << 8U);
    output->ball_position_mm = position;
    output->valid = parser->frame[9];
    output->confidence = parser->frame[10];
    output->timestamp_ms = (uint32_t)parser->frame[11] |
                           ((uint32_t)parser->frame[12] << 8U) |
                           ((uint32_t)parser->frame[13] << 16U) |
                           ((uint32_t)parser->frame[14] << 24U);
    ++parser->frame_ok_count;
    vVisionParserReset(parser);
    return 1U;
}
