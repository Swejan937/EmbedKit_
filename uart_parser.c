#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define SOF_MARKER         0xAA
#define MAX_PAYLOAD_LEN    16U

typedef enum
{
    STATE_WAIT_SOF = 0,
    STATE_CMD,
    STATE_LEN,
    STATE_PAYLOAD,
    STATE_CHECKSUM
} ParserState;

typedef struct
{
    ParserState state;

    uint8_t cmd;
    uint8_t len;
    uint8_t payload[MAX_PAYLOAD_LEN];
    uint8_t checksum;

    uint8_t payload_index;
    uint32_t timeout_ms;
    uint32_t last_byte_time;
} UartParser;

static void parser_reset(UartParser *parser)
{
    parser->state = STATE_WAIT_SOF;
    parser->cmd = 0U;
    parser->len = 0U;
    parser->checksum = 0U;
    parser->payload_index = 0U;
}

static void parser_init(UartParser *parser, uint32_t timeout_ms)
{
    parser_reset(parser);
    parser->timeout_ms = timeout_ms;
    parser->last_byte_time = 0U;
}

static int parser_feed_byte(UartParser *parser, uint8_t byte, uint32_t timestamp_ms)
{
    /* Timeout check MUST happen before processing byte */
    if ((parser->timeout_ms > 0U) && (parser->state != STATE_WAIT_SOF))
    {
        uint32_t gap = timestamp_ms - parser->last_byte_time;
        if (gap >= parser->timeout_ms)
        {
            parser_reset(parser);
            return -2;
        }
    }

    switch (parser->state)
    {
        case STATE_WAIT_SOF:
            if (byte == SOF_MARKER)
            {
                parser->state = STATE_CMD;
            }
            break;

        case STATE_CMD:
            parser->cmd = byte;
            parser->checksum = byte;
            parser->state = STATE_LEN;
            break;

        case STATE_LEN:
            if (byte > MAX_PAYLOAD_LEN)
            {
                parser_reset(parser);
                break;
            }
            parser->len = byte;
            parser->checksum ^= byte;
            parser->payload_index = 0U;

            if (parser->len == 0U)
            {
                parser->state = STATE_CHECKSUM;
            }
            else
            {
                parser->state = STATE_PAYLOAD;
            }
            break;

        case STATE_PAYLOAD:
            parser->payload[parser->payload_index++] = byte;
            parser->checksum ^= byte;

            if (parser->payload_index >= parser->len)
            {
                parser->state = STATE_CHECKSUM;
            }
            break;

        case STATE_CHECKSUM:
            if (byte == parser->checksum)
            {
                parser_reset(parser);
                parser->last_byte_time = timestamp_ms;
                return 1;
            }
            else
            {
                parser_reset(parser);
                parser->last_byte_time = timestamp_ms;
                return -1;
            }

        default:
            parser_reset(parser);
            break;
    }
    parser->last_byte_time = timestamp_ms;
    return 0;
}

static void print_frame(uint8_t cmd,uint8_t len,const uint8_t payload[])
{
    uint8_t i;
    printf("FRAME OK CMD=0x%02X LEN=%u PAYLOAD=[",cmd,len);

    for (i = 0U; i < len; i++)
    {
        printf("%02X", payload[i]);
        if (i < (len - 1U))
        {
            printf(" ");
        }
    }
    printf("]\n");
}

static void feed_stream(UartParser *parser, const uint8_t bytes[],const uint32_t times[],uint32_t count)
{
    uint32_t i;
    uint8_t frame_cmd = 0U;
    uint8_t frame_len = 0U;
    uint8_t frame_payload[MAX_PAYLOAD_LEN];

    for (i = 0U; i < count; i++)
    {
        if (parser->state == STATE_CMD)
        {
            frame_cmd = bytes[i];
        }

        if (parser->state == STATE_LEN)
        {
            frame_len = bytes[i];
        }

        if (parser->state == STATE_PAYLOAD)
        {
            frame_payload[parser->payload_index] = bytes[i];
        }

        uint8_t result = parser_feed_byte(parser,bytes[i],times[i]);
        printf("t=%3ums byte=0x%02X -> ",times[i],bytes[i]);

        if (0 == result)
        {
            printf("receiving...\n");
        }

        else if (-2 == result)
        {
            printf("TIMEOUT\n");
            result = parser_feed_byte(parser, bytes[i],times[i]);
            printf("t=%3ums byte=0x%02X -> receiving... (re-fed after reset)\n",times[i],bytes[i]);
        }

        else if (-1 == result)
        {
            printf("CHECKSUM ERROR\n");
        }

        else if (1 == result )
        {
            print_frame(frame_cmd,frame_len,frame_payload);
        }
    }
}

int main(void)
{
    /* Test 1 */
    printf("\n===== TEST 1 : CLEAN VALID FRAME =====\n");
    UartParser parser1;
    parser_init(&parser1, 50U);
    uint8_t bytes1[] = { 0xAA,0x01,0x03,0x10,0x20,0x30,0x02};
    uint32_t times1[] ={ 0,5,10,15,20,25,30};
    feed_stream(&parser1,bytes1,times1,sizeof(bytes1));

    /* Test 2 */
    printf("\n===== TEST 2 : TIMEOUT + RECOVERY =====\n");
    UartParser parser2;
    parser_init(&parser2, 50U);
    uint8_t bytes2[]  = { 0xAA,0x01,0x03,0x10,0xAA,0xAA,0x05,0x01,0x7F,0x7B };
    uint32_t times2[] = { 0,5,10,15,200,200,205,210,215,220 };
    feed_stream(&parser2,bytes2,times2,sizeof(bytes2));

    /* Test 3 */
    printf("\n===== TEST 3 : TWO VALID FRAMES =====\n");
    UartParser parser3;
    parser_init(&parser3, 50U);
    uint8_t bytes3[] ={ 0xAA,0x03,0x01,0x55,0x57,0xAA,0x04,0x02,0xAA,0xBB,0x17 };
    uint32_t times3[] ={ 0,5,10,15,20,25,30,35,40,45,50 };
    feed_stream(&parser3, bytes3, times3, sizeof(bytes3));

    /* Test 4 */
    printf("\n===== TEST 4 : TIMEOUT DISABLED =====\n");
    UartParser parser4;
    parser_init(&parser4, 0U);
    uint8_t bytes4[] ={ 0xAA,0x01,0x03,0x10,0xAA,0xAA,0x05,0x01,0x7F,0x7B };
    uint32_t times4[] ={ 0,5,10,15,200,200,205,210,215,220 };
    feed_stream(&parser4, bytes4, times4, sizeof(bytes4));
    
    return 0;
}