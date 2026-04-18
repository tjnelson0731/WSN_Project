/**
 * main.c — WSN position receive test (no drive)
 *
 * The STM32 sits stationary, connected to the robot_esp via USART1.
 * It continuously sends temperature readings to the ESP (which forwards them
 * over Zigbee to the coordinator) and prints every position frame it receives
 * from the ESP to the ST-Link VCP on USART2 so it can be read in a terminal.
 *
 * Use this to verify the full chain:
 *   LM34 → USART1 → robot_esp → Zigbee → coordinator  (temperature path)
 *   anchors → Zigbee → robot_esp → trilateration → USART1 → here  (position path)
 *
 * Peripherals:
 *   USART2  PA2/PA3  115200 8N1  ST-Link VCP debug output      (uart.c)
 *   USART1  PA9/PA10 115200 8N1  ESP32-C6 link                  (uart1.c)
 *   ADC1    PA0      LM34 VOUT                                   (lm34.c)
 *   TIM6             µs/ms delays                                (clock.c)
 *
 * Wiring (STM32 ↔ ESP32-C6):
 *   STM32 PA9  (USART1_TX)  →  ESP32-C6 GPIO11 (RX)
 *   STM32 PA10 (USART1_RX)  ←  ESP32-C6 GPIO10 (TX)
 *   STM32 GND               —  ESP32-C6 GND
 */

#include <string.h>
#include "stm32l452xx.h"
#include "clock.h"
#include "lm34.h"
#include "uart.h"
#include "uart1.h"

/* =========================================================================
 * Timing
 * ========================================================================= */

#define TEMP_INTERVAL_MS 4000U    /* how often to send temperature (ms)      */
#define TICK_MS            10U    /* main loop tick period (ms)               */

/* =========================================================================
 * UART1 frame protocol
 * ========================================================================= */

#define FRAME_START      0xAAu
#define FRAME_TYPE_TEMP  0x01u    /* STM32 → ESP: temperature                */
#define FRAME_TYPE_POS   0x02u    /* ESP → STM32: computed 2D position       */

/* =========================================================================
 * Temperature TX
 * ========================================================================= */

static void send_temp_frame(int16_t tenths_f)
{
    uint8_t lo = (uint8_t)((uint16_t)tenths_f & 0xFFu);
    uint8_t hi = (uint8_t)((uint16_t)tenths_f >> 8u);
    uint8_t frame[6] = {
        FRAME_START,
        FRAME_TYPE_TEMP,
        2u,
        lo,
        hi,
        (uint8_t)(FRAME_TYPE_TEMP ^ 2u ^ lo ^ hi)
    };
    UART1_SendBytes(frame, sizeof(frame));
}

/* =========================================================================
 * Debug print helpers (USART2 — no printf available)
 * ========================================================================= */

static void print_u32(uint32_t v)
{
    char buf[10];
    int  n = 0;
    if (v == 0u) { UART_SendChar('0'); return; }
    while (v) { buf[n++] = (char)('0' + v % 10u); v /= 10u; }
    while (n--) UART_SendChar(buf[n]);
}

static void print_int16(int16_t v)
{
    if (v < 0) { UART_SendChar('-'); print_u32((uint32_t)(-v)); }
    else        { print_u32((uint32_t)v); }
}

static void print_tenths(int32_t t)
{
    if (t < 0) { UART_SendChar('-'); t = -t; }
    print_u32((uint32_t)(t / 10));
    UART_SendChar('.');
    UART_SendChar((char)('0' + t % 10));
}

/* =========================================================================
 * Received frame handler
 * ========================================================================= */

#define POS_UNCHANGED  INT16_MIN   /* sentinel — no valid position yet */
static int16_t s_last_x = POS_UNCHANGED;
static int16_t s_last_y = POS_UNCHANGED;

static void process_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    switch (type) {

    case FRAME_TYPE_POS: {
        if (len < 4u) {
            UART_SendString("[POS] bad length\r\n");
            return;
        }
        int16_t x, y;
        memcpy(&x, &payload[0], sizeof(int16_t));
        memcpy(&y, &payload[2], sizeof(int16_t));
        if (x == s_last_x && y == s_last_y) break;   /* same as last frame */
        s_last_x = x;
        s_last_y = y;
        UART_SendString("[POS] x=");
        print_int16(x);
        UART_SendString(" cm  y=");
        print_int16(y);
        UART_SendString(" cm\r\n");
        break;
    }

    default:
        UART_SendString("[RX] unknown type 0x");
        UART_SendChar("0123456789ABCDEF"[type >> 4u]);
        UART_SendChar("0123456789ABCDEF"[type & 0xFu]);
        UART_SendString("\r\n");
        break;
    }
}

/* =========================================================================
 * UART1 RX — non-blocking frame receiver
 * ========================================================================= */

typedef enum { RX_IDLE, RX_TYPE, RX_LEN, RX_PAYLOAD, RX_CHECKSUM } rx_state_t;

static rx_state_t s_rx_state = RX_IDLE;
static uint8_t    s_rx_type  = 0;
static uint8_t    s_rx_len   = 0;
static uint8_t    s_rx_idx   = 0;
static uint8_t    s_rx_cksum = 0;
static uint8_t    s_rx_buf[64];

static void uart1_rx_poll(void)
{
    /* Bytes are buffered by USART1_IRQHandler in uart1.c.
     * This function drains the ring buffer and advances the frame state machine. */
    while (UART1_DataAvailable()) {
        uint8_t b = UART1_ReceiveByte();
        switch (s_rx_state) {

        case RX_IDLE:
            if (b == FRAME_START) s_rx_state = RX_TYPE;
            break;

        case RX_TYPE:
            s_rx_type  = b;
            s_rx_cksum = b;
            s_rx_state = RX_LEN;
            break;

        case RX_LEN:
            s_rx_len   = b;
            s_rx_cksum ^= b;
            s_rx_idx   = 0u;
            s_rx_state = (b == 0u) ? RX_CHECKSUM : RX_PAYLOAD;
            break;

        case RX_PAYLOAD:
            if (s_rx_idx < sizeof(s_rx_buf)) {
                s_rx_buf[s_rx_idx] = b;
            }
            s_rx_idx++;
            s_rx_cksum ^= b;
            if (s_rx_idx >= s_rx_len) {
                s_rx_state = RX_CHECKSUM;
            }
            break;

        case RX_CHECKSUM:
            if (b == s_rx_cksum) {
                process_frame(s_rx_type, s_rx_buf, s_rx_len);
            } else {
                UART_SendString("[RX] checksum error\r\n");
            }
            s_rx_state = RX_IDLE;
            break;
        }
    }
}

/* =========================================================================
 * main
 * ========================================================================= */

int main(void)
{
    SystemClock_Config();
    UART_Init(115200U);
    UART1_Init(115200U);
    LM34_Init();

    UART_SendString("\r\n=== WSN position RX test ===\r\n");
    UART_SendString("Waiting for position frames from robot_esp...\r\n\r\n");

    uint32_t temp_ms = TEMP_INTERVAL_MS;   /* send on first tick */

    while (1) {
        delay_ms(TICK_MS);
        uart1_rx_poll();

        temp_ms += TICK_MS;
        if (temp_ms >= TEMP_INTERVAL_MS) {
            temp_ms = 0u;
            int16_t temp = LM34_Read();
            send_temp_frame(temp);
            UART_SendString("[TEMP] TX: ");
            print_tenths((int32_t)temp);
            UART_SendString(" F\r\n");
        }
    }
}
