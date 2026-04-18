/**
 * main.c — Robot ESP32-C6 Zigbee Bridge Node
 *
 * Zigbee role: Router (ZR).
 *
 * This node bridges the STM32L452 robot controller to the Zigbee mesh:
 *
 *   STM32 (USART1) ←→ ESP32-C6 (UART1) ←→ Zigbee mesh ←→ Coordinator
 *
 * ── Upstream (STM32 → ESP → coordinator) ──────────────────────────────────
 *   The STM32 sends a framed temperature reading over UART.
 *   This node forwards it to the coordinator as WSN Cmd 0x01.
 *
 * ── Downstream (anchors → ESP → STM32) ───────────────────────────────────
 *   Routing anchors periodically broadcast WSN Cmd 0x00 beacons containing
 *   their ID, known (x,y) position, and TX power.
 *   This node collects those beacons, records the RSSI (see TODO below), and
 *   sends a compact RSSI report to the STM32 over UART every second.
 *   The STM32 runs trilateration using the RSSI + anchor positions.
 *
 * ── UART frame protocol (115200 8N1, both directions) ─────────────────────
 *   [0xAA][TYPE][LEN][PAYLOAD × LEN][CHECKSUM]
 *   CHECKSUM = TYPE XOR LEN XOR payload[0] XOR … XOR payload[LEN-1]
 *
 *   TYPE 0x01  STM32 → ESP   Temperature report
 *              LEN  = 2
 *              PAYLOAD = int16_t tenths-of-°F, little-endian
 *              Example: 734 = 73.4 °F → bytes [0xDE, 0x02]
 *
 *   TYPE 0x02  ESP → STM32   Computed 2D position
 *              LEN  = 4
 *              PAYLOAD = x_cm (int16_t LE), y_cm (int16_t LE)
 *              Sent only when ≥ 3 fresh anchors are visible.
 *
 * ── UART wiring (STM32 USART1 ↔ ESP32-C6 UART1) ──────────────────────────
 *   GPIO10 (ESP TX) → STM32 PA10 (USART1_RX)
 *   GPIO11 (ESP RX) ← STM32 PA9  (USART1_TX)
 *   Both sides: 115200 8N1, no hardware flow control.
 *
 * ── RSSI note ─────────────────────────────────────────────────────────────
 *   Per-frame RSSI is not directly exposed in the esp-zigbee-lib ~1.6.0 ZCL
 *   action callback.  A placeholder value is used for now; see the TODO in
 *   handle_beacon() for implementation options.
 *
 * Build:  idf.py build
 * Flash:  idf.py -p <PORT> flash monitor
 * Target: ESP32-C6 DevKit-C1, ESP-IDF ≥5.1, esp-zigbee-lib ~1.6.0
 */

#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_zigbee_core.h"
#include "nwk/esp_zigbee_nwk.h"

/* =========================================================================
 * Configuration
 * ========================================================================= */

/* ---- Zigbee --------------------------------------------------------------- */
#define TAG                 "ROBOT_ESP"
#define MY_ENDPOINT_ID      1
#define INSTALLCODE_POLICY  false

#define WSN_CLUSTER_ID      0xFC00
#define WSN_CMD_BEACON      0x00
#define WSN_CMD_TEMPERATURE 0x01

/* Coordinator always has short address 0x0000 in a Zigbee PAN */
#define COORD_SHORT_ADDR    0x0000
#define COORD_ENDPOINT      1

/* ---- UART ----------------------------------------------------------------- */
/*
 * UART_NUM_1 is used for the STM32 link.
 * UART_NUM_0 is reserved for the ESP-IDF console (USB-UART bridge on DevKit).
 *
 * GPIO10/11 are safe general-purpose IO on the ESP32-C6 DevKit-C1:
 *   they are not strapping pins, not JTAG, and not connected to the USB bridge.
 * Change these defines if you use different GPIOs.
 *
 *   GPIO10 (ESP TX) → STM32 PA10 (USART1_RX)
 *   GPIO11 (ESP RX) ← STM32 PA9  (USART1_TX)
 */
#define UART_STM32          UART_NUM_1
#define UART_STM32_TX_GPIO  10
#define UART_STM32_RX_GPIO  11
#define UART_BAUD_RATE      115200
#define UART_RX_BUF_SIZE    256           /* driver ring-buffer size (bytes)  */
#define UART_TX_BUF_SIZE    256

/* ---- UART frame protocol -------------------------------------------------- */
#define FRAME_START         0xAAu
#define FRAME_TYPE_TEMP     0x01u         /* STM32 → ESP: temperature          */
#define FRAME_TYPE_POS      0x02u         /* ESP → STM32: computed 2D position */

/* ---- Log-distance path-loss model ----------------------------------------- */
/*
 * Free-space path loss at 1 m reference distance for 2.4 GHz:
 *   PL(1 m) = 20·log10(4π·f/c) ≈ 40.0 dB
 *
 * Indoor path-loss exponent:
 *   n = 2.0  — free space / line-of-sight
 *   n = 2.7  — typical indoor with light obstruction (used here)
 *   n = 3.5  — dense indoor / many walls
 *
 * Adjust PATHLOSS_EXP empirically: measure RSSI at a known distance from each
 * anchor in your actual environment and solve for n.
 *
 * Distance formula:
 *   PL_measured = TX_power_dBm − RSSI_dBm
 *   d = d0 · 10^( (PL_measured − PATHLOSS_D0_DB) / (10 · PATHLOSS_EXP) )
 *   with d0 = 1 m
 */
#define PATHLOSS_D0_DB      40.0f         /* path loss at d0 = 1 m, 2.4 GHz   */
#define PATHLOSS_EXP        2.7f          /* indoor path-loss exponent         */

/* ---- Anchor table --------------------------------------------------------- */
/*
 * Safety cap on the number of simultaneously-tracked anchors.  This is NOT
 * a deployment parameter — you do not change it when you add anchors.
 * The system automatically uses however many anchors are currently broadcasting
 * (≥ 3 required for 2D trilateration).  Set this high enough that it is never
 * a practical limit; 16 exceeds any reasonable classroom deployment.
 */
#define ANCHOR_TABLE_SIZE   16

/*
 * An anchor entry is considered stale if no beacon was received within
 * ANCHOR_STALE_US microseconds.  Stale entries are excluded from position
 * computation and not sent to the STM32.
 */
#define ANCHOR_STALE_US     (30LL * 1000 * 1000)   /* 30 seconds in µs        */

/* ---- Periodic intervals -------------------------------------------------- */
/* How often to compute and send a position fix to the STM32 (via UART) */
#define POS_REPORT_INTERVAL_MS   1000

/* How often to check for a new temperature reading and send it to coordinator */
#define TEMP_SEND_INTERVAL_MS    2000

/* =========================================================================
 * Types
 * ========================================================================= */

/*
 * Packed beacon payload — must match the layout defined in routing_anchor and
 * gateway_anchor.  __attribute__((packed)) prevents padding between fields.
 */
typedef struct {
    uint8_t anchor_id;
    int16_t x_cm;
    int16_t y_cm;
    int16_t z_cm;
    int8_t  tx_power_dbm;
} __attribute__((packed)) wsn_beacon_payload_t;

/* Internal anchor table entry — keyed on anchor_id from the beacon payload */
typedef struct {
    bool    valid;             /* true if this slot holds live data           */
    uint8_t anchor_id;         /* application-layer ID set per unit at flash  */
    int16_t x_cm;
    int16_t  y_cm;
    int16_t  z_cm;
    int8_t   tx_power_dbm;
    int8_t   rssi_dbm;
    float    dist_m;           /* path-loss distance estimate in metres       */
    int64_t  last_seen_us;    /* from esp_timer_get_time(); 0 if never seen  */
} anchor_entry_t;

/* UART receive state machine states */
typedef enum {
    RX_IDLE,        /* waiting for 0xAA start byte */
    RX_TYPE,        /* next byte is the frame TYPE  */
    RX_LEN,         /* next byte is the payload LEN */
    RX_PAYLOAD,     /* accumulating LEN payload bytes */
    RX_CHECKSUM,    /* next byte is the CHECKSUM */
} uart_rx_state_t;

/* =========================================================================
 * Global state
 * ========================================================================= */

/*
 * Anchor table — written by the Zigbee action callback (in zigbee_task),
 * read by the ZBOSS scheduler alarm (also in zigbee_task).
 * Both accesses are in the same FreeRTOS task, so no mutex is needed.
 */
static anchor_entry_t g_anchors[ANCHOR_TABLE_SIZE];

/*
 * Temperature shared between the UART RX task (writer) and the Zigbee
 * scheduler alarm (reader).  Both fields are volatile to prevent the
 * compiler from caching them in registers across task-switch points.
 *
 * Safety: int16_t reads/writes are atomic on 32-bit RISC-V when the variable
 * is naturally aligned (guaranteed by the compiler for static globals), so
 * no mutex is needed for this simple single-producer / single-consumer pattern.
 */
static volatile int16_t g_last_temp_tenths_f = 0;
static volatile bool    g_temp_updated       = false;

/* Set to true once the Zigbee network has been joined */
static volatile bool    g_network_joined = false;

/* =========================================================================
 * UART helpers
 * ========================================================================= */

/**
 * uart_init() — configure UART1 for STM32 communication.
 *
 * ESP-IDF UART driver notes:
 *   uart_driver_install(): installs the driver with internal ring buffers.
 *     RX buf size should be >= 2× the max frame size to avoid data loss.
 *     Passing 0 for the event queue size disables the event queue; we poll
 *     instead with uart_read_bytes().
 *   uart_param_config(): sets baud rate, data bits, parity, stop bits.
 *   uart_set_pin():      maps UART signals to GPIO pins.
 *     UART_PIN_NO_CHANGE leaves RTS/CTS unconnected (no hardware flow ctrl).
 */
static void uart_init(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(
            UART_STM32,
            UART_RX_BUF_SIZE, UART_TX_BUF_SIZE,
            0, NULL,            /* event queue: disabled */
            0));
    ESP_ERROR_CHECK(uart_param_config(UART_STM32, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(
            UART_STM32,
            UART_STM32_TX_GPIO, UART_STM32_RX_GPIO,
            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART1 ready: TX=GPIO%d  RX=GPIO%d  %d baud",
             UART_STM32_TX_GPIO, UART_STM32_RX_GPIO, UART_BAUD_RATE);
}

/**
 * compute_position() — least-squares 2D trilateration from all fresh anchors.
 *
 * Linearises the range equations by subtracting the last anchor's equation
 * from each of the others, yielding  A·[x,y]^T = b.  Solves via:
 *
 *   x = (A^T A)^{-1} A^T b
 *
 * A^T A is always 2×2 regardless of anchor count, so inversion is
 * just a determinant and four multiplies — no matrix library needed.
 *
 * Anchor z heights are accounted for: each 3D range d_i is projected to
 * the 2D ground-plane radius  r_i^2 = d_i^2 − z_i^2  (clamped ≥ 0),
 * assuming the robot travels at z = 0.
 *
 * Returns false and leaves *out unchanged if fewer than 3 fresh anchors are
 * visible, or if the anchor geometry is (near-)collinear.
 */
static bool compute_position(float *out_x_cm, float *out_y_cm)
{
    int64_t now_us = esp_timer_get_time();

    /* Collect indices of valid, non-stale entries */
    int valid_idx[ANCHOR_TABLE_SIZE];
    int n = 0;
    for (int i = 0; i < ANCHOR_TABLE_SIZE; i++) {
        if (g_anchors[i].valid &&
            (now_us - g_anchors[i].last_seen_us) < ANCHOR_STALE_US) {
            valid_idx[n++] = i;
        }
    }

    /*
     * 2D trilateration requires ≥ 3 anchors — this is a geometric constraint,
     * not a configuration parameter.  With n anchors we get n-1 linear equations
     * in 2 unknowns; n=3 is the minimum for a determined system.
     */
    if (n < 3) {
        return false;
    }

    /* Reference anchor: last in the collected list */
    int   ref  = valid_idx[n - 1];
    float xr   = (float)g_anchors[ref].x_cm;
    float yr   = (float)g_anchors[ref].y_cm;
    float zr   = (float)g_anchors[ref].z_cm;
    float dr   = g_anchors[ref].dist_m * 100.0f;  /* 3D range in cm */
    float rr2  = dr * dr - zr * zr;               /* 2D range² in cm² */
    if (rr2 < 0.0f) rr2 = 0.0f;

    /*
     * Build A (rows = n−1, cols = 2) and b (rows = n−1).
     *
     * For each non-reference anchor i:
     *   A[row][0] = 2·(x_i − x_r)
     *   A[row][1] = 2·(y_i − y_r)
     *   b[row]    = r_r² − r_i² + x_i² − x_r² + y_i² − y_r²
     */
    float A[ANCHOR_TABLE_SIZE - 1][2];
    float b[ANCHOR_TABLE_SIZE - 1];

    for (int row = 0; row < n - 1; row++) {
        int   k   = valid_idx[row];
        float xi  = (float)g_anchors[k].x_cm;
        float yi  = (float)g_anchors[k].y_cm;
        float zi  = (float)g_anchors[k].z_cm;
        float di  = g_anchors[k].dist_m * 100.0f;
        float ri2 = di * di - zi * zi;
        if (ri2 < 0.0f) ri2 = 0.0f;

        A[row][0] = 2.0f * (xi - xr);
        A[row][1] = 2.0f * (yi - yr);
        b[row]    = rr2 - ri2 + xi * xi - xr * xr + yi * yi - yr * yr;
    }

    int m = n - 1;  /* number of equations */

    /* Accumulate A^T A (symmetric 2×2) and A^T b (2×1) */
    float AtA00 = 0.0f, AtA01 = 0.0f, AtA11 = 0.0f;
    float Atb0  = 0.0f, Atb1  = 0.0f;
    for (int row = 0; row < m; row++) {
        AtA00 += A[row][0] * A[row][0];
        AtA01 += A[row][0] * A[row][1];
        AtA11 += A[row][1] * A[row][1];
        Atb0  += A[row][0] * b[row];
        Atb1  += A[row][1] * b[row];
    }

    /* Invert 2×2: det = a·d − b² (symmetric, so off-diagonals are equal) */
    float det = AtA00 * AtA11 - AtA01 * AtA01;
    if (fabsf(det) < 1.0f) {
        /* Near-singular: anchors are collinear or too close together */
        ESP_LOGW(TAG, "Position: collinear anchors (det=%.1f) — skipping", det);
        return false;
    }

    float inv = 1.0f / det;
    *out_x_cm = ( AtA11 * Atb0 - AtA01 * Atb1) * inv;
    *out_y_cm = (-AtA01 * Atb0 + AtA00 * Atb1) * inv;
    return true;
}

/**
 * send_position_uart() — transmit a TYPE 0x02 position frame to the STM32.
 *
 * Frame layout:
 *   [0xAA][0x02][0x04][x_lo][x_hi][y_lo][y_hi][CHECKSUM]
 *   CHECKSUM = TYPE XOR 0x04 XOR x_lo XOR x_hi XOR y_lo XOR y_hi
 */
static void send_position_uart(float x_cm, float y_cm)
{
    int16_t xi = (int16_t)roundf(x_cm);
    int16_t yi = (int16_t)roundf(y_cm);

    uint8_t xl = (uint8_t)((uint16_t)xi & 0xFFu);
    uint8_t xh = (uint8_t)((uint16_t)xi >> 8u);
    uint8_t yl = (uint8_t)((uint16_t)yi & 0xFFu);
    uint8_t yh = (uint8_t)((uint16_t)yi >> 8u);
    uint8_t len = 4u;

    uint8_t frame[8] = {
        FRAME_START, FRAME_TYPE_POS, len,
        xl, xh, yl, yh,
        (uint8_t)(FRAME_TYPE_POS ^ len ^ xl ^ xh ^ yl ^ yh),
    };
    uart_write_bytes(UART_STM32, frame, sizeof(frame));
    ESP_LOGI(TAG, "Position → STM32: x=%d cm  y=%d cm", (int)xi, (int)yi);
}

/* =========================================================================
 * UART RX task (receives temperature frames from STM32)
 * ========================================================================= */

/**
 * process_uart_frame() — called when a complete, valid frame has been
 * received and its checksum verified.
 *
 * Currently handles TYPE 0x01 (temperature).  Future frame types can be
 * added as additional case entries.
 */
static void process_uart_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    switch (type) {

    case FRAME_TYPE_TEMP: {
        /*
         * Temperature frame: 2-byte payload = int16_t tenths-of-°F LE.
         * memcpy avoids potential unaligned-access UB when reading a
         * 16-bit value from a uint8_t array.
         */
        if (len < 2) {
            ESP_LOGW(TAG, "Temp frame: payload too short (%u)", len);
            return;
        }
        int16_t temp;
        memcpy(&temp, payload, sizeof(int16_t));

        g_last_temp_tenths_f = temp;
        g_temp_updated       = true;

        int whole = temp / 10;
        int frac  = (temp < 0 ? -temp : temp) % 10;
        ESP_LOGI(TAG, "Temp ← STM32: %d.%d °F", whole, frac);
        break;
    }

    default:
        ESP_LOGW(TAG, "Unknown UART frame type 0x%02x", type);
        break;
    }
}

/**
 * uart_rx_task() — FreeRTOS task that continuously reads from UART1 and
 * reassembles frames using a simple state machine.
 *
 * State machine:
 *   IDLE      → wait for 0xAA start byte
 *   RX_TYPE   → read TYPE byte; begin checksum accumulation
 *   RX_LEN    → read LEN byte; XOR into checksum
 *   RX_PAYLOAD→ accumulate LEN bytes; XOR each into checksum
 *   RX_CHECKSUM → read CHECKSUM; compare with computed checksum
 *
 * uart_read_bytes() with a short timeout (10 ms) lets the task yield the
 * CPU when no data is available, rather than busy-spinning.
 */
static void uart_rx_task(void *pvParameters)
{
    uart_rx_state_t state = RX_IDLE;
    uint8_t type          = 0;
    uint8_t len           = 0;
    uint8_t payload[64];  /* max payload length this implementation handles */
    uint8_t payload_idx   = 0;
    uint8_t cksum_calc    = 0;

    uint8_t byte;

    while (1) {
        /*
         * Read one byte at a time.  The 10-ms timeout allows the FreeRTOS
         * scheduler to run other tasks if no data is available.
         */
        int n = uart_read_bytes(UART_STM32, &byte, 1, pdMS_TO_TICKS(10));
        if (n <= 0) {
            continue;  /* timeout — no data, loop */
        }

        switch (state) {

        case RX_IDLE:
            if (byte == FRAME_START) {
                state = RX_TYPE;
            }
            /* Any other byte in IDLE is ignored */
            break;

        case RX_TYPE:
            type      = byte;
            cksum_calc = byte;        /* checksum starts with TYPE */
            state     = RX_LEN;
            break;

        case RX_LEN:
            len        = byte;
            cksum_calc ^= byte;       /* XOR in LEN */
            payload_idx = 0;
            state = (len == 0) ? RX_CHECKSUM : RX_PAYLOAD;
            break;

        case RX_PAYLOAD:
            if (payload_idx < sizeof(payload)) {
                payload[payload_idx] = byte;
            }
            payload_idx++;
            cksum_calc ^= byte;       /* XOR each payload byte */
            if (payload_idx >= len) {
                state = RX_CHECKSUM;
            }
            break;

        case RX_CHECKSUM:
            if (byte == cksum_calc) {
                process_uart_frame(type, payload,
                                   (len < sizeof(payload)) ? len : (uint8_t)sizeof(payload));
            } else {
                ESP_LOGW(TAG, "UART frame checksum error: got 0x%02x, expected 0x%02x",
                         byte, cksum_calc);
            }
            state = RX_IDLE;
            break;
        }
    }
}

/* =========================================================================
 * Zigbee action callback — handles incoming WSN beacon commands
 * ========================================================================= */

/**
 * handle_beacon() — processes a WSN_CMD_BEACON (Cmd 0x00) received from a
 * routing anchor.
 *
 * Payload encoding (ZCL OCTET_STRING as sent by routing_anchor):
 *   data.value[0]   = length byte (= sizeof(wsn_beacon_payload_t) = 6)
 *   data.value[1–6] = packed wsn_beacon_payload_t
 *
 * RSSI: read from the NWK neighbor table entry for `src` immediately after
 *   the ZCL callback fires.  The stack stores the hardware RSSI (int8_t dBm)
 *   in esp_zb_nwk_neighbor_info_t.rssi on every received frame.
 */
static void handle_beacon(const esp_zb_zcl_custom_cluster_command_message_t *msg)
{
    uint16_t src = msg->info.src_address.u.short_addr;

    ESP_LOGI(TAG, "Beacon RX from 0x%04hx  size=%u", src, msg->data.size);

    /* Expect: 1 length byte + sizeof(wsn_beacon_payload_t) payload bytes */
    if (msg->data.size < (1 + sizeof(wsn_beacon_payload_t)) ||
        msg->data.value == NULL) {
        ESP_LOGW(TAG, "Beacon from 0x%04hx: payload too short "
                 "(got %u, need %u) — reflash that anchor",
                 src, msg->data.size,
                 (unsigned)(1 + sizeof(wsn_beacon_payload_t)));
        return;
    }

    /*
     * ZCL OCTET_STRING layout: data.value[0] = length, data.value[1..N] = data.
     * Verify the embedded length byte matches what we expect.
     */
    const uint8_t *raw = (const uint8_t *)msg->data.value;
    if (raw[0] < (uint8_t)sizeof(wsn_beacon_payload_t)) {
        ESP_LOGW(TAG, "Beacon from 0x%04hx: octet-string length byte too small "
                 "(got %u, need %u) — reflash that anchor",
                 src, raw[0], (unsigned)sizeof(wsn_beacon_payload_t));
        return;
    }

    wsn_beacon_payload_t beacon;
    memcpy(&beacon, &raw[1], sizeof(beacon));

    /* ---- RSSI from neighbor table ---------------------------------------- */
    /*
     * Walk the NWK neighbor table to find the entry for this anchor's short
     * address.  The stack updates the entry's rssi field (hardware dBm) each
     * time a frame is successfully received from that neighbor, so querying
     * immediately after the ZCL callback fires gives us the RSSI of the
     * beacon frame that just arrived.
     *
     * If the entry is not yet in the table (e.g. very first beacon before
     * routing-layer acknowledgement), fall back to the anchor's advertised
     * TX power minus a nominal 80 dB path loss, which keeps trilateration
     * working rather than silently emitting a bogus constant.
     */
    int8_t rssi_dbm = beacon.tx_power_dbm - 80;  /* fallback until table entry exists */
    {
        esp_zb_nwk_info_iterator_t iter = ESP_ZB_NWK_INFO_ITERATOR_INIT;
        esp_zb_nwk_neighbor_info_t nbr;
        while (esp_zb_nwk_get_next_neighbor(&iter, &nbr) == ESP_OK) {
            if (nbr.short_addr == src) {
                rssi_dbm = nbr.rssi;
                break;
            }
        }
    }

    /* ---- Update anchor table --------------------------------------------- */
    /*
     * Pass 1: look for an exact anchor_id match, or the first empty slot.
     * Keying on anchor_id (set at flash time) means a re-joining anchor that
     * gets a new Zigbee short address still updates its existing slot correctly.
     */
    int slot = -1;
    for (int i = 0; i < ANCHOR_TABLE_SIZE; i++) {
        if (g_anchors[i].valid && g_anchors[i].anchor_id == beacon.anchor_id) {
            slot = i;   /* exact match — update in place */
            break;
        }
        if (!g_anchors[i].valid && slot < 0) {
            slot = i;   /* first empty slot */
        }
    }

    /*
     * Pass 2: table is full with no match — evict the least-recently-seen
     * anchor.  Unreachable in normal operation with ANCHOR_TABLE_SIZE=16.
     */
    if (slot < 0) {
        slot = 0;
        for (int i = 1; i < ANCHOR_TABLE_SIZE; i++) {
            if (g_anchors[i].last_seen_us < g_anchors[slot].last_seen_us) {
                slot = i;
            }
        }
        ESP_LOGW(TAG, "Anchor table full; evicting ID=0x%02x to make room for ID=0x%02x",
                 g_anchors[slot].anchor_id, beacon.anchor_id);
    }

    float pl_meas = (float)beacon.tx_power_dbm - (float)rssi_dbm;
    float dist_m  = powf(10.0f, (pl_meas - PATHLOSS_D0_DB) / (10.0f * PATHLOSS_EXP));

    g_anchors[slot].valid        = true;
    g_anchors[slot].anchor_id    = beacon.anchor_id;
    g_anchors[slot].x_cm         = beacon.x_cm;
    g_anchors[slot].y_cm         = beacon.y_cm;
    g_anchors[slot].z_cm         = beacon.z_cm;
    g_anchors[slot].tx_power_dbm = beacon.tx_power_dbm;
    g_anchors[slot].rssi_dbm     = rssi_dbm;
    g_anchors[slot].dist_m       = dist_m;
    g_anchors[slot].last_seen_us = esp_timer_get_time();

    ESP_LOGI(TAG, "Anchor ID=0x%02x (from 0x%04hx): pos=(%d,%d,%d) cm  "
             "TxPwr=%d dBm  RSSI=%d dBm  PL=%.1f dB  dist~%d cm",
             beacon.anchor_id, src, beacon.x_cm, beacon.y_cm, beacon.z_cm,
             beacon.tx_power_dbm, rssi_dbm, pl_meas, (int)(dist_m * 100.0f + 0.5f));
}

/**
 * zb_action_handler() — global ZCL action callback.
 *
 * Installed via esp_zb_core_action_handler_register().  Runs inside
 * zigbee_task / the ZBOSS scheduler context.  Must not block.
 */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                   const void *message)
{
    if (callback_id == ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID) {
        const esp_zb_zcl_custom_cluster_command_message_t *msg =
            (const esp_zb_zcl_custom_cluster_command_message_t *)message;

        /* Only handle WSN cluster commands */
        if (msg->info.cluster != WSN_CLUSTER_ID) {
            return ESP_OK;
        }

        switch (msg->info.command.id) {
        case WSN_CMD_BEACON:
            handle_beacon(msg);
            break;
        default:
            ESP_LOGD(TAG, "Unhandled WSN cmd 0x%02x", msg->info.command.id);
            break;
        }
    }
    return ESP_OK;
}

/* =========================================================================
 * ZBOSS scheduler periodic callbacks (run inside zigbee_task)
 * ========================================================================= */

/**
 * send_temp_cb() — check for a new temperature reading and, if one arrived
 * since the last call, transmit it to the coordinator via Zigbee.
 *
 * Called from the ZBOSS scheduler alarm — runs inside zigbee_task, so it is
 * safe to call Zigbee API directly.
 *
 * Temperature is encoded as ZCL S16 (signed 16-bit, little-endian).
 * The coordinator decodes it with a simple memcpy into int16_t.
 *
 * s_last_sent_temp is static so its address remains valid after this function
 * returns, in case esp_zb_zcl_custom_cluster_cmd_req() holds the pointer
 * rather than copying the value immediately.
 */
static void send_temp_cb(uint8_t param)
{
    (void)param;

    if (g_network_joined && g_temp_updated) {
        g_temp_updated = false;

        static int16_t s_last_sent_temp;
        s_last_sent_temp = g_last_temp_tenths_f;

        esp_zb_zcl_custom_cluster_cmd_req_t cmd = {
            .zcl_basic_cmd = {
                .dst_addr_u.addr_short = COORD_SHORT_ADDR,
                .dst_endpoint          = COORD_ENDPOINT,
                .src_endpoint          = MY_ENDPOINT_ID,
            },
            .address_mode  = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            .profile_id    = ESP_ZB_AF_HA_PROFILE_ID,
            .cluster_id    = WSN_CLUSTER_ID,
            .custom_cmd_id = WSN_CMD_TEMPERATURE,
            .direction     = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
            .data = {
                /*
                 * S16: 2-byte signed integer, little-endian.
                 * The stack reads sizeof(int16_t) bytes from the pointer.
                 */
                .type  = ESP_ZB_ZCL_ATTR_TYPE_S16,
                .value = &s_last_sent_temp,
            },
        };
        esp_zb_zcl_custom_cluster_cmd_req(&cmd);

        int whole = s_last_sent_temp / 10;
        int frac  = (s_last_sent_temp < 0 ? -s_last_sent_temp
                                          : s_last_sent_temp) % 10;
        ESP_LOGI(TAG, "Temp → coordinator: %d.%d °F", whole, frac);
    }

    /* Re-schedule for the next interval */
    esp_zb_scheduler_alarm(send_temp_cb, 0, TEMP_SEND_INTERVAL_MS);
}

/**
 * position_cb() — compute a 2D position fix from current anchor table and
 * send it to the STM32 over UART, then re-schedule.
 *
 * Skips transmission silently if fewer than 3 fresh anchors are available
 * or if the geometry is collinear.
 *
 * Runs inside zigbee_task.  uart_write_bytes() is thread-safe in ESP-IDF.
 */
static void position_cb(uint8_t param)
{
    (void)param;
    float x_cm, y_cm;
    if (compute_position(&x_cm, &y_cm)) {
        int16_t xi = (int16_t)roundf(x_cm);
        int16_t yi = (int16_t)roundf(y_cm);

        static int16_t s_last_x = INT16_MIN;
        static int16_t s_last_y = INT16_MIN;

        if (xi != s_last_x || yi != s_last_y) {
            s_last_x = xi;
            s_last_y = yi;
            send_position_uart(x_cm, y_cm);
        }
    } else {
        ESP_LOGD(TAG, "Position: waiting for ≥3 fresh anchors");
    }
    esp_zb_scheduler_alarm(position_cb, 0, POS_REPORT_INTERVAL_MS);
}

/* =========================================================================
 * Zigbee signal handler
 * ========================================================================= */

/**
 * esp_zb_app_signal_handler() — mandatory Zigbee stack event callback.
 *
 * Key signals for a router:
 *   SKIP_STARTUP        stack ready; begin network steering
 *   DEVICE_FIRST_START  first boot; begin steering
 *   DEVICE_REBOOT       subsequent boot; re-join or continue
 *   STEERING            join attempt result
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t  err   = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig = *p_sg_p;

    switch (sig) {

    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Stack ready — searching for network");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "First-start failed (%s) — retrying steering",
                     esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "First start — searching for network");
        }
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Reboot failed (%s) — searching for new network",
                     esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Reboot — rejoining network");
        }
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err == ESP_OK) {
            g_network_joined = true;

            ESP_LOGI(TAG, "Joined network!  PAN: 0x%04hx  CH: %u  Short: 0x%04hx",
                     esp_zb_get_pan_id(),
                     (unsigned)esp_zb_get_current_channel(),
                     esp_zb_get_short_address());

            /*
             * Start both periodic ZBOSS scheduler alarms.
             * These will re-schedule themselves indefinitely.
             *
             * Initial delay = one interval so the first report waits for at
             * least one beacon to arrive before transmitting to the STM32.
             */
            esp_zb_scheduler_alarm(position_cb, 0, POS_REPORT_INTERVAL_MS);
            esp_zb_scheduler_alarm(send_temp_cb,   0, TEMP_SEND_INTERVAL_MS);
            ESP_LOGI(TAG, "Periodic tasks started");
        } else {
            ESP_LOGW(TAG, "Steering failed (%s) — retrying", esp_err_to_name(err));
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;

    default:
        ESP_LOGD(TAG, "Signal %d  status: %s", (int)sig, esp_err_to_name(err));
        break;
    }
}

/* =========================================================================
 * Zigbee task
 * ========================================================================= */

/**
 * zigbee_task() — all Zigbee initialisation and the main stack loop.
 *
 * Note: the WSN cluster is registered as SERVER so incoming beacon commands
 * (from routing anchors) are routed to the action callback.  The robot ESP
 * also sends temperature commands outbound — outbound commands can be
 * initiated from a server-role endpoint in esp-zigbee-lib.
 */
static void zigbee_task(void *pvParameters)
{
    /* ---- Platform configuration ------------------------------------------ */
    esp_zb_platform_config_t platform_cfg = {
        .radio_config = { .radio_mode = ZB_RADIO_MODE_NATIVE },
        .host_config  = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_cfg));

    /* ---- BDB / role configuration ---------------------------------------- */
    /*
     * ROUTER role: participates in routing and can host child end-devices.
     * Using router (instead of end device) means the radio is always on, which
     * is required to receive anchor beacons at any time.
     */
    esp_zb_cfg_t zb_cfg = {
        .esp_zb_role         = ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = INSTALLCODE_POLICY,
        .nwk_cfg.zczr_cfg    = { .max_children = 10 },
    };
    esp_zb_init(&zb_cfg);

    /* ---- Cluster / endpoint descriptors ---------------------------------- */
    static uint8_t s_dummy_attr = 0;

    esp_zb_attribute_list_t *wsn_cluster = esp_zb_zcl_attr_list_create(WSN_CLUSTER_ID);
    ESP_ERROR_CHECK(esp_zb_custom_cluster_add_custom_attr(wsn_cluster,
            0x0000, ESP_ZB_ZCL_ATTR_TYPE_U8,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &s_dummy_attr));
    esp_zb_attribute_list_t *basic_attrs = esp_zb_basic_cluster_create(NULL);

    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(
            cluster_list, basic_attrs, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster(
            cluster_list, wsn_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint           = MY_ENDPOINT_ID,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = 0x0000,
        .app_device_version = 0,
    };
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_cfg));
    esp_zb_device_register(ep_list);

    /* Register ZCL action callback to receive anchor beacons */
    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    /*
     * true = erase stored Zigbee NVS credentials on every boot.
     * This ensures the node always scans for and joins the current coordinator
     * rather than silently re-joining a stale PAN from a previous session.
     * The cost is a slightly longer join time on each power-cycle (~1-2 s).
     */
    ESP_ERROR_CHECK(esp_zb_start(true));
    esp_zb_stack_main_loop();
}

/* =========================================================================
 * Entry point
 * ========================================================================= */

void app_main(void)
{
    /* ---- NVS ------------------------------------------------------------- */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ---- UART (initialise before tasks so it is ready immediately) ------- */
    uart_init();

    ESP_LOGI(TAG, "Robot ESP32-C6 bridge node starting");

    /* ---- FreeRTOS tasks -------------------------------------------------- */
    /*
     * zigbee_task: drives the Zigbee stack.  Must be created before starting
     * the scheduler to ensure the stack is ready before app code runs.
     * Stack size 4096 is consistent with Espressif Zigbee examples.
     *
     * uart_rx_task: blocks on uart_read_bytes() waiting for STM32 frames.
     * Stack size 2048 is sufficient for the state-machine local variables.
     * Lower priority than zigbee_task to ensure Zigbee events are processed
     * promptly; UART data buffering in the driver ring buffer handles
     * any brief scheduling delays.
     */
    xTaskCreate(zigbee_task,  "zigbee_task",  4096, NULL, 5, NULL);
    xTaskCreate(uart_rx_task, "uart_rx_task", 2048, NULL, 4, NULL);
}
