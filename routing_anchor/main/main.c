/**
 * main.c — WSN Routing Anchor (Zigbee Router + Position Beacon)
 *
 * Zigbee role: Router (ZR).
 *
 * This node has two jobs:
 *   1. Act as a Zigbee router — join the coordinator's PAN and relay packets
 *      for other devices, extending the mesh network's range.
 *   2. Periodically broadcast an Anchor Beacon so the robot ESP can measure
 *      the RSSI of each beacon and compute its position via trilateration.
 *
 * Beacon specification (WSN Cmd 0x00 on cluster 0xFC00):
 *   Sent every BEACON_INTERVAL_MS to broadcast address 0xFFFF (all nodes).
 *   Payload encoding: ZCL OCTET_STRING — first byte is the data length (8),
 *   followed by 8 data bytes in the layout below.
 *
 *   Offset  Field           Type    Description
 *   ------  -----           ----    -----------
 *      0    anchor_id       uint8   Unique ID set per unit before flashing
 *      1-2  x_cm            int16   Known X position in centimetres (LE)
 *      3-4  y_cm            int16   Known Y position in centimetres (LE)
 *      5-6  z_cm            int16   Known Z position in centimetres (LE)
 *      7    tx_power_dbm    int8    Radio TX power in dBm (signed)
 *
 * Customisation:
 *   Change ANCHOR_X_CM, ANCHOR_Y_CM, ANCHOR_Z_CM, and ANCHOR_TX_POWER below
 *   before building and flashing each individual anchor unit.
 *
 * Build:  idf.py build
 * Flash:  idf.py -p <PORT> flash monitor
 * Target: ESP32-C6 DevKit-C1, ESP-IDF ≥5.1, esp-zigbee-lib ~1.6.0
 */

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_core.h"

/* ---- Per-anchor configuration -------------------------------------------- */
/*
 * Edit these values for each physical anchor unit before building.
 *
 * ANCHOR_ID must be unique across all anchor units (including the gateway).
 * The robot uses this ID to track anchors regardless of join order or Zigbee
 * short address.  IDs only need to be distinct — any uint8_t values work.
 *
 * Coordinate system: choose any origin and scale, consistent across all anchors.
 * A natural choice is cm from one corner of the room.
 *
 * Example three-anchor layout (top-down view):
 *   Anchor 0x01 at (   0,   0) cm  — origin corner
 *   Anchor 0x02 at ( 400,   0) cm  — same wall, 4 m away
 *   Anchor 0x03 at ( 200, 300) cm  — opposite wall
 *
 * ANCHOR_TX_POWER: the nominal conducted output power of the ESP32-C6 radio
 * at the setting used (default ~0 dBm in the Espressif SDK).  Used by the
 * robot in the log-distance path-loss model:
 *   distance_m ≈ 10 ^ ((tx_power_dbm - rssi_dbm - A) / (10 * n))
 * where A is the RSSI at 1 m (calibrate empirically) and n is the path-loss
 * exponent (typically 2–3 indoors).
 */
#define ANCHOR_ID          0x02   /* uint8: unique ID — change per unit       */
#define ANCHOR_X_CM        300      /* int16: X position in cm                  */
#define ANCHOR_Y_CM        0      /* int16: Y position in cm                  */
#define ANCHOR_Z_CM        90    /* int16: Z position in cm (mount height)   */
#define ANCHOR_TX_POWER    0      /* int8:  TX power in dBm (signed)          */

/* How often to send a beacon broadcast (milliseconds).
 * 5 s gives the robot time to collect multiple readings per interval.
 * Maximum value: 65535 (uint16_t limit of esp_zb_scheduler_alarm). */
#define BEACON_INTERVAL_MS 5000

/* ---- Shared constants ---------------------------------------------------- */

#define TAG                 "ANCHOR"      /* ESP_LOG tag */
#define MY_ENDPOINT_ID      1
#define INSTALLCODE_POLICY  false

/* WSN custom cluster — same IDs used on all three node types */
#define WSN_CLUSTER_ID      0xFC00
#define WSN_CMD_BEACON      0x00

/* ---- Beacon payload type ------------------------------------------------- */

/*
 * __attribute__((packed)) prevents the compiler from inserting padding bytes,
 * ensuring that memcpy produces the exact 8-byte wire layout.
 */
typedef struct {
    uint8_t anchor_id;
    int16_t x_cm;
    int16_t y_cm;
    int16_t z_cm;
    int8_t  tx_power_dbm;
} __attribute__((packed)) wsn_beacon_payload_t;

/* ---- Beacon send (ZBOSS scheduler callback) ------------------------------- */

/**
 * send_beacon() — fires on the ZBOSS scheduler alarm, inside zigbee_task.
 *
 * Builds the ZCL OCTET_STRING-encoded beacon and broadcasts it via a custom
 * cluster command.  Re-schedules itself at the end so beacons continue
 * indefinitely after the initial schedule from the STEERING signal handler.
 *
 * OCTET_STRING encoding (ZCL spec §2.6.2):
 *   Byte 0:    length of the data that follows (8 for our payload)
 *   Bytes 1–8: the wsn_beacon_payload_t struct, packed
 * The esp-zigbee-lib serialises this buffer verbatim into the ZCL frame,
 * so the receiver's data.value pointer will include the length byte.
 *
 * @param param  Unused; required by the esp_zb_callback_t signature.
 */
static void send_beacon(uint8_t param)
{
    (void)param;

    /* ---- Build the ZCL OCTET_STRING buffer ------------------------------- */
    /*
     * Static storage ensures the buffer is valid even if esp_zb_zcl_custom_cluster_cmd_req
     * takes a reference rather than copying the data internally.
     */
    static uint8_t s_octet_buf[1 + sizeof(wsn_beacon_payload_t)];

    s_octet_buf[0] = (uint8_t)sizeof(wsn_beacon_payload_t);  /* length byte */

    wsn_beacon_payload_t payload = {
        .anchor_id    = (uint8_t)ANCHOR_ID,
        .x_cm         = (int16_t)ANCHOR_X_CM,
        .y_cm         = (int16_t)ANCHOR_Y_CM,
        .z_cm         = (int16_t)ANCHOR_Z_CM,
        .tx_power_dbm = (int8_t)ANCHOR_TX_POWER,
    };
    memcpy(&s_octet_buf[1], &payload, sizeof(payload));

    /* ---- Build and dispatch the ZCL broadcast command ------------------- */
    /*
     * esp_zb_zcl_custom_cluster_cmd_req_t fields explained:
     *
     * dst_addr_u.addr_short = 0xFFFF
     *   Zigbee "all-nodes" broadcast address.  Every device in the PAN that
     *   is currently awake will receive this frame.
     *
     * dst_endpoint = 0xFF
     *   Broadcast endpoint — the ZCL frame is delivered to all matching
     *   cluster/profile combinations on every endpoint of every device.
     *
     * address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT
     *   Use 16-bit short addressing with an explicit destination endpoint.
     *   This is the standard mode for both unicast and broadcast short-addr
     *   transmissions in the esp-zigbee-lib API.
     *
     * direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV
     *   ZCL convention: the sender acts as the cluster client, the receiver
     *   as the server.  Receivers that registered ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
     *   for this cluster will see the command in their action callback.
     *
     * data.type = ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING
     *   Variable-length byte array.  The stack transmits data.value verbatim;
     *   data.value[0] must be the ZCL length byte (set above).
     */
    esp_zb_zcl_custom_cluster_cmd_req_t cmd = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = 0xFFFF,
            .dst_endpoint          = 0xFF,
            .src_endpoint          = MY_ENDPOINT_ID,
        },
        .address_mode  = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .profile_id    = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_id    = WSN_CLUSTER_ID,
        .custom_cmd_id = WSN_CMD_BEACON,
        .direction     = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .data = {
            .type  = ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
            .value = s_octet_buf,
        },
    };
    esp_zb_zcl_custom_cluster_cmd_req(&cmd);

    ESP_LOGI(TAG, "Beacon TX: ID=0x%02x  pos=(%d,%d,%d) cm  TxPwr=%d dBm",
             ANCHOR_ID, ANCHOR_X_CM, ANCHOR_Y_CM, ANCHOR_Z_CM, ANCHOR_TX_POWER);

    /* ---- Re-arm the one-shot alarm --------------------------------------- */
    /*
     * esp_zb_scheduler_alarm() schedules a one-shot callback inside the ZBOSS
     * scheduler.  Calling it from within the callback creates a periodic timer.
     *
     * The callback runs in the zigbee_task context, so it is safe to call any
     * Zigbee API (including esp_zb_zcl_custom_cluster_cmd_req) from here.
     *
     * Note: the time argument is uint16_t — maximum value is 65535 ms (~65 s).
     * BEACON_INTERVAL_MS must stay within this limit.
     */
    esp_zb_scheduler_alarm(send_beacon, 0, BEACON_INTERVAL_MS);
}

/* ---- Zigbee signal handler ----------------------------------------------- */

/**
 * esp_zb_app_signal_handler() — mandatory Zigbee stack event callback.
 *
 * Key signals for a router:
 *   SKIP_STARTUP           stack ready; start network steering (join attempt)
 *   DEVICE_FIRST_START     first boot with empty NVS; start steering
 *   DEVICE_REBOOT          subsequent boot; re-join or continue as router
 *   STEERING               join attempt result (success or retry)
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t  err   = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig = *p_sg_p;

    switch (sig) {

    /* ---- Stack initialisation -------------------------------------------- */
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Stack ready — searching for network");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        /* Fresh device (NVS empty) — always safe to start steering. */
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "First-start failed (%s) — retrying steering",
                     esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "First start — searching for network");
        }
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        /*
         * OK   — NVS has a valid prior join; attempt to re-join the same PAN.
         * FAIL — stored NVS join data is stale (different PAN, erased coordinator,
         *         etc.).  Retry steering so the node finds and joins the current
         *         coordinator rather than halting silently.
         */
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Reboot failed (%s) — searching for new network",
                     esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Reboot — rejoining network");
        }
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;

    /* ---- Network steering (join result) ---------------------------------- */
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Joined network!  PAN: 0x%04hx  CH: %u  Short: 0x%04hx",
                     esp_zb_get_pan_id(),
                     (unsigned)esp_zb_get_current_channel(),
                     esp_zb_get_short_address());

            /*
             * Network joined — schedule the first beacon after one interval.
             * send_beacon() re-schedules itself, so this single call starts
             * the infinite periodic broadcast.
             */
            esp_zb_scheduler_alarm(send_beacon, 0, BEACON_INTERVAL_MS);
            ESP_LOGI(TAG, "Beacon timer started (every %u ms)", BEACON_INTERVAL_MS);
        } else {
            /*
             * Steering failed — the coordinator may not be running or the
             * join window may be closed.  Retry immediately; the stack will
             * back-off internally if the channel is busy.
             */
            ESP_LOGW(TAG, "Steering failed (%s) — retrying",
                     esp_err_to_name(err));
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        break;

    default:
        ESP_LOGD(TAG, "Signal %d  status: %s", (int)sig, esp_err_to_name(err));
        break;
    }
}

/* ---- Zigbee task ---------------------------------------------------------- */

static void zigbee_task(void *pvParameters)
{
    /* ---- Platform configuration ------------------------------------------ */
    esp_zb_platform_config_t platform_cfg = {
        .radio_config = { .radio_mode = ZB_RADIO_MODE_NATIVE },
        .host_config  = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_cfg));

    /* ---- Zigbee BDB / role configuration --------------------------------- */
    /*
     * ROUTER role: joins an existing PAN and participates in routing.
     * Routers can also host child end-devices (max_children = 10 here).
     * We use the zczr_cfg (coordinator/router) union member — same struct
     * for both ZC and ZR roles in the ESP-IDF Zigbee API.
     */
    esp_zb_cfg_t zb_cfg = {
        .esp_zb_role         = ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = INSTALLCODE_POLICY,
        .nwk_cfg.zczr_cfg    = { .max_children = 10 },
    };
    esp_zb_init(&zb_cfg);

    /* ---- Endpoint / cluster descriptors ---------------------------------- */
    /*
     * WSN cluster as SERVER — allows us to also receive inbound commands if
     * needed in the future (e.g., a coordinator query for anchor status).
     * Outbound beacon commands can be sent from a server-role endpoint.
     */
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

    /* No action callback needed — this node only sends, not receives WSN cmds */

    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    /*
     * true = erase stored Zigbee NVS credentials on every boot so this anchor
     * always joins the current coordinator rather than a stale PAN.
     */
    ESP_ERROR_CHECK(esp_zb_start(true));
    esp_zb_stack_main_loop();
}

/* ---- Entry point --------------------------------------------------------- */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "WSN Routing Anchor starting  ID=0x%02x  pos=(%d,%d,%d) cm",
             ANCHOR_ID, ANCHOR_X_CM, ANCHOR_Y_CM, ANCHOR_Z_CM);
    xTaskCreate(zigbee_task, "zigbee_task", 4096, NULL, 5, NULL);
}
