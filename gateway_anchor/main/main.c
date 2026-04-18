/**
 * main.c — WSN Zigbee Coordinator / Gateway
 *
 * Zigbee role: Coordinator (ZC).
 *
 * This node forms the Zigbee PAN and acts as the data sink for the wireless
 * sensor network.  It receives temperature reports from the robot's ESP32-C6
 * bridge node and prints them over the USB serial port (idf.py monitor).
 *
 * Custom WSN cluster (cluster ID 0xFC00):
 *   Cmd 0x00  Anchor Beacon       — sent by routing anchors to broadcast addr
 *                                   (not intended for us, logged and ignored)
 *   Cmd 0x01  Temperature Report  — sent by robot ESP to coordinator (us)
 *             Payload type: ESP_ZB_ZCL_ATTR_TYPE_S16
 *             Value: int16_t tenths-of-°F, little-endian
 *             Example: 734 (0x02DE) → 73.4 °F
 *
 * The WSN cluster is registered as SERVER so the stack routes incoming ZCL
 * custom cluster commands to our action callback.
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

/* ---- Configuration ------------------------------------------------------- */

#define TAG                 "GW_COORD"    /* ESP_LOG tag for this file       */
#define MY_ENDPOINT_ID      1             /* ZCL endpoint number             */
#define INSTALLCODE_POLICY  false         /* no install-code pairing         */
#define MAX_CHILDREN        20            /* coordinator neighbour table size */

/*
 * WSN custom cluster — manufacturer-specific cluster ID range 0xFC00–0xFFFF.
 * All three node types (coordinator, routing anchor, robot ESP) use the same
 * cluster ID so they can address each other's endpoints uniformly.
 */
#define WSN_CLUSTER_ID       0xFC00
#define WSN_CMD_BEACON       0x00         /* anchor beacon command ID        */
#define WSN_CMD_TEMPERATURE  0x01         /* temperature report command ID   */

/*
 * Gateway anchor identity and position.
 * Set ANCHOR_ID to a unique value across all anchor units before flashing.
 * The robot uses this ID to track anchors regardless of join order or Zigbee
 * short address, so IDs only need to be distinct — not sequential.
 */
#define ANCHOR_ID          0x00   /* uint8: unique ID for this anchor unit    */
#define ANCHOR_X_CM        300    /* int16: X position in cm                  */
#define ANCHOR_Y_CM        80    /* int16: Y position in cm                  */
#define ANCHOR_Z_CM        100    /* int16: Z position in cm (mount height)   */
#define ANCHOR_TX_POWER    0      /* int8:  TX power in dBm (signed)          */

/* Beacon broadcast period — must be ≤ 65535 (uint16_t limit of scheduler alarm) */
#define BEACON_INTERVAL_MS 5000

/* ---- Beacon payload ------------------------------------------------------- */

typedef struct {
    uint8_t anchor_id;
    int16_t x_cm;
    int16_t y_cm;
    int16_t z_cm;
    int8_t  tx_power_dbm;
} __attribute__((packed)) wsn_beacon_payload_t;

/* ---- Beacon broadcast ---------------------------------------------------- */

/**
 * send_beacon() — broadcast an Anchor Beacon (Cmd 0x00) to all Zigbee nodes.
 *
 * Mirrors the same function in routing_anchor/main/main.c.  The coordinator
 * acts as an additional fixed-position anchor so the robot can use it as one
 * of its trilateration reference points.
 *
 * Called from the ZBOSS scheduler alarm (zigbee_task context).  Re-schedules
 * itself at the end to create a periodic broadcast.
 */
static void send_beacon(uint8_t param)
{
    (void)param;

    static uint8_t s_octet_buf[1 + sizeof(wsn_beacon_payload_t)];
    s_octet_buf[0] = (uint8_t)sizeof(wsn_beacon_payload_t);

    wsn_beacon_payload_t payload = {
        .anchor_id    = (uint8_t)ANCHOR_ID,
        .x_cm         = (int16_t)ANCHOR_X_CM,
        .y_cm         = (int16_t)ANCHOR_Y_CM,
        .z_cm         = (int16_t)ANCHOR_Z_CM,
        .tx_power_dbm = (int8_t)ANCHOR_TX_POWER,
    };
    memcpy(&s_octet_buf[1], &payload, sizeof(payload));

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

    esp_zb_scheduler_alarm(send_beacon, 0, BEACON_INTERVAL_MS);
}

/* ---- Handle incoming WSN commands ---------------------------------------- */

/**
 * handle_wsn_command() — called from the ZCL action callback whenever a
 * command arrives on cluster 0xFC00.
 *
 * ESP-IDF Zigbee API notes for the message struct fields:
 *   msg->info.cluster_id          — cluster this command belongs to
 *   msg->info.command             — command ID (WSN_CMD_*)
 *   msg->info.src_address.u.short_addr
 *                                 — 16-bit network address of the sender
 *   msg->data.value               — pointer to the raw ZCL payload bytes
 *   msg->data.size                — number of bytes at data.value
 *
 * Temperature payload (ESP_ZB_ZCL_ATTR_TYPE_S16):
 *   data.size  = 2
 *   data.value = pointer to int16_t, little-endian, tenths of °F
 */
static esp_err_t handle_wsn_command(const esp_zb_zcl_custom_cluster_command_message_t *msg)
{
    if (msg->info.cluster != WSN_CLUSTER_ID) {
        return ESP_OK;  /* not our cluster, ignore */
    }

    switch (msg->info.command.id) {

    /* ---- Temperature report (robot ESP → coordinator) -------------------- */
    case WSN_CMD_TEMPERATURE: {
        if (msg->data.size < sizeof(int16_t) || msg->data.value == NULL) {
            ESP_LOGW(TAG, "Temp cmd: bad payload (size=%u)", msg->data.size);
            return ESP_ERR_INVALID_ARG;
        }

        /*
         * The ZCL S16 type is 2 bytes, little-endian.  On the ESP32-C6
         * (RISC-V, little-endian), a memcpy into int16_t gives the correct
         * value without any byte-swapping.
         */
        int16_t temp_tenths_f;
        memcpy(&temp_tenths_f, msg->data.value, sizeof(int16_t));

        /* Format as "NN.N °F" with correct sign handling */
        int whole = temp_tenths_f / 10;
        int frac  = (temp_tenths_f < 0 ? -temp_tenths_f : temp_tenths_f) % 10;
        ESP_LOGI(TAG, "Temperature from 0x%04hx: %d.%d °F  (raw %d tenths)",
                 msg->info.src_address.u.short_addr,
                 whole, frac, (int)temp_tenths_f);
        break;
    }

    /* ---- Anchor beacon (should not arrive here, but handle gracefully) --- */
    case WSN_CMD_BEACON:
        ESP_LOGD(TAG, "Anchor beacon received (unexpected) from 0x%04hx",
                 msg->info.src_address.u.short_addr);
        break;

    default:
        ESP_LOGW(TAG, "Unknown WSN cmd 0x%02x from 0x%04hx",
                 msg->info.command.id,
                 msg->info.src_address.u.short_addr);
        break;
    }

    return ESP_OK;
}

/* ---- ZCL action callback ------------------------------------------------- */

/**
 * zb_action_handler() — installed via esp_zb_core_action_handler_register().
 *
 * The Zigbee stack calls this from within esp_zb_stack_main_loop() whenever
 * a ZCL action occurs (attribute report, custom cluster command, etc.).
 *
 * Rules:
 *   - Do NOT block in this function.  If heavy processing is needed, post to
 *     a FreeRTOS queue and handle it in a separate task.
 *   - Return ESP_OK on success; a non-OK return causes the stack to log a
 *     warning but does not crash.
 *
 * @param callback_id  Identifies the ZCL event type.
 * @param message      Opaque pointer; cast according to callback_id.
 */
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                   const void *message)
{
    switch (callback_id) {

    /*
     * ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID fires when a ZCL custom
     * cluster command is received on any server-role cluster we registered.
     */
    case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID:
        return handle_wsn_command(
                (const esp_zb_zcl_custom_cluster_command_message_t *)message);

    default:
        /* Silently ignore other action types */
        break;
    }
    return ESP_OK;
}

/* ---- Zigbee signal handler ----------------------------------------------- */

/**
 * esp_zb_app_signal_handler() — mandatory Zigbee stack event callback.
 *
 * The Zigbee stack calls this for BDB (Base Device Behaviour), ZDO (Zigbee
 * Device Object), and NWK (network layer) events.  It runs in the context of
 * the zigbee_task FreeRTOS task.
 *
 * Key signals for a coordinator:
 *   SKIP_STARTUP        stack is ready; begin network formation
 *   DEVICE_FIRST_START  first boot (NVS empty); begin formation
 *   DEVICE_REBOOT       subsequent boot; re-form or re-open the network
 *   FORMATION           formation complete (check err for success/failure)
 *   STEERING            a new device joined the open network
 *   DEVICE_ANNCE        a device announced itself (short addr + IEEE EUI-64)
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t  err   = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig = *p_sg_p;

    switch (sig) {

    /* ---- Stack initialisation -------------------------------------------- */
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        /*
         * The stack finished internal init.  We must explicitly kick off
         * commissioning; the stack will not do it on its own.
         */
        ESP_LOGI(TAG, "Zigbee stack ready — forming network");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        /*
         * Fresh device (NVS empty).  Always attempt formation; if the stack
         * returns FAIL here something unexpected happened, but retrying is safe.
         */
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "First-start signal failed (%s) — retrying formation",
                     esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "First start — forming network");
        }
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        /*
         * Device rebooted with existing NVS Zigbee data.
         *
         * OK   — NVS data is valid; trigger formation so the coordinator
         *         re-creates its PAN on the saved channel / PAN-ID.
         * FAIL — NVS data is corrupt or incompatible.  Do NOT retry formation:
         *         that just re-triggers DEVICE_REBOOT in an infinite loop.
         *         Erase the Zigbee partitions and reboot to recover.
         */
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Reboot — re-forming network from NVS");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        } else {
            ESP_LOGE(TAG, "Reboot failed (%s): NVS Zigbee data is corrupt.",
                     esp_err_to_name(err));
            ESP_LOGE(TAG, "Run 'idf.py erase-flash' then reflash to recover.");
        }
        break;

    /* ---- Network formation ----------------------------------------------- */
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err == ESP_OK) {
            esp_zb_ieee_addr_t ieee;
            esp_zb_get_long_address(ieee);

            ESP_LOGI(TAG, "Network formed!  PAN: 0x%04hx  CH: %u  Short: 0x%04hx",
                     esp_zb_get_pan_id(),
                     (unsigned)esp_zb_get_current_channel(),
                     esp_zb_get_short_address());
            ESP_LOG_BUFFER_HEX(TAG, ieee, 8);

            /*
             * Open the network for joining for 180 seconds.  All nodes
             * (routing anchors, robot ESP) must be powered on during this
             * window to join.  Previously-paired nodes re-join automatically
             * on reboot without needing the window to be open.
             *
             * To keep the network permanently open, call esp_zb_bdb_open_network
             * again in the STEERING handler when err != ESP_OK (window closed).
             */
            esp_zb_bdb_open_network(180);
            ESP_LOGI(TAG, "Network open for 180 s — power on your nodes now");

            /*
             * Start the beacon broadcast timer.  The coordinator is already
             * on-network at this point, so it can broadcast immediately.
             * send_beacon() re-schedules itself after each transmission.
             */
            esp_zb_scheduler_alarm(send_beacon, 0, BEACON_INTERVAL_MS);
            ESP_LOGI(TAG, "Beacon timer started (every %u ms)", BEACON_INTERVAL_MS);
        } else {
            ESP_LOGE(TAG, "Formation failed: %s — retrying",
                     esp_err_to_name(err));
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        }
        break;

    /* ---- Device join / steering events ----------------------------------- */
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "A device joined the network");
        } else {
            /*
             * Join window closed — re-open immediately so devices can join
             * at any time without needing a coordinator reboot.
             */
            esp_zb_bdb_open_network(180);
            ESP_LOGI(TAG, "Join window re-opened for 180 s");
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        /*
         * esp_zb_app_signal_get_params() retrieves the signal-specific
         * parameter block that the stack appended to the signal.
         */
        esp_zb_zdo_signal_device_annce_params_t *dev =
            (esp_zb_zdo_signal_device_annce_params_t *)
            esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "Device announced — short: 0x%04hx", dev->device_short_addr);
        ESP_LOG_BUFFER_HEX(TAG, dev->ieee_addr, 8);
        break;
    }

    default:
        ESP_LOGD(TAG, "Unhandled signal %d  status: %s",
                 (int)sig, esp_err_to_name(err));
        break;
    }
}

/* ---- Zigbee task ---------------------------------------------------------- */

/**
 * zigbee_task() — all Zigbee work runs here.
 *
 * Initialisation order (must not be reordered):
 *   1. esp_zb_platform_config()           radio + host connection mode
 *   2. esp_zb_init()                      BDB role + network parameters
 *   3. Build cluster/endpoint descriptors
 *   4. esp_zb_device_register()           commit endpoint list to stack
 *   5. esp_zb_core_action_handler_register() install ZCL action callback
 *   6. esp_zb_set_primary_network_channel_set()
 *   7. esp_zb_start(false)                start stack; false = keep NVS data
 *   8. esp_zb_stack_main_loop()           never returns
 */
static void zigbee_task(void *pvParameters)
{
    /* ---- 1. Platform configuration --------------------------------------- */
    /*
     * ZB_RADIO_MODE_NATIVE: use the ESP32-C6's built-in IEEE 802.15.4 radio.
     * ZB_HOST_CONNECTION_MODE_NONE: no external host controller (SoC only).
     */
    esp_zb_platform_config_t platform_cfg = {
        .radio_config = { .radio_mode = ZB_RADIO_MODE_NATIVE },
        .host_config  = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_cfg));

    /* ---- 2. BDB / network configuration ---------------------------------- */
    /*
     * COORDINATOR role: forms the PAN and assigns short addresses to joiners.
     * The coordinator's own short address is always 0x0000.
     * max_children: size of the direct-child neighbour table entry allocation.
     * Increase if many routers join directly under the coordinator.
     */
    esp_zb_cfg_t zb_cfg = {
        .esp_zb_role         = ESP_ZB_DEVICE_TYPE_COORDINATOR,
        .install_code_policy = INSTALLCODE_POLICY,
        .nwk_cfg.zczr_cfg    = { .max_children = MAX_CHILDREN },
    };
    esp_zb_init(&zb_cfg);

    /* ---- 3. Endpoint / cluster descriptors ------------------------------- */
    /*
     * esp_zb_custom_cluster_create() requires at least one attribute entry.
     * We add a dummy read-only U8 attribute (ID 0x0000) as a placeholder —
     * it is never queried by any node in this network.
     *
     * The cluster is added as SERVER role so the stack delivers inbound ZCL
     * commands (our temperature reports) to the action callback.
     */
    static uint8_t s_dummy_attr = 0;

    esp_zb_attribute_list_t *wsn_cluster = esp_zb_zcl_attr_list_create(WSN_CLUSTER_ID);
    ESP_ERROR_CHECK(esp_zb_custom_cluster_add_custom_attr(wsn_cluster,
            0x0000, ESP_ZB_ZCL_ATTR_TYPE_U8,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY, &s_dummy_attr));

    /* Basic cluster (mandatory for most ZCL profiles) */
    esp_zb_attribute_list_t *basic_attrs = esp_zb_basic_cluster_create(NULL);

    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(
            cluster_list, basic_attrs, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster(
            cluster_list, wsn_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /*
     * We reuse the HA (Home Automation) profile ID (0x0104) because it is
     * broadly compatible with standard ZCL tooling.  app_device_id 0x0000 is
     * the generic HA device — fine for a custom sink node.
     */
    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint           = MY_ENDPOINT_ID,
        .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id      = 0x0000,
        .app_device_version = 0,
    };
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_cfg));

    /* ---- 4. Register endpoint list --------------------------------------- */
    esp_zb_device_register(ep_list);

    /* ---- 5. ZCL action callback ------------------------------------------ */
    /*
     * Must be called after esp_zb_device_register() and before esp_zb_start().
     * The single callback receives ALL ZCL actions; we dispatch by callback_id.
     */
    esp_zb_core_action_handler_register(zb_action_handler);

    /* ---- 6. Channel selection -------------------------------------------- */
    /*
     * ALL_CHANNELS_MASK: scan all 16 channels (11–26) during formation and
     * choose the least congested one.  The result is saved to NVS and reused
     * on subsequent boots.
     *
     * To lock to a single channel (e.g., channel 15, which sits between
     * Wi-Fi channels 1 and 6):
     *   esp_zb_set_primary_network_channel_set(1u << 15);
     */
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

    /* ---- 7–8. Start the stack -------------------------------------------- */
    /*
     * esp_zb_start(false):
     *   false = reuse NVS zigbee data (PAN ID, channel, keys) from last boot.
     *   true  = erase NVS zigbee data and start fresh (use during development
     *           to reset the PAN after changing cluster definitions).
     *
     * esp_zb_stack_main_loop() never returns; it drives the ZBOSS scheduler,
     * calls esp_zb_app_signal_handler(), and invokes the action callback.
     */
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

/* ---- Entry point --------------------------------------------------------- */

void app_main(void)
{
    /*
     * NVS must be initialised before the Zigbee stack starts because the
     * stack uses it to persist PAN ID, channel, and security keys across reboots.
     *
     * ESP_ERR_NVS_NO_FREE_PAGES / NEW_VERSION_FOUND both indicate that the
     * NVS partition needs to be erased before re-initialising.
     */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "WSN Gateway / Coordinator starting");

    /*
     * All Zigbee work must run inside a dedicated FreeRTOS task.
     * Stack size 4096 bytes is sufficient for ZBOSS internal buffers.
     * Priority 5 matches the Espressif Zigbee example convention.
     */
    xTaskCreate(zigbee_task, "zigbee_task", 4096, NULL, 5, NULL);
}
