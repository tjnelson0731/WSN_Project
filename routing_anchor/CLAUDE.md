# CLAUDE.md — Routing Anchor Firmware (Zigbee Router + Position Beacon)

## Project Overview

ESP32-C6 firmware for a **fixed-position routing anchor**.  Each anchor is
bolted to a known location in the robot's operating environment.  It performs
two roles simultaneously:

1. **Zigbee Router** — joins the coordinator's PAN and relays packets for
   other devices, extending the mesh network's range.
2. **Position Beacon** — periodically broadcasts its known (x, y) position
   and TX power so the robot ESP can measure RSSI and triangulate its location.

**Toolchain:** ESP-IDF ≥5.1, FreeRTOS, VSCode ESP-IDF extension.
**Target chip:** ESP32-C6 DevKit-C1 (native IEEE 802.15.4 radio, `ZB_RADIO_MODE_NATIVE`).
**Zigbee role:** `ESP_ZB_DEVICE_TYPE_ROUTER`

---

## Beacon Specification

The beacon is sent as **WSN Cmd 0x00** on custom cluster **0xFC00** to the
broadcast address `0xFFFF` (all nodes in the PAN).

**Encoding:** ZCL OCTET_STRING — `data.value[0]` = length byte (6),
`data.value[1–6]` = packed `wsn_beacon_payload_t`.

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | anchor_id | uint8 | Unique identifier for this physical unit |
| 1–2 | x_cm | int16 LE | Known X position in centimetres |
| 3–4 | y_cm | int16 LE | Known Y position in centimetres |
| 5 | tx_power_dbm | int8 | Radio TX power in dBm |

**Interval:** `BEACON_INTERVAL_MS` (default 5000 ms = 5 s).

---

## Per-Unit Customisation

Before flashing each physical anchor unit, edit these four `#define` values
at the top of `main/main.c`:

```c
#define ANCHOR_ID          0x01   /* unique per unit  */
#define ANCHOR_X_CM        0      /* cm from origin   */
#define ANCHOR_Y_CM        0      /* cm from origin   */
#define ANCHOR_TX_POWER    0      /* dBm, signed      */
```

Suggested coordinate system: origin at one corner of the room, X along one
wall, Y along the adjacent wall, all values in centimetres.

---

## Architecture & Key Files

```
routing_anchor/
├── main/
│   ├── main.c              Router init, beacon broadcaster, signal handler
│   ├── CMakeLists.txt      Requires: zigbee-lib, zboss-lib, nvs_flash
│   └── idf_component.yml   esp-zigbee-lib ~1.6.0, esp-zboss-lib ~1.6.0
├── sdkconfig.defaults      CONFIG_ZB_ZCZR_ROLE=y (router/coordinator role)
├── partitions.csv          NVS + Zigbee storage partitions
└── CMakeLists.txt          Top-level project file
```

---

## ESP-IDF / Zigbee Conventions

### Periodic Beacon — ZBOSS Scheduler
The beacon timer uses `esp_zb_scheduler_alarm()` rather than a FreeRTOS timer.
This keeps all Zigbee API calls within the `zigbee_task` context.

```c
// Schedule once from signal handler on join success:
esp_zb_scheduler_alarm(send_beacon, 0, BEACON_INTERVAL_MS);

// send_beacon() re-schedules itself at the end:
static void send_beacon(uint8_t param) {
    // ... build and send beacon ...
    esp_zb_scheduler_alarm(send_beacon, 0, BEACON_INTERVAL_MS);
}
```

`BEACON_INTERVAL_MS` must be ≤ 65535 (uint16_t limit of the scheduler alarm).

### Signal Handler
`esp_zb_app_signal_handler` handles join events.  On successful steering,
it fires the first scheduler alarm.  On failure, it retries steering.

### No Inbound Data Handling
This node only sends beacons; it does not process incoming WSN commands.
No action callback is registered (`esp_zb_core_action_handler_register` is
intentionally omitted).

### Error Handling
Always use `ESP_ERROR_CHECK()` for ESP-IDF API calls.
Log with `ESP_LOGE(TAG, ...)` for errors.

### Logging
Use `ESP_LOGI`, `ESP_LOGW`, `ESP_LOGE` with `TAG`.  Never use `printf`.

---

## Build & Flash

```bash
idf.py build
idf.py -p <PORT> flash monitor
```

Or use the VSCode ESP-IDF extension (Build / Flash / Monitor buttons).

---

## Deploying Multiple Anchors

Each physical anchor unit is a separate flash of the same binary with
different `#define` values.  Typical three-anchor layout:

| Unit | ANCHOR_ID | ANCHOR_X_CM | ANCHOR_Y_CM | Notes |
|------|-----------|-------------|-------------|-------|
| A | 0x01 | 0 | 0 | Origin corner |
| B | 0x02 | 400 | 0 | 4 m along one wall |
| C | 0x03 | 200 | 300 | 3 m from A on adjacent axis |

All three must be within radio range of the coordinator and the robot ESP.
More anchors improve trilateration accuracy but are not required; a minimum
of three non-collinear anchors is needed for 2-D localisation.
