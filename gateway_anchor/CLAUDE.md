# CLAUDE.md — Zigbee Gateway / Coordinator Firmware

## Project Overview

ESP32-C6 firmware for the **Zigbee coordinator** (gateway node).  This device
forms the Zigbee PAN, accepts joins from routing anchors and the robot ESP,
and aggregates temperature data reported by the robot.

**Toolchain:** ESP-IDF ≥5.1, FreeRTOS, VSCode ESP-IDF extension.
**Target chip:** ESP32-C6 DevKit-C1 (native IEEE 802.15.4 radio, `ZB_RADIO_MODE_NATIVE`).
**Zigbee role:** `ESP_ZB_DEVICE_TYPE_COORDINATOR`

---

## Current State

The project implements the core WSN coordinator functionality:
- Forms a Zigbee network on startup and opens it for 180 s.
- Re-opens the network automatically on subsequent formation signals.
- Logs device announcements (short address + IEEE EUI-64).
- Receives WSN temperature reports (Cmd 0x01 on cluster 0xFC00) from the
  robot ESP and logs them over USB serial (`idf.py monitor`).

---

## Custom WSN Cluster (0xFC00)

All three node types use this shared cluster ID.

| Cmd | Name | Sender | Receiver | Payload |
|-----|------|--------|----------|---------|
| 0x00 | Anchor Beacon | routing_anchor | robot_esp | `{id,x,y,z,tx_power}` (8 B) |
| 0x01 | Temperature Report | robot_esp | gateway_anchor | `int16_t` tenths-of-°F |

The gateway registers the cluster as **SERVER** so incoming commands are
routed to the ZCL action callback (`zb_action_handler`).

---

## Architecture & Key Files

```
gateway_anchor/
├── main/
│   ├── main.c              Coordinator init, signal handler, action callback
│   ├── CMakeLists.txt      Requires: zigbee-lib, zboss-lib, nvs_flash
│   └── idf_component.yml   esp-zigbee-lib ~1.6.0, esp-zboss-lib ~1.6.0
├── sdkconfig.defaults      CONFIG_ZB_ZCZR_ROLE=y (coordinator/router role)
├── partitions.csv          NVS + Zigbee storage partitions
└── CMakeLists.txt          Top-level project file
```

To extend functionality, add source files under `main/` and list them in
`main/CMakeLists.txt` `SRCS`.  Suggested future files:
- `data_store.c` — buffer received readings and print a CSV summary
- `usb_output.c` — forward data over USB CDC to a host PC

---

## ESP-IDF / Zigbee Conventions

### Entry Pattern
All Zigbee work runs inside `zigbee_task` (pinned via `xTaskCreate` after
`nvs_flash_init()`).  Never call Zigbee API directly from `app_main`.

### Signal Handler
`esp_zb_app_signal_handler` is the mandatory Zigbee event callback.  Do not
block here — defer heavy work to a task via a FreeRTOS queue.

### ZCL Action Callback
`esp_zb_core_action_handler_register(zb_action_handler)` installs a single
global callback for all ZCL events.  The callback is called from the
`zigbee_task` / ZBOSS scheduler context.

### Initialisation Order
```c
esp_zb_platform_config()            // 1. radio + host mode
esp_zb_init()                       // 2. BDB role
// build cluster/endpoint descriptors
esp_zb_device_register()            // 3. commit endpoints
esp_zb_core_action_handler_register() // 4. ZCL callback
esp_zb_set_primary_network_channel_set()
esp_zb_start(false)                 // 5. start (false = keep NVS)
esp_zb_stack_main_loop()            // 6. never returns
```

### Error Handling
Always wrap ESP-IDF API calls with `ESP_ERROR_CHECK()` unless the failure is
explicitly handled.  Log errors with `ESP_LOGE(TAG, ...)`.

### Logging
Use `ESP_LOGI`, `ESP_LOGW`, `ESP_LOGE` with module `TAG`.  Never use `printf`.

---

## Build & Flash

```bash
idf.py build
idf.py -p <PORT> flash monitor
```

Or use the VSCode ESP-IDF extension (Build / Flash / Monitor buttons).

---

## Zigbee Network Parameters

| Parameter | Value |
|-----------|-------|
| Channel | Auto (all-channels scan on formation; saved to NVS) |
| PAN ID | Auto-assigned on first boot; reused from NVS on reboot |
| Max children | 20 (configurable via `MAX_CHILDREN` in `main.c`) |
| Install code | Disabled |

To fix a specific channel (e.g., channel 15 to avoid Wi-Fi):
```c
esp_zb_set_primary_network_channel_set(1u << 15);
```

To force a fresh network (erases NVS Zigbee data):
```c
esp_zb_start(true);   // true = erase and re-form
```
