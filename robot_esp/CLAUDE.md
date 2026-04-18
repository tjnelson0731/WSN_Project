# CLAUDE.md — Robot ESP32-C6 Bridge Node Firmware

## Project Overview

ESP32-C6 firmware for the **robot's wireless bridge node**. This device rides
on the robot alongside the STM32L452RE, connected via UART.  It bridges the
STM32 (bare-metal C firmware) to the Zigbee mesh network in both directions.

**Toolchain:** ESP-IDF ≥5.1, FreeRTOS, VSCode ESP-IDF extension.
**Target chip:** ESP32-C6 DevKit-C1 (native IEEE 802.15.4 radio, `ZB_RADIO_MODE_NATIVE`).
**Zigbee role:** `ESP_ZB_DEVICE_TYPE_ROUTER` — always-on radio, participates in
routing, can receive beacon broadcasts at any time.

---

## Functionality

### Upstream: STM32 → ESP → Coordinator
1. The STM32 sends a framed temperature packet over UART (USART1, 115200 8N1).
2. The `uart_rx_task` (FreeRTOS task) receives and parses the frame.
3. A ZBOSS scheduler alarm (`send_temp_cb`) checks every 2 s for new temperature
   data and forwards it to the coordinator as WSN Cmd 0x01.

### Downstream: Anchors → ESP → STM32
1. Routing anchors periodically broadcast WSN Cmd 0x00 beacon frames.
2. The ZCL action callback (`zb_action_handler`) receives the beacon and stores
   anchor ID, (x,y) position, TX power, and RSSI in a table (`g_anchors[]`).
3. A ZBOSS scheduler alarm (`rssi_report_cb`) sends the anchor table to the
   STM32 as a framed UART packet every 1 s.
4. The STM32 runs path-loss trilateration using the RSSI values.

---

## UART Frame Protocol

```
[0xAA][TYPE][LEN][PAYLOAD × LEN][CHECKSUM]
CHECKSUM = TYPE XOR LEN XOR payload[0] XOR … XOR payload[LEN-1]
```

| Direction   | TYPE | LEN | Payload |
|-------------|------|-----|---------|
| STM32 → ESP | 0x01 | 2   | `int16_t` tenths-of-°F, little-endian |
| ESP → STM32 | 0x02 | 1+7N | `uint8 count`, then N × `{id,x_cm,y_cm,tx_power,rssi}` |

Per anchor entry in TYPE 0x02 (7 bytes, packed):

| Offset | Field | Type |
|--------|-------|------|
| 0 | anchor_id | uint8 |
| 1–2 | x_cm | int16 LE |
| 3–4 | y_cm | int16 LE |
| 5 | tx_power_dbm | int8 |
| 6 | rssi_dbm | int8 |

---

## UART Wiring

| Signal | ESP32-C6 GPIO | STM32 pin | Direction |
|--------|---------------|-----------|-----------|
| TX | GPIO10 | PA10 (USART1_RX) | ESP → STM32 |
| RX | GPIO11 | PA9  (USART1_TX) | STM32 → ESP |

Baud rate: **115200 8N1**, no hardware flow control.
GPIO10 and GPIO11 are defined as `UART_STM32_TX_GPIO` / `UART_STM32_RX_GPIO`
in `main.c` and can be changed to any available GPIO on the DevKit-C1.

---

## RSSI Measurement

RSSI is read from the NWK neighbor table entry for the anchor's short address
immediately after the ZCL beacon callback fires.  The stack populates
`esp_zb_nwk_neighbor_info_t.rssi` (hardware `int8_t` dBm) each time a frame
is received from a direct neighbor.

The lookup iterates the neighbor table with `esp_zb_nwk_get_next_neighbor()`
until `nbr.short_addr == src`.  If no entry exists yet (first beacon before
the routing layer has catalogued the anchor), a fallback of
`tx_power_dbm − 80` is used to keep trilateration functional.

---

## Architecture & Key Files

```
robot_esp/
├── main/
│   ├── main.c              All firmware: UART, Zigbee, FreeRTOS tasks
│   ├── CMakeLists.txt      Requires: zigbee-lib, zboss-lib, nvs_flash, driver
│   └── idf_component.yml   esp-zigbee-lib ~1.6.0, esp-zboss-lib ~1.6.0
├── sdkconfig.defaults      CONFIG_ZB_ZCZR_ROLE=y  (router role)
├── partitions.csv          NVS + Zigbee storage partitions
└── CMakeLists.txt          Top-level project file
```

---

## ESP-IDF / Zigbee Conventions

### FreeRTOS Task Structure
| Task | Priority | Stack | Purpose |
|------|----------|-------|---------|
| `zigbee_task` | 5 | 4096 | Zigbee stack main loop; action callbacks; ZBOSS alarms |
| `uart_rx_task` | 4 | 2048 | Poll UART1; frame parser; update shared temperature state |

ZBOSS scheduler alarms (`esp_zb_scheduler_alarm`) run inside `zigbee_task`
and are the correct place to call Zigbee API.

### Shared State Between Tasks
- `g_last_temp_tenths_f` / `g_temp_updated`: volatile, written by `uart_rx_task`,
  read by `send_temp_cb` in `zigbee_task`.  Atomic for aligned int16_t on RISC-V.
- `g_anchors[]`: only accessed from `zigbee_task` (both writer and reader).

### Signal Handler
`esp_zb_app_signal_handler` handles Zigbee events.  On successful steering,
it starts the two ZBOSS scheduler alarms.  Do not block here.

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

## Relationship to robot_node/

The STM32 firmware lives in `../robot_node/`.  Key interface details:

| Detail | Value |
|--------|-------|
| UART port | USART1 (STM: PA9 TX, PA10 RX) |
| Baud rate | 115200 8N1 |
| STM-side driver | `robot_node/Src/uart1.c` (polling) |
| Data from STM | Temperature: `int16_t` tenths-of-°F via TYPE 0x01 frame |
| Data to STM | Anchor RSSI table via TYPE 0x02 frame |
