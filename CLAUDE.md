# CLAUDE.md — WSN Monorepo Root

## Project Overview

Wireless Sensor Network for a robotics class project. A differential-drive robot
navigates an environment, collects temperature readings, and reports its location.
The system uses a Zigbee mesh for wireless communication between three node types.

## Repository Layout

```
wsn/
├── robot_node/          STM32L452RE firmware (bare-metal C, arm-none-eabi-gcc)
├── test_coordinator/    ESP32-C6 firmware — Zigbee gateway/coordinator (ESP-IDF + FreeRTOS)
└── test_end_device/     ESP32-C6 firmware — robot's UART bridge node (ESP-IDF + FreeRTOS)
```

Each subfolder has its own `CLAUDE.md` with node-specific rules and context.

---

## System Architecture

```
[STM32L452 Robot Node]
  ├── LM34 temperature sensor (ADC)
  ├── ICM-20948 IMU — magnetometer (heading) + gyro (turn integration)
  ├── SN754410 H-bridge motor driver (diff. drive, TIM3 PWM)
  └── USART2 ─────────────────────────────────────────────────────┐
                                                                   │ 115200 8N1
[ESP32-C6 Robot Bridge Node]  ◄──────────────────────────────────┘
  ├── UART ↔ STM32 (receives temp, sends location)
  └── Zigbee ──────► mesh network
                         │
              ┌──────────┴──────────┐
    [Anchor Node A]        [Anchor Node B]   (Zigbee routers, fixed positions)
         │                      │
         └──────────┬───────────┘
                    ▼
         [ESP32-C6 Gateway]   ←── test_coordinator
           (Zigbee coordinator, collects all data)
```

### Data Flows

| Flow | Description |
|------|-------------|
| STM → ESP (UART) | Temperature reading |
| ESP → Coordinator (Zigbee) | Temperature + node ID |
| Anchor → Robot ESP (Zigbee beacon) | Anchor position (x,y) + TX power |
| Robot ESP → STM (UART) | RSSI measurements from each visible anchor |
| STM → ESP (UART) | (Optional) computed location if STM does trilateration |

### Localization

- Anchor nodes periodically broadcast their known (x,y) position and TX power.
- The robot's ESP32-C6 measures RSSI from each beacon and passes the data to the STM via UART.
- The STM uses path-loss trilateration (or sends raw RSSI back over Zigbee to the coordinator).
- The IMU magnetometer provides heading; the gyro helps detect drift or unintended turns.

---

## Zigbee Node Roles

| Node | Zigbee Role | Project Folder |
|------|-------------|----------------|
| Gateway | Coordinator (ZC) | `test_coordinator/` |
| Robot ESP | End Device (ZED) or Router | `test_end_device/` |
| Anchor nodes | Router (ZR) | TBD — likely a third project |

> **Note:** The `test_coordinator` and `test_end_device` names reflect the initial
> test scaffold. Production firmware will replace the placeholder ZCL clusters with
> custom clusters or manufacturer-specific attributes for temp/location data.

---

## Toolchain

| Subproject | Framework | Build |
|------------|-----------|-------|
| `robot_node` | Bare-metal CMSIS | `make` |
| `test_coordinator` | ESP-IDF ≥5.1, FreeRTOS | `idf.py build` / VSCode ESP-IDF extension |
| `test_end_device` | ESP-IDF ≥5.1, FreeRTOS | `idf.py build` / VSCode ESP-IDF extension |

All ESP32-C6 projects target the **ESP32-C6 DevKit** with the native IEEE 802.15.4
radio (no external radio chip). Zigbee stack: `espressif/esp-zigbee-lib ~1.6.0` +
`espressif/esp-zboss-lib ~1.6.0`.
