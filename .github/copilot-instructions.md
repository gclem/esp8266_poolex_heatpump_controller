# Copilot Instructions

## Build & Flash

This project uses [PlatformIO](https://platformio.org/). The target board is **Wemos D1 Mini (ESP8266)** running the Arduino framework.

```bash
# Build
pio run

# Flash via USB
pio run -t upload

# Flash via OTA (update upload_port in platformio.ini first)
pio run -t upload --upload-port <IP_ADDRESS>

# Open serial monitor
pio device monitor
```

There is no automated test suite — the `test/` directory is a PlatformIO placeholder only.

## First-Time Setup

`include/secrets.h` is gitignored and must be created from the sample:

```bash
cp src/secrets.h.sample include/secrets.h
# then fill in WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER_ADDR
```

## Architecture

The entire firmware lives in **`src/main.cpp`** — a single Arduino-style file with `setup()` and `loop()`.

**Data flow:**
1. `SoftwareSerial` (EspSoftwareSerial lib) reads RS-485/UART from the heat pump at 9600 8N1 via a MAX485 adapter.
2. `sniffing()` is called every loop iteration. It uses `readBytes()` with a **50 ms timeout** to isolate frames (no hardware break-condition detection).
3. Recognised frames (49–51 bytes) are decoded and published to MQTT via `PubSubClient`.
4. Home Assistant consumes the MQTT topics directly.

**Debugging** is done through **RemoteDebug** (Telnet, not Serial):
```bash
telnet <ESP_IP>
```
Use `debugV()`, `debugI()`, `debugE()` macros — **not** `Serial.print()` — for all runtime logging.

## Key Conventions

### Bus topology
Three devices share the same RS485 half-duplex bus: PAC + real remote control + Wemos D1 Mini (passive sniffer). The Wemos only receives; all its MAX485 DE/RE pins must be LOW at all times.

### ⚠️ GPIO safety rule
**GPIO2 (D4) is HIGH at boot** (ESP8266 strapping pin). Never use it for MAX485 DE/RE — it would briefly enable transmit during boot and corrupt the bus. Safe pins are GPIO14/D5, GPIO5/D1, GPIO4/D2.

### Pin assignments
| Constant   | GPIO | Wemos Pin | Note |
|------------|------|-----------|------|
| `UART_RTS` | 14   | D5        | MAX485 DE/RE — LOW at boot ✓ |
| `UART_TX`  | 5    | D1        | MAX485 DI — decoupled from hardware Serial ✓ |
| `UART_RX`  | 4    | D2        | MAX485 RO |

### Bus protocol (Jetline 90/95 — confirmed from captures)

Cycle period: **~190 ms** (2 heartbeat rounds/second). Inter-frame separator: `0x01` bytes (idle bus).

| Direction | Header | Size | Description |
|-----------|--------|------|-------------|
| remote → PAC | `0x19 0x04` | 12 B | Heartbeat (constant, every ~98 ms) |
| PAC → remote | `0xFD 0xF5` | 13 B | ACK |
| PAC → remote | `0xFF [flags]` | 11–15 B | Short config frame (setpoint? — TBD) |
| PAC → remote | `0xFF [flags]` | 29–36 B | Sensor values frame |

**`0xFF` prefix structure:** `FF [0x00|0x80] [optional: 0x06|0x08|0xFC] <data...>`

### Sensor frame decoding formula
Applied to the data bytes immediately following the `FF [flags]` prefix:

```cpp
water_in_temp    = ((0xFF - data[0]) >> 3) & 0b111111;
air_ambient_temp = ((0xFF - data[1]) >> 1) & 0b111111;
coil_temp        = ((0xFF - data[2]) >> 1) & 0b111111;
gas_temp         = ((0xFF - data[3]) >> 1) & 0b111111;
water_out_temp   = ((0xFF - data[4]) >> 1) & 0b111111;
active_status    = ((0xFF - data[6]) >> 4) & 0b000001;
```

The first data byte encodes `water_in_temp` — it varies with temperature, it is **not** a fixed header. Frame detection relies on burst length (≥ 25 B) + temperature range validation (5–45 °C).

### Frame parsing strategy
`sniffing()` splits the raw `readBytes()` buffer into **bursts** (non-`0x01` sequences) and dispatches each to `processBurst()`. This replaces the old fragile `size == 49–51` heuristic. Adding new frame types = adding a new `if` branch in `processBurst()`.

### MQTT topic namespace
All topics live under `poolheater/`:
- `poolheater/values/*` — sensor readings
- `poolheater/command/frame/timeout` — writable; uint16 ms to adjust `SoftwareSerial.setTimeout()` at runtime
- `poolheater/status` — retained `ON` when connected

### CPU frequency
`system_update_cpu_freq(160)` is called in `setup()`. Keep this for timing-sensitive changes.

### Frame log files
`src/frame_logs/*.txt` — raw UART captures for protocol reverse-engineering. Reference them when working on frame parsing. `motherboard_only.txt` = PAC-only output (no remote, 93-byte frames starting `0x0C`).
