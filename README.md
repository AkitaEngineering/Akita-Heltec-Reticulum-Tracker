# Akita Engineering Asset Tracker

Arduino-based GPS asset tracker firmware for the Heltec Wireless Tracker V1.1.
The default data path is direct LoRa transmission of a compact binary payload.
Optional Reticulum integration remains available as a compile-time template, but
it is not enabled or bundled by default.

## Current Status

- Maintained target: Arduino core on ESP32 using `esp32:esp32:heltec_wireless_tracker`.
- Default transport: direct LoRa, 40-byte little-endian `AssetData` packet.
- Optional Reticulum mode: template only, requires a compatible third-party port.
- Example Python receiver: offline packet parser for captured payloads, not a live radio driver.

See [deployment.md](deployment.md) for a deployment checklist and
[USE_CASES.md](USE_CASES.md) for scenarios this firmware fits well.

## Key Features

- GPS fix acquisition with latitude, longitude, altitude, UTC time, UTC date, satellite count, and HDOP.
- Last Known Location fallback when a new GPS fix cannot be obtained.
- Battery voltage sampling and inclusion in the transmitted payload.
- Persistent configuration in NVS for sleep interval, LoRa frequency, destination name, and LKL behavior.
- ESP32 watchdog, LoRa sleep, Wi-Fi disable, Bluetooth disable, and VEXT power control.
- Direct binary payload format shared with the example Python parser.
- Basic compile-time support for nRF52840 and RAK4631 with conservative stubs for ESP32-specific features.

## Hardware Requirements

- Heltec Wireless Tracker V1.1 or a closely compatible ESP32 LoRa tracker board.
- Suitable LoRa antenna for the selected frequency.
- GPS antenna with reasonable sky visibility.
- USB cable for flashing and serial configuration.
- Multimeter if you want to calibrate battery readings.

## Software Requirements

- Arduino IDE or `arduino-cli`.
- ESP32 Arduino core.
- Arduino libraries:
  - `TinyGPSPlus`
  - `LoRa` by Sandeep Mistry
- Python 3 if you want to use the example packet parser.

## Quick Start

### 1. Clone the repository

```bash
git clone https://github.com/AkitaEngineering/Akita-Heltec-Reticulum-Tracker
cd Akita-Heltec-Reticulum-Tracker
```

### 2. Install Arduino dependencies

Using `arduino-cli`:

```bash
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install "TinyGPSPlus"
arduino-cli lib install "LoRa"
```

Using Arduino IDE:

- Install the `esp32` platform from Boards Manager.
- Install `TinyGPSPlus` and `LoRa` from Library Manager.
- Select `Heltec Wireless Tracker` as the board when available.

### 3. Review board-specific defaults

The Arduino sketch entry point is
`src/Akita_Heltec_Reticulum_Tracker/Akita_Heltec_Reticulum_Tracker.ino`, but the
actual firmware configuration lives in
`src/Akita_Heltec_Reticulum_Tracker/Akita_Heltec_Reticulum_Tracker.cpp`.

Important Heltec defaults:

- `GPS_RX 16`
- `GPS_TX 17`
- `STATUS_LED 25`
- `VEXT_CTRL 21`
- `BATT_ADC_GPIO 13`
- `LORA_SCK 5`, `LORA_MISO 19`, `LORA_MOSI 27`, `LORA_CS 18`, `LORA_RST 14`, `LORA_IRQ 26`

If you build for a non-Heltec board, review and override those values first.

### 4. Build and flash

Verified build command:

```bash
arduino-cli compile \
  --fqbn esp32:esp32:heltec_wireless_tracker \
  src/Akita_Heltec_Reticulum_Tracker/Akita_Heltec_Reticulum_Tracker.ino \
  --warnings all
```

Flash with your normal Arduino IDE upload flow or the matching `arduino-cli`
upload command for your serial port.

### 5. Configure at runtime

Open the serial monitor at `115200` baud, reset the board, and type `config`
within 10 seconds.

Available commands:

```text
sleep <seconds>
loraFreq <frequency>
dest <name>
sendLKL <0|1>
show
save
reboot
```

Example session:

```text
config
loraFreq 915000000
sleep 180
dest office_tracker
sendLKL 1
save
reboot
```

## Runtime Flow

1. Boot and start the watchdog.
2. Allow a short serial window for configuration mode.
3. Power GPS and related peripherals through VEXT if configured.
4. Read battery voltage.
5. Initialize LoRa.
6. Attempt GPS acquisition until timeout or acceptable fix quality.
7. Serialize and send a 40-byte packet over LoRa.
8. Put the LoRa radio to sleep and enter deep sleep.

## Packet Format

The current firmware writes a canonical 40-byte little-endian payload instead
of sending the raw in-memory C struct. This avoids compiler-padding problems.

| Field | Type | Description |
| --- | --- | --- |
| `latitude` | `double` | Degrees |
| `longitude` | `double` | Degrees |
| `altitude` | `float` | Meters |
| `gps_time` | `uint32_t` | UTC `HHMMSSCC` |
| `gps_date` | `uint32_t` | UTC `DDMMYY` |
| `satellites` | `uint8_t` | Number of satellites used |
| `hdop` | `float` | Horizontal dilution of precision |
| `battery_voltage` | `float` | Estimated battery voltage |
| `fix_status` | `uint8_t` | `0 = no fix`, `1 = new fix`, `2 = last known location` |
| `firmware_version_major` | `uint8_t` | Firmware major version |
| `firmware_version_minor` | `uint8_t` | Firmware minor version |

The example Python parser also accepts the older 44-byte legacy packet layout
that came from sending the raw struct directly.

## Example Receiver and Test Workflow

The example Python tool in `examples/python_receiver/receiver.py` does not
listen to LoRa hardware by itself. It parses a raw binary payload from disk and
logs it to CSV.

The bundled `examples/python_receiver/requirements.txt` has no mandatory
dependencies for this offline parser.

Generate and parse a test payload:

```bash
python tools/generate_test_payload.py
python examples/python_receiver/receiver.py \
  --input-file test_payload.bin \
  --log-file asset_tracker_log.csv \
  --verbose
```

If you want live reception, use a separate LoRa capture tool or radio bridge to
save raw payloads, then pass those payloads to the parser.

## Optional Reticulum Integration

Reticulum support is intentionally off by default.

To experiment with it:

1. Install a compatible ESP32 or Arduino Reticulum port.
2. Set `USE_RETICULUM` to `1` in
   `src/Akita_Heltec_Reticulum_Tracker/Akita_Heltec_Reticulum_Tracker.cpp`.
3. Adapt `initializeReticulum()` in the same file to your port's API.
4. Optionally use `src/reticulum_lora_adapter.h` as a guarded template for LoRa interface registration.

Notes:

- No official Reticulum Arduino port is vendored here.
- The default firmware path remains direct LoRa even when the template code is present.
- If you need production Reticulum routing, identity management, or LXMF behavior,
  expect to do port-specific integration work.

## Platform Notes

- The maintained path is the Arduino core on ESP32. The current project does
  not require an ESP-IDF migration for normal operation.
- ESP-IDF only becomes attractive if you need lower-level power tuning, native
  driver work beyond the Arduino stack, OTA infrastructure, or a custom radio
  abstraction outside the current LoRa library.
- Non-ESP32 builds are supported at compile time, but deep sleep and power
  control are intentionally conservative stubs there.

## Status LED Behavior

- Solid on: boot, initialization, or transmission in progress.
- Slow blink: serial configuration mode.
- Fast blink: GPS acquisition.
- Short success blink phase: packet transmitted successfully.
- Fast error blink: LoRa or GPS failure.
- Off: idle or sleeping.

## Troubleshooting

- No serial output:
  - Confirm `115200` baud.
  - Check the cable, board selection, and serial port.
- LoRa initialization fails:
  - Confirm antenna is attached.
  - Confirm pins and configured frequency match your hardware.
- GPS never fixes:
  - Verify `GPS_RX` and `GPS_TX` match your board.
  - Verify VEXT control matches your GPS power wiring.
  - Test with clear sky visibility.
- Battery voltage looks wrong:
  - Confirm `BATT_ADC_GPIO` matches the real battery divider input.
  - Calibrate `BATT_VOLTAGE_MULTIPLIER` against a multimeter.
- Python receiver fails to parse a payload:
  - Confirm the file contains exactly one raw packet.
  - Confirm the payload is either 40 bytes or the older 44-byte legacy format.

## Additional Documentation

- [deployment.md](deployment.md): deployment and verification checklist.
- [USE_CASES.md](USE_CASES.md): recommended scenarios and configuration guidance.

## License

GPLv3.
