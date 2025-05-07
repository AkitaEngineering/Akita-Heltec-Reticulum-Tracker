# Akita Engineering ESP32 Asset Tracker (Heltec V1.1) 

This project, building upon and significantly enhancing the original work by Akita Engineering, provides a robust, power-efficient, and feature-rich asset tracking solution for the Heltec Wireless Tracker V1.1. It utilizes LoRa for long-range communication and the Reticulum network stack with LXMF for secure, resilient, and extended-range mesh networking. The device transmits GPS location data, incorporates advanced power management for prolonged battery life, and offers enhanced data payloads.

**License:** GPLv3

**Current Firmware Version (as per this README):** v1.1

## Key Features

* **Reticulum Network Integration:** Leverages Reticulum for secure, resilient, and potentially mesh-capable communication.
* **LXMF Enabled:** Designed for extended range using Reticulum's LXMF (Link Exchange and Message Forwarding). LXMF is a feature configured on your *entire Reticulum network* (tracker, relays, receiver).
* **Advanced Power Efficiency:**
    * Utilizes ESP32's deep sleep.
    * **Vext Control:** Manages power to external peripherals like the GPS module (user must verify board wiring).
    * **LoRa Module Sleep:** Puts the LoRa radio into low-power mode before deep sleep.
    * **Wi-Fi & Bluetooth Disable:** Explicitly turns off unused radios.
* **Enhanced GPS Data & Handling:**
    * Acquires latitude, longitude, altitude, UTC timestamp, and satellite count.
    * **HDOP (Horizontal Dilution of Precision):** Included for a better assessment of GPS fix accuracy.
    * **Fix Status:** Reports if the fix is new, if no fix was obtained, or if it's the Last Known Location (LKL).
    * **Last Known Location (LKL):** Optionally transmits the previously good location if a new fix fails.
* **Battery Voltage Monitoring:** Reads the device's battery voltage and includes it in the data payload (requires user calibration).
* **On-Device Configuration:** Settings (sleep interval, LoRa frequency, Reticulum destination, LKL behavior) are configurable via Serial Monitor and saved in Non-Volatile Storage (NVS).
* **Firmware Version Reporting:** Transmits its firmware version in the payload.
* **Watchdog Timer (WDT):** Implemented to improve reliability by recovering from potential software hangs.
* **Status LED:** Provides clear visual feedback for various operational states and errors.
* **Enhanced Python Receiver:** Example script now includes CSV data logging.

## Hardware Requirements

* Heltec Wireless Tracker V1.1 (ensure your version matches pinouts, especially for Vext and Battery ADC).
* Suitable LoRa Antenna (matched to your configured frequency).
* USB cable for programming and serial communication.
* (Optional but Recommended) Multimeter for battery voltage calibration.

## Software Requirements

* **Arduino IDE:** Latest version recommended.
* **ESP32 Board Support for Arduino IDE:** Install via Boards Manager.
* **Arduino Libraries (Install via Library Manager or manually):**
    * `TinyGPSPlus` by Mikal Hart
    * `LoRa` by Sandeep Mistry (ensure compatibility with Heltec ESP32 LoRa)
    * `Preferences` (usually included with ESP32 core)
    * **`Reticulum` for ESP32/Arduino:** This is a **critical dependency**. You must source an ESP32/Arduino-compatible port of the Reticulum stack. This might involve manual installation from a Git repository or a ZIP file. The official Reticulum (`markqvist/Reticulum`) is for Python.
* **Reticulum Python Package:** For the receiving device and any relay nodes: `pip install rns`
* **Git:** For cloning this repository.

## Setup Instructions

### 1. Clone the Repository

```bash
git clone https://github.com/AkitaEngineering/Akita-Heltec-Reticulum-Tracker
cd Akita-Heltec-Reticulum-Tracker-Enhanced # Or your chosen directory name
```
## 2. Arduino IDE Setup

### Install Arduino IDE and ESP32 Core:

- Download from [arduino.cc](https://www.arduino.cc).
- Add ESP32 package URL to **File > Preferences > Additional Boards Manager URLs**:
  ```
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
  ```
- Install "esp32" via **Tools > Board > Boards Manager...**.

### Select Board:

- Connect your Heltec Wireless Tracker.
- Go to **Tools > Board > ESP32 Arduino > Heltec Wireless Tracker** (or the closest match for your board).
- Select the correct **Port**.

### Install Libraries:

- **Tools > Manage Libraries...**: Install `TinyGPSPlus` and `LoRa` (by Sandeep Mistry).
- **Reticulum Library (Manual Install Often Required):**
  - Locate a Reticulum-compatible library for ESP32.
  - Download as ZIP or clone from GitHub.
  - Install via **Sketch > Include Library > Add .ZIP Library...** or place manually in `Documents/Arduino/libraries/`.

> The code assumes this Reticulum library is installed and API-compatible.

---

## 3. Hardware Configuration & Verification

- **Antenna**: Connect LoRa antenna securely.
- **Vext for GPS (GPIO21)**: Check if GPS is powered via `VEXT_CTRL (GPIO21)` on your board.
- **Battery ADC Pin (GPIO13)**: Ensure the firmware pin matches your board; calibrate voltage reading (see *Battery Voltage Calibration* section).

---

## 4. Code Upload & Critical Firmware Modification

- Open `src/Akita_Heltec_Reticulum_Tracker/Akita_Heltec_Reticulum_Tracker.ino` in Arduino IDE.
- **Review Pin Definitions**: Confirm settings for `LORA_SCK`, `STATUS_LED`, `VEXT_CTRL`, `BATT_ADC_PIN`, etc.

### <span style="color:red; font-weight:bold;">CRITICAL STEP: Reticulum LoRa Interface Setup</span>

- Go to `initializeReticulum()` function.
- Modify the section marked:
  ```cpp
  // !!! CRITICAL SECTION FOR RETICULUM AND LORA !!!
  ```
- Adjust this according to your Reticulum ESP32 library’s API and documentation.
- Incorrect setup will prevent Reticulum from sending/receiving over LoRa.

- Upload sketch to your Heltec device.

---

## 5. Device Configuration (via Serial Monitor)

- Open **Tools > Serial Monitor**, set baud rate to `115200`.
- Reset device and type `config` within 10 seconds.

### Available Commands:

```plaintext
sleep <seconds>         # Deep sleep interval, e.g. sleep 300
loraFreq <frequency>    # LoRa freq, e.g. loraFreq 915000000
dest <name>             # Reticulum app_name, e.g. dest my_tracker
sendLKL <0|1>           # Send Last Known Location fallback (1 = yes)
show                    # Display current settings
save                    # Save settings to NVS
reboot                  # Reboot the device
```

### Example:
```plaintext
config
loraFreq 915000000
sleep 180
dest office_tracker
sendLKL 1
save
reboot
```

---

## 6. Reticulum Network Configuration (Relays & Receiver)

### Install Reticulum:
```bash
pip install rns
```

### Configure `reticulum.conf` (typically in `~/.reticulum/config` or `/etc/reticulum/config`):

```ini
[lxmf]
lxmf = true

[interfaces]

[[My LoRa Modem]]
type = LoRaInterface
enabled = yes
port = /dev/ttyUSB0
speed = 115200
frequency = 915000000
bandwidth = 125000
spreading_factor = 7
coding_rate = 5
# output_power = 17  # Optional, set legally
```

- **Restart Reticulum**: `sudo systemctl restart reticulum` or `rnsd -d`.

---

## 7. Receiving Data

- Run `examples/python_receiver/receiver.py`.

### Example:
```bash
python receiver.py --app_name your_tracker_dest_name [--log_file path/to/your.csv]
```

- Output appears on console and logs to `asset_tracker_log.csv`.

---

## How it Works

1. **Boot & Config Check**: Initializes Watchdog Timer, waits for `config` input.
2. **Peripheral Power-Up**: Enables peripherals (GPS via VEXT).
3. **Battery Check**: Reads and stores voltage.
4. **LoRa & Reticulum Init**: Sets up LoRa, Reticulum stack, and destination identity.
5. **GPS Acquisition**:
   - Acquires GPS fix (min. 3 satellites, valid HDOP).
   - Stores Last Known Location (LKL) if fix is obtained.
6. **Data Transmission**:
   - Sends asset data via LXMF message.
7. **Sleep**:
   - Disables peripherals, enters deep sleep with radios off.

---

## AssetData Payload Structure (40 bytes)

| Field | Type | Description |
|---|---|---|
| latitude | double | Degrees |
| longitude | double | Degrees |
| altitude | float | Meters |
| gps_time | uint32_t | UTC HHMMSSCC |
| gps_date | uint32_t | UTC DDMMYY |
| satellites | uint8_t | Number used |
| hdop | float | Horizontal Dilution |
| battery_voltage | float | Volts |
| fix_status | uint8_t | 0=No Fix, 1=Fix, 2=LKL |
| firmware_version_major | uint8_t | Major ver |
| firmware_version_minor | uint8_t | Minor ver |

---

## Status LED Indications (GPIO25)

- **Solid ON**: Boot, init, transmit.
- **Slow Blink (1s)**: Serial config mode.
- **Fast Blink (0.25s)**: GPS fix in progress.
- **3 Short Blinks**: Success.
- **Fast Blink (Error)**:
  - `LORA`: Init failure.
  - `GPS`: Fix failure.
  - `RNS`: Reticulum send/init failure.
- **OFF**: Sleeping or idle.

---

## Troubleshooting

- **No Serial Output**: Confirm `115200` baud, cable, COM port, reset.
- **LoRa Init Failed**:
  - Antenna connected?
  - Correct pins/frequency?
- **Can't Enter Config Mode**:
  - Type `config` within 10s post-reset.
- **GPS Issues**:
  - Antenna & sky visibility.
  - VEXT usage validated?
  - GPS pins and baud rate correct?
- **Reticulum Errors**:
  - Check `initializeReticulum()` customization.
  - Frequency, app name, and interface must match on both ends.
  - Use `rnsstatus` and `rnpath` to debug connectivity.


## Akita Engineering

Akita Engineering specializes in custom embedded systems and IoT solutions. Contact us at akitaengineering.com.

## Contributing

Contributions are welcome. Please submit pull requests or issues.
