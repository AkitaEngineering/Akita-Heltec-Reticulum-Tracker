# Deployment Checklist

This checklist matches the current firmware and example tooling in this
repository.

## 1. Prepare hardware

- Attach a LoRa antenna before powering the radio.
- Confirm the GPS module is wired to the expected UART pins.
- Confirm the board's battery divider is connected to the configured battery ADC pin.
- For the default Heltec profile, verify these assumptions in
    `src/Akita_Heltec_Reticulum_Tracker/Akita_Heltec_Reticulum_Tracker.cpp`:
    - `GPS_RX 16`
    - `GPS_TX 17`
    - `VEXT_CTRL 21`
    - `STATUS_LED 25`
    - `BATT_ADC_GPIO 13`

## 2. Install software prerequisites

```bash
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install "TinyGPSPlus"
arduino-cli lib install "LoRa"
```

If you prefer Arduino IDE, install the same core and libraries there.

## 3. Build the firmware

Verified build command:

```bash
arduino-cli compile \
    --fqbn esp32:esp32:heltec_wireless_tracker \
    src/Akita_Heltec_Reticulum_Tracker/Akita_Heltec_Reticulum_Tracker.ino \
    --warnings all
```

If you are targeting another board, override the board selection and confirm the
pin profile before flashing.

## 4. Flash the device

- Use Arduino IDE upload or the matching `arduino-cli upload` command for your port.
- After flashing, open a serial monitor at `115200` baud.

## 5. Configure device settings

Reset the board and enter `config` within 10 seconds.

Supported commands:

```text
sleep <seconds>
loraFreq <frequency>
dest <name>
sendLKL <0|1>
show
save
reboot
```

Recommended first-pass setup:

```text
config
loraFreq 915000000
sleep 300
dest asset_tracker
sendLKL 1
save
reboot
```

## 6. Bench-verify payload handling

The example Python tool is a parser for captured payloads, not a live LoRa
receiver. Use it to verify packet format and CSV logging.

```bash
python tools/generate_test_payload.py
python examples/python_receiver/receiver.py \
    --input-file test_payload.bin \
    --log-file asset_tracker_log.csv \
    --verbose
```

If you need live radio reception, use a separate LoRa capture path to save raw
payloads, then feed those payloads into the parser.

## 7. Field validation

- Confirm LoRa packets are received at the expected frequency.
- Confirm GPS fixes are obtained in open sky.
- Compare reported battery voltage to a multimeter and adjust
    `BATT_VOLTAGE_MULTIPLIER` if necessary.
- Test the no-fix path by observing whether Last Known Location behavior matches
    your `sendLKL` setting.

## 8. Optional Reticulum path

Reticulum is not part of the default deployment path.

If you want to experiment with it:

1. Install a compatible ESP32 or Arduino Reticulum port.
2. Set `USE_RETICULUM` to `1` in the main `.cpp` implementation.
3. Adapt `initializeReticulum()` and, if needed,
     `src/reticulum_lora_adapter.h` to your chosen port.

For general scenarios this firmware is intended to cover, see
[USE_CASES.md](USE_CASES.md).
