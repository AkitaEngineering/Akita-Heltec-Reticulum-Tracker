# Akita Engineering ESP32 Asset Tracker (Heltec V1.1)

This project, developed by Akita Engineering, provides a power-efficient asset tracking solution for the Heltec Wireless Tracker V1.1, leveraging LoRa and the Reticulum network stack with LXMF for robust and extended range communication. It transmits GPS location data, utilizing deep sleep for prolonged battery life.

## Hardware Requirements

* Heltec Wireless Tracker V1.1
* LoRa Antenna

## Software Requirements

* Arduino IDE
* ESP32 Board Support for Arduino IDE
* Arduino Libraries:
    * TinyGPSPlus
    * Reticulum
    * LoRa
    * Preferences
* Reticulum (installed on receiving and relay devices)

## Setup Instructions

1.  **Arduino IDE Setup:**
    * Install the Arduino IDE and ESP32 board support.
    * Install the required libraries using the Arduino IDE Library Manager.

2.  **Hardware Configuration:**
    * Connect a LoRa antenna to the Heltec V1.1.
    * (Optional) Connect an LED to GPIO 2 for status indication.

3.  **Code Upload:**
    * Upload the provided Arduino code to the Heltec V1.1.

4.  **Reticulum Network Configuration (Critical):**
    * Install and configure Reticulum on all receiving and relay devices.
    * Enable LXMF in the `reticulum.conf` file (`lxmf = true`).
    * Configure LoRa interfaces within Reticulum.
    * Restart the Reticulum service after configuration changes.

5.  **Device Configuration (Serial Monitor):**
    * Open the Arduino IDE Serial Monitor (115200 baud).
    * Use the following commands to configure the device:
        * `sleep <seconds>`: Set deep sleep interval (e.g., `sleep 120`).
        * `loraFreq <frequency>`: Set LoRa frequency (e.g., `loraFreq 868000000`).
        * `dest <destination>`: Set Reticulum destination address (e.g., `dest new_tracker`).

6.  **Receiving Data:**
    * On a Reticulum-enabled receiving device, use a compatible application or script to listen for packets on the configured destination address.
    * A sample python script is provided within the code comments.

## Key Features

* **LXMF Enabled:** Utilizes Reticulum's LXMF for extended range and robust networking.
* **Deep Sleep:** Power-efficient deep sleep mode for extended battery life.
* **NVS Configuration:** Configuration parameters stored in non-volatile storage.
* **GPS Fix Quality:** Includes satellite count in transmitted data.
* **Status LED:** LED indicator for GPS/LoRa activity.

## LXMF Importance

This project relies heavily on LXMF (Link Exchange and Message Forwarding) for effective operation. Without LXMF, the communication range will be severely limited. Ensure LXMF is properly configured in your Reticulum network.

## Akita Engineering

Akita Engineering specializes in custom embedded systems and IoT solutions. Contact us at akitaengineering.com.

## Contributing

Contributions are welcome. Please submit pull requests or issues.
