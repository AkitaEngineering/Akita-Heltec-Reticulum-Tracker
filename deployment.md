## Deployment Steps

1.  **Hardware:**
    * Connect a LoRa antenna to the Heltec Wireless Tracker V1.1.
    * (Optional) Connect an LED to GPIO 2 for status indication.

2.  **Arduino IDE Setup:**
    * Install the Arduino IDE and ESP32 board support.
    * Install the TinyGPSPlus, LoRa, and Preferences libraries.
    * Select the correct Heltec ESP32 Wireless Tracker board and port in the Arduino IDE.

3.  **Code Upload:**
    * Copy and paste the provided code into the Arduino IDE.
    * Upload the code to the Heltec Wireless Tracker V1.1.

4.  **Configuration (Serial Monitor):**
    * Open the Arduino IDE Serial Monitor (115200 baud).
    * To change settings, use the following commands:
        * `sleep <seconds>` (e.g., `sleep 120`)
        * `loraFreq <frequency>` (e.g., `loraFreq 868000000`)
        * `dest <destination>` (e.g., `dest new_tracker`)

5.  **Receiving Data:**
    * Use the provided Python script `examples/python_receiver/receiver.py` to receive LoRa packets.
    * Install dependencies: `pip install -r examples/python_receiver/requirements.txt`
    * Run the script: `python examples/python_receiver/receiver.py`
    * The script will listen for LoRa packets and parse the AssetData structure.
