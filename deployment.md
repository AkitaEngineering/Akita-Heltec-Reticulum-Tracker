## Deployment Steps

1.  **Hardware:**
    * Connect a LoRa antenna to the Heltec Wireless Tracker V1.1.
    * (Optional) Connect an LED to GPIO 2 for status indication.

2.  **Arduino IDE Setup:**
    * Install the Arduino IDE and ESP32 board support.
    * Install the TinyGPSPlus, Reticulum, LoRa, and Preferences libraries.
    * Select the correct Heltec ESP32 Wireless Tracker board and port in the Arduino IDE.

3.  **Code Upload:**
    * Copy and paste the provided code into the Arduino IDE.
    * Upload the code to the Heltec Wireless Tracker V1.1.

4.  **Reticulum Configuration (Crucial):**
    * Install Reticulum on the receiving device and any other devices that will be part of the LoRa network.
    * Edit the `reticulum.conf` file on all Reticulum nodes:
        * Enable LXMF: Ensure `lxmf = true` is set.
        * Configure LoRa interfaces appropriately.
        * Set up any other necessary Reticulum settings.
    * Restart the Reticulum service after making changes to `reticulum.conf`.

5.  **Configuration (Serial Monitor):**
    * Open the Arduino IDE Serial Monitor (115200 baud).
    * To change settings, use the following commands:
        * `sleep <seconds>` (e.g., `sleep 120`)
        * `loraFreq <frequency>` (e.g., `loraFreq 868000000`)
        * `dest <destination>` (e.g., `dest new_tracker`)

6.  **Receiving Data:**
    * On the Reticulum receiving device, use a Reticulum-compatible application or script to listen for packets on the configured destination address (default: `asset_tracker`).
    * Here is an example Python script to receive the data:

    ```python
    import reticulum
    import struct

    def packet_received(packet):
        if packet.destination.human == "asset_tracker":
            try:
                data = struct.unpack("<ddIQB", packet.data)
                latitude, longitude, altitude, timestamp, fix_quality = data
                print(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}, Timestamp: {timestamp}, Fix Quality: {fix_quality}")
            except struct.error:
                print("Error unpacking packet data")

    reticulum.init()
    reticulum.interfaces.add_interface(reticulum.LinkInterface())
    reticulum.destination.announce("asset_tracker")
    reticulum.packet_received_callback = packet_received

    try:
        while True:
            reticulum.run()
    except KeyboardInterrupt:
        print("Exiting")
    ```

    * Install the Python Reticulum library with `pip install reticulum`.

7.  **Deployment:**
    * Place the Heltec Wireless Tracker V1.1 in the desired location, ensuring good GPS coverage.
    * Ensure that the Reticulum network has enough nodes to provide sufficient coverage for the intended area.

This comprehensive guide and code should get your Heltec Wireless Tracker V1.1 up and running with Reticulum and LXMF.
