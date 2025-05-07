# Reticulum LXMF Receiver Example - Enhanced
import RNS
import LXMF
import time
import struct
import argparse
import csv
import os
from datetime import datetime

# Define the AssetData structure format for unpacking
# Matches C++ struct:
# double latitude;    (8 bytes, 'd')
# double longitude;   (8 bytes, 'd')
# float altitude;     (4 bytes, 'f')
# uint32_t gps_time;  (4 bytes, 'I') (HHMMSSCC)
# uint32_t gps_date;  (4 bytes, 'I') (DDMMYY)
# uint8_t satellites; (1 byte,  'B')
# float hdop;         (4 bytes, 'f')
# float battery_voltage; (4 bytes, 'f')
# uint8_t fix_status; (1 byte, 'B') (0=NoFix, 1=New, 2=LKL)
# uint8_t fw_major;   (1 byte, 'B')
# uint8_t fw_minor;   (1 byte, 'B')
ASSET_DATA_FORMAT = "<ddfIIBffBBB" # Little-endian
ASSET_DATA_SIZE = struct.calcsize(ASSET_DATA_FORMAT)

DEFAULT_DESTINATION_APP_NAME = "asset_tracker" # Default if not overridden by tracker's "dest" command
# Tracker uses "location" and "asset_data" as aspects. We listen for the app_name primarily.
# More specific aspect filtering can be added if needed.

DEFAULT_CSV_LOG_FILE = "asset_tracker_log.csv"
IDENTITY_PATH = "receiver_identity" # Path to store receiver's RNS identity

class AssetReceiver:
    def __init__(self, rns_configdir=None, destination_app_name=None, log_file=None):
        self.rns_configdir = rns_configdir
        self.app_name = destination_app_name or DEFAULT_DESTINATION_APP_NAME
        self.log_file_path = log_file or DEFAULT_CSV_LOG_FILE

        # Initialize Reticulum
        self.reticulum = RNS.Reticulum(configdir=self.rns_configdir, loglevel=RNS.LOG_VERBOSE) # or RNS.LOG_INFO

        # Create or load receiver's identity
        if os.path.exists(IDENTITY_PATH) and RNS.Identity.validate_path(IDENTITY_PATH):
            try:
                self.identity = RNS.Identity.from_file(IDENTITY_PATH)
                if self.identity is None: # from_file might return None on error
                    raise FileNotFoundError("Identity file corrupted or unreadable.")
                RNS.log(f"Loaded identity from {IDENTITY_PATH}", RNS.LOG_INFO)
            except Exception as e:
                RNS.log(f"Could not load identity from {IDENTITY_PATH}: {e}. Creating new one...", RNS.LOG_ERROR)
                self.identity = RNS.Identity()
                self.identity.to_file(IDENTITY_PATH)
        else:
            RNS.log(f"No valid identity found at {IDENTITY_PATH}, creating new one...", RNS.LOG_INFO)
            self.identity = RNS.Identity()
            self.identity.to_file(IDENTITY_PATH)

        RNS.log(f"Receiver RNS Identity: {self.identity.hash_hex()}", RNS.LOG_INFO)

        # Initialize LXMF Router with the identity
        # Ensure storagepath for LXMRouter is valid (e.g., within .reticulum or a dedicated app dir)
        lxmf_storage_path = None
        if self.rns_configdir:
            lxmf_storage_path = os.path.join(self.rns_configdir, "lxmf_storage")
            if not os.path.exists(lxmf_storage_path):
                os.makedirs(lxmf_storage_path, exist_ok=True)
        elif RNS.Reticulum.configdir: # Use default if specific not provided
             lxmf_storage_path = os.path.join(RNS.Reticulum.configdir, "lxmf_storage")
             if not os.path.exists(lxmf_storage_path):
                os.makedirs(lxmf_storage_path, exist_ok=True)


        self.lxmf_router = LXMF.LXMRouter(identity=self.identity, storagepath=lxmf_storage_path)
        RNS.log("LXMF Router initialized.", RNS.LOG_INFO)

        # Create an LXMF Destination for listening
        # This destination will match announcements for the app_name.
        self.destination = LXMF.LXMFDestination(
            self.identity,
            self.lxmf_router.aspect_filter_app_name(self.app_name.encode('utf-8'))
        )
        # Tracker announces with "location" and "asset_data" aspects.
        # If you need to filter specifically for one of these, you could add:
        # self.destination.aspects.append("location".encode('utf-8'))

        self.destination.set_message_callback(self.message_received_callback)
        self.init_csv_log()

        RNS.log(f"Listening for LXMF messages for App: '{self.app_name}'", RNS.LOG_INFO)
        RNS.log(f"Receiver LXMF Destination hash: {self.destination.hash_hex()}", RNS.LOG_INFO)
        RNS.log("Waiting for messages... Press Ctrl+C to exit.", RNS.LOG_VERBOSE)


    def init_csv_log(self):
        write_header = not os.path.exists(self.log_file_path) or os.path.getsize(self.log_file_path) == 0
        try:
            with open(self.log_file_path, 'a', newline='') as csvfile:
                log_writer = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                if write_header:
                    log_writer.writerow([
                        "rx_timestamp_utc", "source_hash", "latitude", "longitude", "altitude_m",
                        "gps_time_utc", "gps_date_utc", "satellites", "hdop",
                        "battery_v", "fix_status_code", "fix_status_str", "fw_major", "fw_minor", "raw_payload_hex"
                    ])
            RNS.log(f"Logging data to {self.log_file_path}", RNS.LOG_INFO)
        except Exception as e:
            RNS.log(f"Error initializing CSV log '{self.log_file_path}': {e}", RNS.LOG_ERROR)


    def log_to_csv(self, data_dict):
        try:
            with open(self.log_file_path, 'a', newline='') as csvfile:
                log_writer = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                log_writer.writerow([
                    data_dict.get("rx_timestamp_utc"), data_dict.get("source_hash"),
                    data_dict.get("latitude"), data_dict.get("longitude"), data_dict.get("altitude_m"),
                    f"{data_dict.get('gps_time_utc'):08d}", f"{data_dict.get('gps_date_utc'):06d}",
                    data_dict.get("satellites"), data_dict.get("hdop"),
                    data_dict.get("battery_v"), data_dict.get("fix_status_code"), data_dict.get("fix_status_str"),
                    data_dict.get("fw_major"), data_dict.get("fw_minor"),
                    data_dict.get("raw_payload_hex")
                ])
        except Exception as e:
            RNS.log(f"Error writing to CSV log '{self.log_file_path}': {e}", RNS.LOG_ERROR)

    def message_received_callback(self, message):
        try:
            source_hash_hex = RNS.prettyhex(message.source_hash)
            RNS.log(f"LXMF Message received from {source_hash_hex}", RNS.LOG_VERBOSE)
            RNS.log(f"  Title: {message.title.decode('utf-8') if message.title else 'N/A'}", RNS.LOG_DEBUG)
            RNS.log(f"  Content size: {len(message.content)} bytes", RNS.LOG_DEBUG)

            rx_timestamp = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S')

            if len(message.content) == ASSET_DATA_SIZE:
                # Unpack data according to the defined format
                data = struct.unpack(ASSET_DATA_FORMAT, message.content)
                latitude, longitude, altitude, gps_time, gps_date, satellites, \
                hdop, battery_voltage, fix_status_raw, fw_major, fw_minor = data

                fix_status_map = {0: "NoFix", 1: "NewFix", 2: "LastKnownFix"}
                fix_status_str = fix_status_map.get(fix_status_raw, f"Unknown({fix_status_raw})")

                print("\n--- Asset Location Update ---")
                print(f"  Received At: {rx_timestamp} UTC")
                print(f"  Source Hash: {source_hash_hex}")
                print(f"  Latitude:    {latitude:.6f}")
                print(f"  Longitude:   {longitude:.6f}")
                print(f"  Altitude:    {altitude:.1f} m")
                print(f"  GPS Time:    {gps_time:08d} UTC (HHMMSSCC)") # Assumes tracker sends HHMMSSCC
                print(f"  GPS Date:    {gps_date:06d} UTC (DDMMYY)")   # Assumes tracker sends DDMMYY
                print(f"  Satellites:  {satellites}")
                print(f"  HDOP:        {hdop:.2f}")
                print(f"  Battery:     {battery_voltage:.2f} V")
                print(f"  Fix Status:  {fix_status_str} (Code: {fix_status_raw})")
                print(f"  Firmware:    v{fw_major}.{fw_minor}")
                print("-----------------------------")

                log_data = {
                    "rx_timestamp_utc": rx_timestamp, "source_hash": source_hash_hex,
                    "latitude": f"{latitude:.6f}", "longitude": f"{longitude:.6f}", "altitude_m": f"{altitude:.1f}",
                    "gps_time_utc": gps_time, "gps_date_utc": gps_date,
                    "satellites": satellites, "hdop": f"{hdop:.2f}",
                    "battery_v": f"{battery_voltage:.2f}",
                    "fix_status_code": fix_status_raw, "fix_status_str": fix_status_str,
                    "fw_major": fw_major, "fw_minor": fw_minor,
                    "raw_payload_hex": message.content.hex()
                }
                self.log_to_csv(log_data)

            else:
                RNS.log(f"Received message from {source_hash_hex} with unexpected content size: {len(message.content)} bytes. Expected {ASSET_DATA_SIZE}.", RNS.LOG_WARNING)
                RNS.log(f"Raw content (hex): {message.content.hex()}", RNS.LOG_DEBUG)

        except struct.error as e:
            RNS.log(f"Error unpacking message content: {e}. Size: {len(message.content)}, Expected format: {ASSET_DATA_FORMAT}", RNS.LOG_ERROR)
        except Exception as e:
            RNS.log(f"Error processing received message: {e}", RNS.LOG_ERROR)
            import traceback
            traceback.print_exc()


def main():
    parser = argparse.ArgumentParser(description="Akita Heltec Reticulum Tracker - LXMF Receiver (Enhanced)")
    parser.add_argument(
        "-c", "--config",
        action="store",
        default=None,
        help="Path to alternative Reticulum config directory"
    )
    parser.add_argument(
        "--app_name",
        action="store",
        default=DEFAULT_DESTINATION_APP_NAME,
        help=f"The application name to listen for (default: {DEFAULT_DESTINATION_APP_NAME})"
    )
    parser.add_argument(
        "--log_file",
        action="store",
        default=DEFAULT_CSV_LOG_FILE,
        help=f"Path to CSV log file (default: {DEFAULT_CSV_LOG_FILE})"
    )
    args = parser.parse_args()

    try:
        receiver = AssetReceiver(
            rns_configdir=args.config,
            destination_app_name=args.app_name,
            log_file=args.log_file
        )

        while True: # Keep the main thread alive to allow callbacks
            time.sleep(1)

    except KeyboardInterrupt:
        RNS.log("\nUser interrupted. Shutting down...", RNS.LOG_INFO)
        # Optional: Add cleanup for RNS components if necessary, though Reticulum usually handles it.
        # if receiver and hasattr(receiver, 'reticulum') and receiver.reticulum:
        #    receiver.reticulum.exit_handler()
        print("Receiver stopped.")
        return
    except Exception as e:
        RNS.log(f"A critical error occurred in main: {e}", RNS.LOG_CRITICAL)
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
