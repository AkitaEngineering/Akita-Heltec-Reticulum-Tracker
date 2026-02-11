# LoRa Asset Tracker Receiver Example - Updated for direct LoRa transmission
# This script assumes you have a way to receive raw LoRa packets (e.g., via another LoRa module)
# For demonstration, it can parse raw data from a file or stdin

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

DEFAULT_CSV_LOG_FILE = "asset_tracker_log.csv"

class AssetReceiver:
    def __init__(self, log_file=None):
        self.log_file_path = log_file or DEFAULT_CSV_LOG_FILE
        self.init_csv()

    def init_csv(self):
        if not os.path.exists(self.log_file_path):
            with open(self.log_file_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    'timestamp', 'latitude', 'longitude', 'altitude', 
                    'gps_time', 'gps_date', 'satellites', 'hdop', 
                    'battery_voltage', 'fix_status', 'fw_major', 'fw_minor'
                ])

    def parse_asset_data(self, data):
        if len(data) != ASSET_DATA_SIZE:
            print(f"Invalid data size: {len(data)}, expected {ASSET_DATA_SIZE}")
            return None
        
        try:
    def parse_asset_data(self, data):
        if len(data) != ASSET_DATA_SIZE:
            print(f"Invalid data size: {len(data)}, expected {ASSET_DATA_SIZE}")
            return None
        
        try:
            unpacked = struct.unpack(ASSET_DATA_FORMAT, data)
            return {
                'latitude': unpacked[0],
                'longitude': unpacked[1],
                'altitude': unpacked[2],
                'gps_time': unpacked[3],
                'gps_date': unpacked[4],
                'satellites': unpacked[5],
                'hdop': unpacked[6],
                'battery_voltage': unpacked[7],
                'fix_status': unpacked[8],
                'fw_major': unpacked[9],
                'fw_minor': unpacked[10]
            }
        except struct.error as e:
            print(f"Error unpacking data: {e}")
            return None

    def log_to_csv(self, data):
        with open(self.log_file_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                datetime.now().isoformat(),
                data['latitude'], data['longitude'], data['altitude'],
                data['gps_time'], data['gps_date'], data['satellites'],
                data['hdop'], data['battery_voltage'], data['fix_status'],
                data['fw_major'], data['fw_minor']
            ])

    def process_packet(self, raw_data):
        parsed = self.parse_asset_data(raw_data)
        if parsed:
            print(f"Received asset data: {parsed}")
            self.log_to_csv(parsed)
        else:
            print("Failed to parse asset data")

def main():
    parser = argparse.ArgumentParser(description='LoRa Asset Tracker Receiver')
    parser.add_argument('--log-file', default=DEFAULT_CSV_LOG_FILE, help='CSV log file path')
    parser.add_argument('--input-file', help='File containing raw packet data for testing')
    
    args = parser.parse_args()
    
    receiver = AssetReceiver(log_file=args.log_file)
    
    if args.input_file:
        # Read from file for testing
        with open(args.input_file, 'rb') as f:
            data = f.read()
            receiver.process_packet(data)
    else:
        print("No input file specified. Use --input-file to test with raw data.")
        print("For actual LoRa reception, integrate with a LoRa library like pyLoRa.")

if __name__ == "__main__":
    main()

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
