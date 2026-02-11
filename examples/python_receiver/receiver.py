"""Simple LoRa AssetData parser for testing raw packet payloads.

Usage:
  python receiver.py --input-file packet.bin

Note: This script does not perform radio reception. It parses a binary file
containing a single raw packet payload matching the tracker `AssetData` struct
and appends a CSV row. Integrate with your LoRa hardware/library for live
reception.
"""

import struct
import argparse
import csv
import os
from datetime import datetime

# Matches C++ struct layout in the tracker firmware
ASSET_DATA_FORMAT = "<ddfIIBffBBB"  # little-endian
ASSET_DATA_SIZE = struct.calcsize(ASSET_DATA_FORMAT)

DEFAULT_CSV_LOG_FILE = "asset_tracker_log.csv"


class AssetReceiver:
    def __init__(self, log_file=None):
        self.log_file_path = log_file or DEFAULT_CSV_LOG_FILE
        self._ensure_csv()

    def _ensure_csv(self):
        if not os.path.exists(self.log_file_path):
            with open(self.log_file_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    "rx_timestamp", "latitude", "longitude", "altitude",
                    "gps_time", "gps_date", "satellites", "hdop",
                    "battery_voltage", "fix_status", "fw_major", "fw_minor"
                ])

    def parse_asset_data(self, data: bytes):
        if len(data) != ASSET_DATA_SIZE:
            print(f"Invalid data length: {len(data)} (expected {ASSET_DATA_SIZE})")
            return None
        try:
            unpacked = struct.unpack(ASSET_DATA_FORMAT, data)
            return {
                "latitude": unpacked[0],
                "longitude": unpacked[1],
                "altitude": unpacked[2],
                "gps_time": unpacked[3],
                "gps_date": unpacked[4],
                "satellites": unpacked[5],
                "hdop": unpacked[6],
                "battery_voltage": unpacked[7],
                "fix_status": unpacked[8],
                "fw_major": unpacked[9],
                "fw_minor": unpacked[10],
            }
        except struct.error as e:
            print(f"Unpack error: {e}")
            return None

    def log_to_csv(self, parsed: dict):
        with open(self.log_file_path, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                datetime.utcnow().isoformat(),
                parsed["latitude"], parsed["longitude"], parsed["altitude"],
                parsed["gps_time"], parsed["gps_date"], parsed["satellites"], parsed["hdop"],
                parsed["battery_voltage"], parsed["fix_status"], parsed["fw_major"], parsed["fw_minor"]
            ])


def main():
    p = argparse.ArgumentParser(description="Parse raw AssetData packet payload")
    p.add_argument("--input-file", required=True, help="Binary file containing raw payload to parse")
    p.add_argument("--log-file", default=DEFAULT_CSV_LOG_FILE, help="CSV output file")
    args = p.parse_args()

    r = AssetReceiver(log_file=args.log_file)
    with open(args.input_file, "rb") as f:
        payload = f.read()
    parsed = r.parse_asset_data(payload)
    if parsed:
        print("Parsed:", parsed)
        r.log_to_csv(parsed)
    else:
        print("Failed to parse payload")


if __name__ == "__main__":
    main()
