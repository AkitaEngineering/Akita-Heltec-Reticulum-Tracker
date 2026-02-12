import struct

# Test values matching AssetData struct:
# double latitude; double longitude; float altitude; uint32_t gps_time; uint32_t gps_date;
# uint8_t satellites; float hdop; float battery_voltage; uint8_t fix_status; uint8_t fw_major; uint8_t fw_minor

lat = 51.507351
lon = -0.127758
alt = 15.2
gps_time = 12345678  # HHMMSSCC style placeholder
gps_date = 110226   # DDMMYY placeholder
satellites = 7
hdop = 0.9
battery = 3.78
fix_status = 1
fw_major = 1
fw_minor = 1

packed = struct.pack('<ddfIIBffBBB', lat, lon, alt, gps_time, gps_date, satellites, hdop, battery, fix_status, fw_major, fw_minor)
with open('test_payload.bin', 'wb') as f:
    f.write(packed)
print('Wrote test_payload.bin ({} bytes)'.format(len(packed)))
