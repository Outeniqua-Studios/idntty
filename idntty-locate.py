#!/usr/bin/env python3
import serial
import time
import re

def send_at_command(ser, command, timeout=3):
    ser.write((command + "\r\n").encode())
    ser.flush()
    time.sleep(timeout)
    return ser.read(ser.in_waiting).decode()

def parse_gps_info(gps_info):
    match = re.search(r'\+CGPSINFO: (\d+\.\d+),([NS]),(\d+\.\d+),([EW])', gps_info)
    if match:
        lat, lat_dir, lon, lon_dir = match.groups()
        lat = float(lat[:2]) + float(lat[2:]) / 60
        lon = float(lon[:3]) + float(lon[3:]) / 60
        if lat_dir == 'S':
            lat = -lat
        if lon_dir == 'W':
            lon = -lon
        return lat, lon
    else:
        return None, None

def get_gps_location(port="/dev/ttyS0", baudrate=115200):
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            # Turn on GPS
            send_at_command(ser, "AT+CGPS=1", timeout=1)
            time.sleep(60)  # Wait for GPS fix

            # Get GPS information
            response = send_at_command(ser, "AT+CGPSINFO", timeout=1)
            lat, lon = parse_gps_info(response)
            return lat, lon

    except Exception as e:
        print("Error:", e)
        return None, None

if __name__ == '__main__':
    lat, lon = get_gps_location()
    if lat is not None and lon is not None:
        print(f"Latitude: {lat}, Longitude: {lon}")
    else:
        print("Failed to get GPS location")
