#!/usr/bin/env python3
import time
try:
    import smbus2 as smbus
except:
    print('Install: sudo apt-get install python3-smbus2')
    exit(1)

print("Opening I2C bus 2...")
bus = smbus.SMBus(2)
print("Bus opened successfully")

try:
    print("Attempting to read from 0x76 (sensor address)...")
    data = bus.read_byte(0x76)
    print(f"Success! Read byte: 0x{data:02x}")
except Exception as e:
    print(f"Read failed: {e}")

try:
    print("Attempting soft reset...")
    bus.write_byte(0x76, 0x1E)
    print("Reset command sent")
    time.sleep(0.1)
    print("Waiting for sensor to respond...")
    time.sleep(1.0)
    
    print("Attempting to read calibration data...")
    data = bus.read_byte_data(0x76, 0xA0)
    print(f"Read calibration: 0x{data:02x}")
except Exception as e:
    print(f"Operation failed: {e}")
finally:
    bus.close()
    print("Bus closed")
