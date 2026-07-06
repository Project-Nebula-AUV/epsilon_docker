#!/usr/bin/env python3
"""Quick ESP32 depth-bridge check (run inside robosub_dev):
    python3 sysid/esp32_check.py
Resets the chip (the stream does not flow on a plain open) and reports what
the firmware says. Healthy: 'STREAMING' lines with ambient ~960-1000 mbar.
'INIT-FAIL' = the ESP32 booted but could not talk to the MS5837 -> check the
sensor-to-Xiao wiring; re-run (init is a per-boot attempt, retry often wins).
"""
import json
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'

for attempt in range(1, 4):
    try:
        s = serial.Serial(PORT, 115200, timeout=1)
    except Exception as e:
        print('cannot open %s: %s' % (PORT, e))
        sys.exit(1)
    s.dtr = False
    s.rts = True
    time.sleep(0.1)
    s.rts = False
    t0 = time.monotonic()
    verdict, shown = 'SILENT (no output in 6 s)', 0
    while time.monotonic() - t0 < 6:
        line = s.readline().decode(errors='replace').strip()
        if not line:
            continue
        try:
            d = json.loads(line)
        except ValueError:
            continue  # ROM boot banner
        if 'error' in d:
            verdict = 'INIT-FAIL: %s' % d['error']
            break
        if 'pressure' in d:
            print('  %s' % line)
            shown += 1
            if shown >= 5:
                verdict = 'STREAMING (healthy)'
                break
    s.close()
    print('attempt %d: %s' % (attempt, verdict))
    if verdict.startswith('STREAMING'):
        sys.exit(0)
    time.sleep(1)
print('sensor did not come up after 3 resets -> check MS5837-to-Xiao wiring')
sys.exit(1)
