#!/usr/bin/env python3
"""MS5837 sanity + 30 s continuous read. Prints every attempt, live."""
import sys
import time

sys.path.insert(0, '/home/robosub/robosub_ws/src/epsilon_sensors/epsilon_sensors')
import ms5837

print('--- sanity: init (reset + PROM/CRC) ---', flush=True)
s = ms5837.MS5837(model=ms5837.MODEL_30BA, bus=1)
t0 = time.monotonic()
ok = False
for attempt in range(5):
    try:
        ok = s.init()
    except Exception as e:
        print('init attempt %d: EXCEPTION %s' % (attempt + 1, e), flush=True)
        ok = False
    else:
        print('init attempt %d: %s' % (attempt + 1, 'OK' if ok else 'FAILED (bus/CRC)'), flush=True)
    if ok:
        break
    time.sleep(1.0)

if not ok:
    print('RESULT: sensor never initialized -- continuing to raw-probe for 30 s anyway', flush=True)

print('--- 30 s PACED read (1 attempt/s) ---', flush=True)
t_start = time.monotonic()
n_try = n_ok = 0
while time.monotonic() - t_start < 30.0:
    ts = time.monotonic() - t_start
    n_try += 1
    try:
        if not ok:
            # sensor uninitialized: retry init each pass instead of read
            ok = s.init()
            print('[%6.2fs] re-init: %s' % (ts, 'OK' if ok else 'fail'), flush=True)
            if not ok:
                time.sleep(1.0)
                continue
        if s.read(ms5837.OSR_1024):
            n_ok += 1
            print('[%6.2fs] OK  pressure %.1f mbar  temp %.2f C  depth %.3f m'
                  % (ts, s.pressure(), s.temperature(), s.depth()), flush=True)
        else:
            print('[%6.2fs] read returned False (corrupt/bus error)' % ts, flush=True)
    except Exception as e:
        print('[%6.2fs] EXCEPTION %s' % (ts, e), flush=True)
    time.sleep(1.0)  # 1 Hz: well inside the bus duty-cycle (healthy read ~40 ms)

print('--- summary: %d/%d successful reads in 30 s ---' % (n_ok, n_try), flush=True)
