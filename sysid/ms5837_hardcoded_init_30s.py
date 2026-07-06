#!/usr/bin/env python3
"""MS5837 30 s read with init BYPASSED (hardcoded calibration).

Skips reset + PROM read + CRC entirely: stuffs representative 30BA calibration
constants into the driver and goes straight to D1/D2 conversions. Pressure/
temp numbers are only ballpark (not this unit's factory calibration) -- the
signal here is whether conversion commands get ACKs + plausible ADC data at
all when the PROM phase is taken out of the path.
"""
import sys
import time

sys.path.insert(0, '/home/robosub/robosub_ws/src/epsilon_sensors/epsilon_sensors')
import ms5837

s = ms5837.MS5837(model=ms5837.MODEL_30BA, bus=2)
# Representative MS5837-30BA PROM words (BlueRobotics-class unit). C[0] is the
# factory/CRC word and is unused once CRC is skipped.
s._C = [0, 34982, 36352, 20328, 22354, 26646, 26146]
print('--- init BYPASSED: hardcoded 30BA calibration, straight to reads ---', flush=True)

t_start = time.monotonic()
n_try = n_ok = n_false = n_exc = 0
last_exc = ''
while time.monotonic() - t_start < 30.0:
    ts = time.monotonic() - t_start
    n_try += 1
    try:
        if s.read(ms5837.OSR_1024):
            n_ok += 1
            print('[%6.2fs] OK  pressure %.1f mbar  temp %.2f C  (approx cal!)'
                  % (ts, s.pressure(), s.temperature()), flush=True)
        else:
            n_false += 1
            print('[%6.2fs] ACKed but corrupt/implausible ADC (read False)' % ts, flush=True)
    except Exception as e:
        n_exc += 1
        last_exc = str(e)
        print('[%6.2fs] EXCEPTION %s' % (ts, e), flush=True)
    time.sleep(0.3)

print('--- summary: %d attempts | %d OK | %d ACK-but-corrupt | %d no-ACK exceptions (%s) ---'
      % (n_try, n_ok, n_false, n_exc, last_exc or '-'), flush=True)
