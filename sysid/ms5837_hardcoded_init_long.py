#!/usr/bin/env python3
"""MS5837 long read test, init BYPASSED (hardcoded calibration).

Same as ms5837_hardcoded_init_30s.py but runs for DURATION seconds (default
600) and prints a rolling 10 s window summary instead of every attempt, so a
human flexing the wiring gets live per-window feedback: which 10 s slices had
contact, and whether any read was ever VALID.
"""
import sys
import time

sys.path.insert(0, '/home/robosub/robosub_ws/src/epsilon_sensors/epsilon_sensors')
import ms5837

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
WINDOW = 10.0

s = ms5837.MS5837(model=ms5837.MODEL_30BA, bus=2)
s._C = [0, 34982, 36352, 20328, 22354, 26646, 26146]
print('--- init BYPASSED, %.0f s run, %.0f s windows ---' % (DURATION, WINDOW), flush=True)

t_start = time.monotonic()
tot = {'try': 0, 'ok': 0, 'ack': 0, 'exc': 0}
win = {'try': 0, 'ok': 0, 'ack': 0, 'exc': 0}
win_start = t_start
while time.monotonic() - t_start < DURATION:
    tot['try'] += 1
    win['try'] += 1
    try:
        if s.read(ms5837.OSR_1024):
            tot['ok'] += 1
            win['ok'] += 1
            print('[%7.2fs] VALID READ  pressure %.1f mbar  temp %.2f C (approx cal)'
                  % (time.monotonic() - t_start, s.pressure(), s.temperature()),
                  flush=True)
        else:
            tot['ack'] += 1
            win['ack'] += 1
    except Exception:
        tot['exc'] += 1
        win['exc'] += 1
    now = time.monotonic()
    if now - win_start >= WINDOW:
        pct = 100.0 * (win['ok'] + win['ack']) / max(1, win['try'])
        print('[%7.2fs] window: %2d tries | %d valid | %d ack-corrupt | %d no-ack  (contact %3.0f%%)'
              % (now - t_start, win['try'], win['ok'], win['ack'], win['exc'], pct),
              flush=True)
        win = {'try': 0, 'ok': 0, 'ack': 0, 'exc': 0}
        win_start = now
    time.sleep(0.25)

pct = 100.0 * (tot['ok'] + tot['ack']) / max(1, tot['try'])
print('--- TOTAL: %d tries | %d VALID | %d ack-corrupt | %d no-ack (contact %.1f%%) ---'
      % (tot['try'], tot['ok'], tot['ack'], tot['exc'], pct), flush=True)
