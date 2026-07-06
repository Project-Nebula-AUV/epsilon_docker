#!/usr/bin/env python3
"""IMU rest-capture statistics (W2/A4): noise floors, bias drift, true rates.

Usage: python3 sysid/fit/imu_rest_stats.py sysid/runs/<id>
Reads imu.csv (+ gravity.csv). Zeroed quat/gyro/accel fields are corrupt-read
markers from the driver -- they are counted and excluded, not averaged.
Dependency-light on purpose (runs on the Pi): stdlib only.
"""
import csv
import math
import os
import sys


def stats(xs):
    n = len(xs)
    if n < 2:
        return float('nan'), float('nan')
    m = sum(xs) / n
    var = sum((x - m) ** 2 for x in xs) / (n - 1)
    return m, math.sqrt(var)


def drift(ts, xs, m):
    """Least-squares slope (unit/min) of xs vs ts."""
    if len(xs) < 2:
        return float('nan')
    t0 = ts[0]
    tt = [t - t0 for t in ts]
    tm = sum(tt) / len(tt)
    num = sum((t - tm) * (x - m) for t, x in zip(tt, xs))
    den = sum((t - tm) ** 2 for t in tt)
    return (num / den) * 60.0 if den else float('nan')


def main(run_dir):
    path = os.path.join(run_dir, 'imu.csv')
    rows = list(csv.reader(open(path)))
    hdr, data = rows[0], rows[1:]
    cols = {name: i for i, name in enumerate(hdr)}
    t = [float(r[cols['t']]) for r in data]
    dur = t[-1] - t[0]
    print('run: %s   samples: %d   duration: %.1f s' % (run_dir, len(t), dur))

    dts = [b - a for a, b in zip(t, t[1:])]
    dts_s = sorted(dts)
    print('rate: %.2f Hz mean | dt p50 %.3f p95 %.3f p99 %.3f max %.3f s | gaps>0.2s: %d'
          % (1.0 / (sum(dts) / len(dts)), dts_s[len(dts) // 2],
             dts_s[int(len(dts) * 0.95)], dts_s[int(len(dts) * 0.99)],
             dts_s[-1], sum(1 for d in dts if d > 0.2)))

    groups = {
        'quat': ['qw', 'qx', 'qy', 'qz'],
        'gyro rad/s': ['gx', 'gy', 'gz'],
        'accel m/s2': ['ax', 'ay', 'az'],
        'euler deg': ['eroll_deg', 'epitch_deg', 'eyaw_deg'],
    }
    for gname, names in groups.items():
        # corrupt-read convention: the driver zeroes a whole group it rejects
        idxs = [cols[n] for n in names]
        good_rows = []
        for i, r in enumerate(data):
            vals = [float(r[j]) for j in idxs]
            if gname != 'euler deg' and all(abs(v) < 1e-12 for v in vals):
                continue  # zeroed = corrupt/unavailable
            if any(math.isnan(v) for v in vals):
                continue
            good_rows.append((t[i], vals))
        miss = len(data) - len(good_rows)
        print('%-11s missing/corrupt: %d/%d (%.1f%%)'
              % (gname, miss, len(data), 100.0 * miss / max(1, len(data))))
        for k, n in enumerate(names):
            ts_g = [tv for tv, _ in good_rows]
            xs = [v[k] for _, v in good_rows]
            m, s = stats(xs)
            print('   %-10s mean %+11.6f   std %10.6f   drift %+11.6f /min'
                  % (n, m, s, drift(ts_g, xs, m)))

    gpath = os.path.join(run_dir, 'gravity.csv')
    if os.path.exists(gpath):
        g = list(csv.reader(open(gpath)))[1:]
        if g:
            mags = [math.sqrt(sum(float(r[i]) ** 2 for i in (1, 2, 3))) for r in g]
            m, s = stats(mags)
            print('gravity |g|: mean %.4f std %.4f (n=%d) -- should be ~9.81' % (m, s, len(g)))


if __name__ == '__main__':
    main(sys.argv[1])
