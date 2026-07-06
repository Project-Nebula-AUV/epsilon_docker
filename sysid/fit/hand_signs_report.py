#!/usr/bin/env python3
"""A1 hand-signs run report: which channels moved during each marker window.

Usage: python3 sysid/fit/hand_signs_report.py sysid/runs/<id> [window_s=4]
For each marker in markers.csv, takes the [t, t+window] slice of imu.csv and
attitude.csv and prints each channel's departure from its pre-marker baseline
(peak |delta| and signed peak). The axis map + signs fall straight out:
e.g. marker 'pitch-bow-up' -> whichever euler channel spikes is body pitch,
and its sign at bow-up is the sign convention.
Stdlib only (runs on the Pi).
"""
import csv
import math
import os
import sys


def load(path):
    rows = list(csv.reader(open(path)))
    return rows[0], [[r[0]] + r[1:] for r in rows[1:]]


def channel(data, ti, ci):
    out = []
    for r in data:
        try:
            out.append((float(r[ti]), float(r[ci])))
        except (ValueError, IndexError):
            pass
    return out


def wrap_deg(d):
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


def report(series, name, t_mark, window, is_angle=False):
    base = [v for t, v in series if t_mark - 3.0 <= t < t_mark - 0.2]
    win = [(t, v) for t, v in series if t_mark <= t <= t_mark + window]
    if len(base) < 3 or len(win) < 3:
        return None
    b = sorted(base)[len(base) // 2]
    if is_angle:
        deltas = [(wrap_deg(v - b), t) for t, v in win]
    else:
        deltas = [(v - b, t) for t, v in win]
    peak = max(deltas, key=lambda dv: abs(dv[0]))
    return name, b, peak[0]


def main(run_dir, window=4.0):
    mhdr, marks = load(os.path.join(run_dir, 'markers.csv'))
    ihdr, imu = load(os.path.join(run_dir, 'imu.csv'))
    ahdr, att = load(os.path.join(run_dir, 'attitude.csv'))
    icol = {n: i for i, n in enumerate(ihdr)}
    acol = {n: i for i, n in enumerate(ahdr)}

    chans = []
    for n in ('gx', 'gy', 'gz', 'ax', 'ay', 'az'):
        chans.append((n, channel(imu, icol['t'], icol[n]), False))
    for n in ('eroll_deg', 'epitch_deg', 'eyaw_deg'):
        chans.append((n, channel(imu, icol['t'], icol[n]), True))
    for n in ('heading_deg', 'roll_deg'):
        chans.append((n, channel(att, acol['t'], acol[n]), True))

    for r in marks:
        t_mark = float(r[0])
        text = ','.join(r[1:]).strip('"')
        if text.startswith(('STEP', 'RUN-')):
            continue
        print('\n=== %s  (t=%.1f, window %.1fs) ===' % (text, t_mark, window))
        results = []
        for name, series, is_angle in chans:
            res = report(series, name, t_mark, window, is_angle)
            if res:
                results.append(res)
        results.sort(key=lambda x: -abs(x[2]))
        for name, base, peak in results:
            flag = ' <<<' if abs(peak) > (8.0 if name.endswith('deg') else 0.15) else ''
            print('  %-12s baseline %+9.3f   peak-delta %+9.3f%s'
                  % (name, base, peak, flag))


if __name__ == '__main__':
    main(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 4.0)
