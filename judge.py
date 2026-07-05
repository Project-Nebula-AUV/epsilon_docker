#!/usr/bin/env python3
"""Qualitative evidence extractor for sim course runs.

Reads the simulator ground-truth trace + course geometry and prints the
things a human judge cares about: state timeline (with hang suspects), gate
crossing centering, style-roll xyz/heading hold + recovery, slalom pass
sides/clearances, depth-hold quality, end state.

    python3 judge.py <sim_truth.csv> <sim_geom.json>

Not a pass/fail gate — prints evidence for qualitative judgment.
Stdlib only, so it runs identically on the sub and locally.
"""
import csv
import json
import math
import sys
from collections import defaultdict


def load(csv_path, geom_path):
    rows = []
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            try:
                rows.append({
                    't': float(r['t']), 'x': float(r['x']), 'y': float(r['y']),
                    'z': float(r['z']), 'heading': float(r['heading']),
                    'roll': float(r['roll']), 'task': r['task'], 'state': r['state'],
                })
            except (ValueError, KeyError):
                continue
    with open(geom_path) as f:
        geom = json.load(f)
    return rows, geom


def collapse_state(s):
    out, num = [], False
    for ch in s:
        if ch.isdigit() or ch in '+-.':
            num = True
            continue
        if num:
            out.append('#')
            num = False
        out.append(ch)
    return ''.join(out)


def segments(rows):
    """Contiguous (task, collapsed-state) spans."""
    segs = []
    for r in rows:
        key = (r['task'], collapse_state(r['state']))
        if segs and segs[-1]['key'] == key:
            segs[-1]['t1'] = r['t']
            segs[-1]['rows'].append(r)
        else:
            segs.append({'key': key, 't0': r['t'], 't1': r['t'], 'rows': [r]})
    return segs


def unwrap_deg(series):
    out, prev, acc = [], None, 0.0
    for v in series:
        if prev is not None:
            d = v - prev
            if d > 180:
                acc -= 360
            elif d < -180:
                acc += 360
        out.append(v + acc)
        prev = v
    return out


def fmt_t(t):
    return f"{t:7.1f}s"


def main():
    rows, geom = load(sys.argv[1], sys.argv[2])
    if not rows:
        print("no rows")
        return
    print(f"=== RUN: {rows[0]['t']:.1f}s .. {rows[-1]['t']:.1f}s "
          f"({len(rows)} samples) ===")

    segs = segments(rows)

    # --- Timeline (skip WAITING) ---
    print("\n--- STATE TIMELINE (dur > 0.5s; * = >60s HANG-SUSPECT) ---")
    for s in segs:
        dur = s['t1'] - s['t0']
        task, state = s['key']
        if task.startswith('Waiting') or dur < 0.5:
            continue
        mark = ' *' if dur > 60 else ''
        flag = ' [TIMEOUT-STATE]' if 'TIMEOUT' in state.upper() else ''
        print(f"{fmt_t(s['t0'])} +{dur:6.1f}s  {task:24s} {state}{mark}{flag}")

    # --- Gate crossings ---
    g = geom.get('gate')
    if g:
        gx, gcy, gw = g['x'], g['center_y'], g['width']
        print(f"\n--- GATE (x={gx}, center_y={gcy}, half-centers y="
              f"{gcy - gw / 4:.2f}/{gcy + gw / 4:.2f}, opening z>{g['z_top']}) ---")
        for i in range(1, len(rows)):
            a, b = rows[i - 1], rows[i]
            if (a['x'] - gx) * (b['x'] - gx) < 0:
                frac = (gx - a['x']) / (b['x'] - a['x'])
                y = a['y'] + frac * (b['y'] - a['y'])
                z = a['z'] + frac * (b['z'] - a['z'])
                direction = 'OUT' if b['x'] > a['x'] else 'BACK'
                half = 'right(y<c)' if y < gcy else 'left(y>c)'
                half_c = gcy - gw / 4 if y < gcy else gcy + gw / 4
                print(f"{fmt_t(b['t'])} {direction}  y={y:.2f} "
                      f"(off-center {y - gcy:+.2f} m; {half}, "
                      f"off-half-center {y - half_c:+.2f} m)  z={z:.2f} "
                      f"during {b['task']}")

    # --- Style roll ---
    roll_segs = [s for s in segs if 'StyleRoll' in s['key'][1]]
    if roll_segs:
        print("\n--- STYLE ROLL ---")
        t0, t1 = roll_segs[0]['t0'], roll_segs[-1]['t1']
        span = [r for r in rows if t0 <= r['t'] <= t1]
        pre_i = max(0, next(i for i, r in enumerate(rows) if r['t'] >= t0) - 1)
        pre = rows[pre_i]
        un = unwrap_deg([r['roll'] for r in span])
        total_roll = un[-1] - un[0]
        peak_roll = max(abs(v - un[0]) for v in un)
        dx = max(abs(r['x'] - pre['x']) for r in span)
        dy = max(abs(r['y'] - pre['y']) for r in span)
        dz = max(abs(r['z'] - pre['z']) for r in span)
        hd = [abs((r['heading'] - pre['heading'] + 180) % 360 - 180) for r in span]
        end = span[-1]
        print(f"span {fmt_t(t0)}..{fmt_t(t1)} ({t1 - t0:.1f}s)  "
              f"net roll {total_roll:+.0f} deg (peak excursion {peak_roll:.0f})")
        print(f"pre-roll pose x={pre['x']:.2f} y={pre['y']:.2f} z={pre['z']:.2f} "
              f"hdg={pre['heading']:.1f}")
        print(f"max drift during roll: dx={dx:.2f} dy={dy:.2f} dz={dz:.2f} m, "
              f"max heading dev {max(hd):.1f} deg")
        print(f"at roll end: dx={end['x'] - pre['x']:+.2f} "
              f"dy={end['y'] - pre['y']:+.2f} dz={end['z'] - pre['z']:+.2f} "
              f"hdg dev {(end['heading'] - pre['heading'] + 180) % 360 - 180:+.1f} "
              f"roll={end['roll']:+.1f}")
        # recovery: end of the GateTask segment right before drive-through
        after = [r for r in rows if r['t'] > t1 and r['task'] == end['task']]
        drive = next((r for r in after if 'DriveThrough' in r['state'] or 'DriveUntil' in r['state']), None)
        if drive:
            print(f"at drive-through start (t={drive['t']:.1f}): "
                  f"dx={drive['x'] - pre['x']:+.2f} dy={drive['y'] - pre['y']:+.2f} "
                  f"dz={drive['z'] - pre['z']:+.2f} "
                  f"hdg dev {(drive['heading'] - pre['heading'] + 180) % 360 - 180:+.1f}")
        else:
            print("never reached drive-through after the roll")

    # --- Slalom ---
    poles = geom.get('poles', [])
    if poles:
        by_x = defaultdict(list)
        for p in poles:
            by_x[round(p['x'], 2)].append(p)
        print("\n--- SLALOM GATELETS ---")
        for gx_ in sorted(by_x):
            trip = by_x[gx_]
            red = next((p for p in trip if p['red']), None)
            whites = [p for p in trip if not p['red']]
            if not red:
                continue
            for i in range(1, len(rows)):
                a, b = rows[i - 1], rows[i]
                if (a['x'] - gx_) * (b['x'] - gx_) < 0:
                    frac = (gx_ - a['x']) / (b['x'] - a['x'])
                    y = a['y'] + frac * (b['y'] - a['y'])
                    direction = 'OUT' if b['x'] > a['x'] else 'BACK'
                    side = 'left(y>red)' if y > red['y'] else 'right(y<red)'
                    gaps = []
                    for w in whites:
                        lo, hi = sorted((red['y'], w['y']))
                        mid = (lo + hi) / 2
                        gaps.append((abs(y - mid), lo, hi, mid))
                    dmid, lo, hi, mid = min(gaps)
                    inside = lo < y < hi
                    clear = min(abs(y - red['y']),
                                min(abs(y - w['y']) for w in whites))
                    print(f"{fmt_t(b['t'])} {direction}  x={gx_:5.1f} y={y:.2f} "
                          f"{side} {'THROUGH-GAP' if inside else 'OUTSIDE-GAP'} "
                          f"off-gap-mid {y - mid:+.2f} m, nearest pole {clear:.2f} m "
                          f"({b['task']})")

    # --- Depth hold per task ---
    print("\n--- DEPTH BY TASK (target 1.5, gate legs 1.55) ---")
    by_task = defaultdict(list)
    started = False
    for r in rows:
        if not started and not r['task'].startswith('Waiting') \
                and r['task'] != '':
            started = True
        if started and not r['task'].startswith('Waiting'):
            by_task[r['task']].append(r['z'])
    for task, zs in by_task.items():
        mean = sum(zs) / len(zs)
        print(f"{task:24s} z mean {mean:.2f}  min {min(zs):.2f}  max {max(zs):.2f}")

    # --- End state ---
    last = rows[-1]
    print(f"\n--- END: t={last['t']:.1f}s task={last['task']} state={last['state']}")
    print(f"    pose x={last['x']:.2f} y={last['y']:.2f} z={last['z']:.2f} "
          f"hdg={last['heading']:.1f} roll={last['roll']:.1f}")


if __name__ == '__main__':
    main()
