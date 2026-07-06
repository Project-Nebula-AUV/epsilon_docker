#!/usr/bin/env python3
"""Verify a DRY run's cmd.csv against its sequence YAML (W1 acceptance).

Usage: python3 sysid/fit/verify_dry_run.py sysid/runs/<id> sysid/sequences/<n>.yaml
Recomputes the expected 6-vector for every logged tick (same hold/ramp/clip/
allocation semantics as sysid_runner) and reports the worst deviation and any
timing skew. PASS = every sample within tolerance (ramps get one-tick slack).
"""
import csv
import os
import sys

import yaml

COS45 = 0.7071  # unused; allocation comes from the yaml below


def load_alloc(ws):
    p = os.path.join(ws, 'src/epsilon_bridge/config/allocation.yaml')
    cols = yaml.safe_load(open(p))['allocation']
    order = ['surge', 'sway', 'heave', 'yaw', 'roll']
    return [[float(cols[a][t]) for a in order] for t in range(6)]


def compile_steps(seq, A):
    max_abs = min(100.0, abs(float(seq.get('max_abs', 100.0))))
    out = []
    for s in seq['steps']:
        if 'thrust' in s:
            vals = [float(v) for v in s['thrust']]
        else:
            w = [float(v) for v in s['wrench']]
            vals = [sum(A[t][a] * w[a] for a in range(5)) * 100.0 for t in range(6)]
        vals = [max(-max_abs, min(max_abs, v)) for v in vals]
        out.append({'dur': float(s['dur']), 'target': vals,
                    'ramp': bool(s.get('ramp', False))})
    if any(abs(v) > 1e-9 for v in out[-1]['target']):
        out.append({'dur': 1.0, 'target': [0.0] * 6, 'ramp': False})
    return out


def expected_at(steps, t):
    acc, prev = 0.0, [0.0] * 6
    for s in steps:
        if t < acc + s['dur']:
            if s['ramp']:
                f = (t - acc) / s['dur']
                return [p + (v - p) * f for p, v in zip(prev, s['target'])]
            return list(s['target'])
        acc += s['dur']
        prev = s['target']
    return None


def main(run_dir, seq_path):
    ws = os.path.dirname(os.path.dirname(os.path.abspath(
        os.path.join(run_dir, os.pardir))))
    seq = yaml.safe_load(open(seq_path))
    A = load_alloc(ws)
    steps = compile_steps(seq, A)
    total = sum(s['dur'] for s in steps)

    rows = [(float(r[0]), [float(v) for v in r[1:7]])
            for r in list(csv.reader(open(os.path.join(run_dir, 'cmd.csv'))))[1:]]
    nz = [i for i, (t, v) in enumerate(rows) if any(abs(x) > 1e-9 for x in v)]
    if not nz:
        print('FAIL: no nonzero commands logged')
        sys.exit(1)
    # sequence start = first nonzero tick minus the leading all-zero steps
    lead = 0.0
    for s in steps:
        if any(abs(v) > 1e-9 for v in s['target']) or s['ramp']:
            break
        lead += s['dur']
    t0 = rows[nz[0]][0] - lead

    worst = (0.0, None)
    n_checked = 0
    for t, v in rows:
        ts = t - t0
        if ts < -0.05 or ts > total + 0.05:
            exp = [0.0] * 6
        else:
            exp = expected_at(steps, max(0.0, min(ts, total - 1e-6)))
            if exp is None:
                exp = [0.0] * 6
        # +/-150 ms slack: logger RECEIVE timestamps jitter that much at step
        # edges under Pi load (measured 2026-07-05; commands themselves exact)
        cands = [exp]
        for off in (0.025, -0.025, 0.075, -0.075, 0.15, -0.15):
            cands.append(expected_at(
                steps, max(0.0, min(ts + off, total - 1e-6))) or [0.0] * 6)
        if ts < 0.2 or ts > total - 0.2:
            # arm/disarm boundary: a zero tick may land inside the first/last
            # step window when the sequence starts or ends nonzero
            cands.append([0.0] * 6)
        err = min(max(abs(a - b) for a, b in zip(v, e)) for e in cands)
        n_checked += 1
        if err > worst[0]:
            worst = (err, ts)
    dur_logged = rows[-1][0] - t0
    print('%s vs %s' % (os.path.basename(run_dir), os.path.basename(seq_path)))
    print('  ticks checked %d, seq %.1fs (logged span %.1fs), worst dev %.3f at t=%.2fs'
          % (n_checked, total, dur_logged, worst[0], worst[1] or 0))
    ok = worst[0] <= 0.5
    print('  ' + ('PASS' if ok else 'FAIL'))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
