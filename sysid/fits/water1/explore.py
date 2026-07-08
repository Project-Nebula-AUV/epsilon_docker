#!/usr/bin/env python3
"""Quick-look pass over water session 1 runs: segment, spot bottom contact,
sanity-check channels before fitting."""
import numpy as np, csv, io, os, re, sys

BASE = os.path.dirname(os.path.abspath(__file__))
RUNS = {
    's1': '20260707-023942-s1_static_trim-hand',
    's2': '20260707-025309-s2_tilt_release-hand',
    's3': '20260707-065749-s3_heave_staircase-live',
    's4': '20260707-071456-s4_yaw_staircase_trim-live',
    's7': '20260707-072200-s7_single_thruster_trim-live',
}

def load(run, name):
    p = os.path.join(BASE, 'runs', run, name + '.csv')
    with open(p) as f:
        rdr = csv.reader(f)
        hdr = next(rdr)
        rows = []
        for r in rdr:
            if len(r) != len(hdr):
                continue
            rows.append(r)
    out = {}
    arr = np.array(rows)
    for i, h in enumerate(hdr):
        if h == 'text':
            out[h] = arr[:, i]
        else:
            out[h] = np.array([float(x) if x not in ('', 'nan') else np.nan for x in arr[:, i]])
    return out

def markers(run):
    p = os.path.join(BASE, 'runs', run, 'markers.csv')
    out = []
    with open(p) as f:
        rdr = csv.reader(f)
        next(rdr)
        for r in rdr:
            out.append((float(r[0]), r[1]))
    return out

def seg_stats(t, v, t0, t1, label, tail_frac=0.5):
    m = (t >= t0) & (t < t1)
    if m.sum() < 3:
        print(f'    {label}: <3 samples')
        return None
    vv = v[m]; tt = t[m]
    # steady window = last tail_frac of the segment
    n0 = int(len(vv) * (1 - tail_frac))
    steady = vv[n0:]
    return dict(label=label, n=int(m.sum()), mean=float(np.nanmean(steady)),
                std=float(np.nanstd(steady)), first=float(vv[0]), last=float(vv[-1]),
                vmin=float(np.nanmin(vv)), vmax=float(np.nanmax(vv)),
                slope=float(np.polyfit(tt - tt[0], vv, 1)[0]) if len(vv) > 5 else np.nan)

def p(s):
    if s is None: return
    print(f"    {s['label']:24s} n={s['n']:4d} steady={s['mean']:+8.3f}±{s['std']:.3f} "
          f"range[{s['vmin']:+.3f},{s['vmax']:+.3f}] slope={s['slope']:+.4f}/s")

# ---------------- S1 ----------------
print('================ S1 static trim ================')
r = RUNS['s1']
fu = load(r, 'fused'); at = load(r, 'attitude'); im = load(r, 'imu'); dr = load(r, 'depth_raw')
t0 = fu['t'][0]; tend = fu['t'][-1]
print(f'duration {tend-t0:.1f}s; discard last 33 s per user note -> use [0, {tend-t0-33:.1f}]s')
use = (fu['t'] < tend - 33)
print(f'depth fused: mean {np.nanmean(fu["depth_m"][use]):+.4f} std {np.nanstd(fu["depth_m"][use]):.4f} '
      f'min {np.nanmin(fu["depth_m"][use]):+.4f} max {np.nanmax(fu["depth_m"][use]):+.4f}')
usei = (im['t'] < tend - 33)
for ch in ('eroll_deg', 'epitch_deg', 'eyaw_deg'):
    v = im[ch][usei]
    # unwrap-ish for yaw
    print(f'imu {ch}: mean {np.nanmean(v):+7.2f} std {np.nanstd(v):6.3f} min {np.nanmin(v):+7.2f} max {np.nanmax(v):+7.2f}')
ua = (at['t'] < tend - 33)
print(f'attitude heading: mean {np.nanmean(at["heading_deg"][ua]):7.2f} std {np.nanstd(at["heading_deg"][ua]):.2f}; '
      f'roll(navport+): mean {np.nanmean(at["roll_deg"][ua]):+6.2f} std {np.nanstd(at["roll_deg"][ua]):.2f}')
print(f'depth_raw rate: {len(dr["t"])/(dr["t"][-1]-dr["t"][0]):.2f} Hz; fused rate: {len(fu["t"])/(tend-t0):.2f} Hz')

# ---------------- S2 ----------------
print('\n================ S2 tilt release ================')
r = RUNS['s2']
im = load(r, 'imu'); mks = markers(r)
lvl_eroll = None
for mt, mtext in mks:
    # analysis window: marker+4s (in water, released) to marker+18s or next marker-15s
    t_lo = mt + 4.0
    t_hi = mt + 20.0
    m = (im['t'] >= t_lo) & (im['t'] <= t_hi)
    er = im['eroll_deg'][m]; ep = im['epitch_deg'][m]; tt = im['t'][m]
    if len(er) < 5:
        print(f'  {mtext}: no data'); continue
    print(f'  {mtext}: window {t_hi-t_lo:.0f}s n={len(er)} '
          f'eroll[{np.nanmin(er):+7.1f},{np.nanmax(er):+7.1f}] end {np.nanmean(er[-20:]):+7.1f} | '
          f'epitch[{np.nanmin(ep):+7.1f},{np.nanmax(ep):+7.1f}] end {np.nanmean(ep[-20:]):+7.1f}')

# ---------------- S3 ----------------
print('\n================ S3 heave staircase ================')
r = RUNS['s3']
fu = load(r, 'fused'); dr = load(r, 'depth_raw'); im = load(r, 'imu'); cm = load(r, 'cmd')
mks = [(t, x) for t, x in markers(r) if x.startswith('STEP') or x.startswith('RUN')]
tf = fu['t']; d = fu['depth_m']
print(f'depth overall: min {np.nanmin(d):+.3f} max {np.nanmax(d):+.3f}  (pool 1.52 m, hull height 0.356 m)')
steps = []
for i, (mt, mtext) in enumerate(mks):
    if not mtext.startswith('STEP'): continue
    t_next = mks[i+1][0] if i+1 < len(mks) else tf[-1]
    name = mtext.split()[2] if len(mtext.split()) > 2 else mtext
    s = seg_stats(tf, d, mt, t_next, f'{mtext.split()[1]} {name}')
    p(s)
# velocity between fused samples for terminal-velocity fitting sanity
vz = np.gradient(d, tf)
for i, (mt, mtext) in enumerate(mks):
    if not mtext.startswith('STEP'): continue
    t_next = mks[i+1][0] if i+1 < len(mks) else tf[-1]
    m = (tf >= mt + 1.5) & (tf < t_next)   # skip 1.5s transient
    if m.sum() > 4:
        print(f'    vz steady {mtext.split()[1]:3s} {mtext.split()[2]:18s} {np.nanmedian(vz[m]):+.3f} m/s')

# ---------------- S4 ----------------
print('\n================ S4 yaw staircase ================')
r = RUNS['s4']
im = load(r, 'imu'); at = load(r, 'attitude'); fu = load(r, 'fused')
mks = [(t, x) for t, x in markers(r) if x.startswith('STEP') or x.startswith('RUN')]
ti = im['t']; gz = im['gz']
# corrupt gyro reads: whole group zeros -> mask
corrupt = (im['gx'] == 0) & (im['gy'] == 0) & (im['gz'] == 0)
print(f'gyro corrupt-zero rate: {corrupt.mean()*100:.1f}%')
gzc = np.where(corrupt, np.nan, gz)
for i, (mt, mtext) in enumerate(mks):
    if not mtext.startswith('STEP'): continue
    t_next = mks[i+1][0] if i+1 < len(mks) else ti[-1]
    m = (ti >= mt + 2.0) & (ti < t_next)
    if m.sum() > 4:
        yaw_rate = -np.nanmedian(gzc[m]) * 180/np.pi   # F11: heading rate = -gz
        print(f'    {mtext.split()[1]:3s} {mtext.split()[2]:12s} steady yawrate {yaw_rate:+7.2f} deg/s '
              f'(n={m.sum()}), depth {np.nanmean(np.interp(ti[m], fu["t"], fu["depth_m"])):+.3f}')

# ---------------- S7 ----------------
print('\n================ S7 single thruster ================')
r = RUNS['s7']
im = load(r, 'imu'); fu = load(r, 'fused')
mks = [(t, x) for t, x in markers(r) if x.startswith('STEP')]
ti = im['t']
corrupt = (im['gx'] == 0) & (im['gy'] == 0) & (im['gz'] == 0)
gzc = np.where(corrupt, np.nan, im['gz'])
gxc = np.where(corrupt, np.nan, im['gx'])
gyc = np.where(corrupt, np.nan, im['gy'])
d_at = lambda tt: float(np.interp(tt, fu['t'], fu['depth_m']))
print('pulse: name | depth at pulse | peak |gz|,|gy|,|gx| (rad/s) during pulse')
for i, (mt, mtext) in enumerate(mks):
    parts = mtext.split()
    name = parts[2]
    if not re.match(r't\d[+-]\d+', name): continue
    t_next = mks[i+1][0] if i+1 < len(mks) else ti[-1]
    m = (ti >= mt) & (ti < t_next)
    if m.sum() < 3: continue
    print(f'    {name:7s} depth {d_at(mt):+.3f}  gz {np.nanmax(np.abs(gzc[m])):.3f} '
          f'gy {np.nanmax(np.abs(gyc[m])):.3f} gx {np.nanmax(np.abs(gxc[m])):.3f}')
print('\ndepth trace S7: min %.3f max %.3f' % (np.nanmin(fu['depth_m']), np.nanmax(fu['depth_m'])))
