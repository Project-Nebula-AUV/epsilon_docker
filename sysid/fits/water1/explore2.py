#!/usr/bin/env python3
"""Follow-ups: (a) when do the -0.262 readings occur; (b) post-RUN-DONE ascent
tails in S3/S7; (c) raw accel norm at rest (scale check); (d) ESP32 in-water
stream health (4b): raw rate, raw-vs-fused outliers."""
import numpy as np, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from explore import load, markers, RUNS

for key in ('s3', 's7'):
    r = RUNS[key]
    fu = load(r, 'fused'); dr = load(r, 'depth_raw')
    t0 = fu['t'][0]
    mks = markers(r)
    done = [t for t, x in mks if x.startswith('RUN-DONE')][0]
    print(f'===== {key}: log t0={0:.1f}, RUN-DONE at {done-t0:.1f}s, log end {fu["t"][-1]-t0:.1f}s')
    neg = fu['depth_m'] < -0.05
    if neg.any():
        tt = fu['t'][neg] - t0
        print(f'  fused < -0.05 m: n={neg.sum()} at t={tt.min():.1f}..{tt.max():.1f}s '
              f'values[{fu["depth_m"][neg].min():.3f},{fu["depth_m"][neg].max():.3f}]')
    negr = dr['depth_m'] < -0.05
    if negr.any():
        tt = dr['t'][negr] - t0
        print(f'  raw   < -0.05 m: n={negr.sum()} at t={tt.min():.1f}..{tt.max():.1f}s')
    # first 12 fused + raw samples
    print('  first fused:', np.round(fu['depth_m'][:10], 3), 'at', np.round(fu['t'][:10]-t0, 2))
    print('  first raw  :', np.round(dr['depth_m'][:8], 3))
    # post-run tail
    m = fu['t'] > done
    tt = fu['t'][m] - t0; dd = fu['depth_m'][m]
    if m.sum() > 10:
        print(f'  post-RUN-DONE: {tt[0]:.0f}s->{tt[-1]:.0f}s depth {dd[0]:+.3f} -> {dd[-1]:+.3f}')
        # sample every ~5s
        for target in np.arange(tt[0], tt[-1], 10.0):
            i = np.argmin(np.abs(tt - target))
            print(f'    t={tt[i]:6.1f}s depth={dd[i]:+.3f}')
    # 4b stream health during the LIVE window (countdown start -> done)
    start = [t for t, x in mks if x.startswith('COUNTDOWN-START')][0]
    mm = (dr['t'] >= start) & (dr['t'] <= done)
    tr = dr['t'][mm]; zr = dr['depth_m'][mm]
    dt = np.diff(tr)
    print(f'  4b raw stream in-window: {len(tr)} lines, rate {1/np.median(dt):.2f} Hz, '
          f'max gap {dt.max():.2f}s, gaps>0.5s: {(dt>0.5).sum()}')
    zf = np.interp(tr, fu['t'], fu['depth_m'])
    resid = zr - zf
    print(f'  raw-vs-fused: |resid| p50 {np.percentile(np.abs(resid),50)*1000:.0f}mm '
          f'p99 {np.percentile(np.abs(resid),99)*1000:.0f}mm  n>10cm: {(np.abs(resid)>0.10).sum()}')

# (c) raw accel norm at rest, S1
r = RUNS['s1']
im = load(r, 'imu')
t0 = im['t'][0]
use = (im['t'] - t0 > 20) & (im['t'] - t0 < 120)
ax, ay, az = im['ax'][use], im['ay'][use], im['az'][use]
corrupt = (ax == 0) & (ay == 0) & (az == 0)
norm = np.sqrt(ax**2 + ay**2 + az**2)[~corrupt]
print(f'\n===== S1 raw accel: |a| mean {norm.mean():.3f} median {np.median(norm):.3f} '
      f'std {norm.std():.3f}  (g=9.80 -> scale factor {np.median(norm)/9.80:.3f})')
print(f'  ax {np.median(ax[~corrupt]):+.2f} ay {np.median(ay[~corrupt]):+.2f} az {np.median(az[~corrupt]):+.2f}')
gr = load(r, 'gravity')
gnorm = np.sqrt(gr['gx']**2 + gr['gy']**2 + gr['gz']**2)
print(f'  gravity.csv |g| median {np.median(gnorm):.4f}')
