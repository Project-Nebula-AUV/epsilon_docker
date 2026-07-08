#!/usr/bin/env python3
"""Submerged roll righting from S7: post-pulse roll decay after t0/t1 vertical
pulses in free water (depth 0.15-1.75, off bottom). Same offset-decay model."""
import numpy as np, os, sys, re, json
from scipy.optimize import least_squares
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from explore import load, markers, RUNS

LEVEL_EPITCH = -18.19
im = load(RUNS['s7'], 'imu')
c = (im['gx']==0)&(im['gy']==0)&(im['gz']==0)
gy = np.where(c, np.nan, im['gy'])
fu = load(RUNS['s7'], 'fused')
mks = [(t,x) for t,x in markers(RUNS['s7']) if x.startswith('STEP')]
ti = im['t']
results = []
for i,(mt,mtext) in enumerate(mks):
    name = mtext.split()[2]
    mm = re.match(r't([01])([+-]\d+)', name)
    if not mm: continue
    # decay window = the rest step after this pulse
    if i+1 >= len(mks): continue
    t_rest = mks[i+1][0]
    t_end = mks[i+2][0] if i+2 < len(mks) else ti[-1]
    depth = float(np.interp(t_rest, fu['t'], fu['depth_m']))
    if depth > 1.85: continue   # on/near bottom
    m = (ti >= t_rest+0.1) & (ti < t_end)
    if m.sum() < 15: continue
    ang = np.radians(im['epitch_deg'][m]-LEVEL_EPITCH)
    rate = -gy[m]
    tt = ti[m]-ti[m][0] if False else ti[m]
    tw = ti[m]-ti[m][0]
    ok = ~np.isnan(ang)&~np.isnan(rate)
    tw, aw, rw = tw[ok], ang[ok], rate[ok]
    if len(tw) < 12: continue
    def resid(p):
        K, C, th0 = p
        a, v = aw[0], rw[0]; out=[]; tp=tw[0]
        for j in range(len(tw)):
            dt=tw[j]-tp; tp=tw[j]; n=max(1,int(dt/0.01))
            for _ in range(n):
                h=dt/n; v += (-K*(a-th0)-C*v)*h; a += v*h
            out.append(a-aw[j])
        return np.array(out)
    try:
        f = least_squares(resid, [1.0,1.0,0.0], bounds=([0.001,0.0,-0.5],[100,50,0.5]))
        results.append(dict(after=name, depth=round(depth,2),
            start_deg=round(float(np.degrees(aw[0])),1),
            K=round(float(f.x[0]),3), C=round(float(f.x[1]),3),
            th0=round(float(np.degrees(f.x[2])),1),
            rms=round(float(np.degrees(np.sqrt(np.mean(f.fun**2)))),2),
            n=len(tw), dur=round(float(tw[-1]),1)))
    except Exception:
        pass
for r in results: print(r)
Ks = [r['K'] for r in results if abs(r['start_deg']-r['th0']) > 3 and r['rms'] < 3]
Cs = [r['C'] for r in results if abs(r['start_deg']-r['th0']) > 3 and r['rms'] < 3]
print('usable (excursion>3deg, rms<3):', len(Ks), 'K median', np.median(Ks) if Ks else None,
      'C median', np.median(Cs) if Cs else None)
json.dump(results, open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'fits_out','s7_submerged_roll_decay.json'),'w'), indent=1)
