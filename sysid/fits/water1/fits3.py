#!/usr/bin/env python3
"""S2 decay refit WITH equilibrium offset: theta'' = -K*(theta-th0) - C*theta'"""
import numpy as np, os, sys, json
from scipy.optimize import least_squares
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from explore import load, markers, RUNS

LEVEL_EROLL, LEVEL_EPITCH = -170.17, -18.19
def unwrap_deg(a): return np.degrees(np.unwrap(np.radians(a)))
im2 = load(RUNS['s2'], 'imu')
c = (im2['gx']==0)&(im2['gy']==0)&(im2['gz']==0)
for k in ('gx','gy','gz'):
    im2[k] = np.where(c, np.nan, im2[k])
mks2 = markers(RUNS['s2']); ti2 = im2['t']
ok2 = ~((im2['eroll_deg']==0)&(im2['epitch_deg']==0)&(im2['eyaw_deg']==0))

def fit(tt, ang, rate):
    i0 = int(np.nanargmax(np.abs(ang)))
    i1 = len(ang)-1
    tw, aw, rw = tt[i0:i1]-tt[i0], ang[i0:i1], rate[i0:i1]
    ok = ~np.isnan(aw)&~np.isnan(rw)
    tw, aw, rw = tw[ok], aw[ok], rw[ok]
    if len(tw) < 20: return None
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
        f = least_squares(resid, [1.0,1.0,0.0], bounds=([0.005,0.0,-0.6],[200,100,0.6]))
        return dict(K=float(f.x[0]), C=float(f.x[1]), th0_deg=float(np.degrees(f.x[2])),
                    peak=float(np.degrees(aw[0])), rms=float(np.degrees(np.sqrt(np.mean(f.fun**2)))),
                    dur=float(tw[-1]))
    except Exception:
        return None

out = {'roll': [], 'pitch': []}
for mt, mtext in mks2:
    nxt = min([t for t,_ in mks2 if t>mt], default=ti2[-1])
    lo, hi = mt+4.0, min(mt+30.0, nxt-15.0, ti2[-1])
    m = (ti2>=lo)&(ti2<=hi)&ok2
    if m.sum()<30: continue
    if 'roll' in mtext:
        f = fit(ti2[m], np.radians(im2['epitch_deg'][m]-LEVEL_EPITCH), -im2['gy'][m])
        if f: f['rep']=mtext; out['roll'].append(f)
    else:
        er = unwrap_deg(im2['eroll_deg'][m])
        er = er - 360.0*np.round((np.nanmedian(er)-LEVEL_EROLL)/360.0)
        f = fit(ti2[m], np.radians(-(er-LEVEL_EROLL)), -im2['gx'][m])
        if f: f['rep']=mtext; out['pitch'].append(f)
for axn, L in out.items():
    print(axn.upper())
    for f in L:
        print(f"  {f['rep']:14s} K={f['K']:7.3f} C={f['C']:6.3f} th0={f['th0_deg']:+6.1f} "
              f"peak={f['peak']:+6.1f} rms={f['rms']:5.2f} dur={f['dur']:.1f}")
    Ks = [f['K'] for f in L]; Cs = [f['C'] for f in L]
    print(f"  median K={np.median(Ks):.3f} C={np.median(Cs):.3f}")
json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'fits_out','s2_offset_fits.json'),'w'), indent=1)
