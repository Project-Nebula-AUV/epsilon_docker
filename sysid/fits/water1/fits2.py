#!/usr/bin/env python3
"""Refined fits.
S3: kb (dB/dz) FIXED from the post-run free-sink observation (B~0 at 1.9 m);
    sensitivity sweep kb in {0.4, 0.53, 0.7}. Adds up20 ascent as validation.
S7: initial roll angular acceleration of t0/t1 pulses -> Ix_eff via known
    torque F*0.184 (F from S3 per-motor curve).
S2: normalized decay fit -> k/I_eff, c/I_eff per release; absolute k via Ix_eff.
"""
import numpy as np, os, sys, json, re
from scipy.optimize import least_squares
from scipy.signal import savgol_filter
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from explore import load, markers, RUNS
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fits_out')
MASS = 9.98
LEVEL_EROLL, LEVEL_EPITCH = -170.17, -18.19
ROLL_ARM = 0.184
R = {}

def mask_corrupt(im):
    c = (im['gx'] == 0) & (im['gy'] == 0) & (im['gz'] == 0)
    return {k: (np.where(c, np.nan, v) if k in ('gx','gy','gz') else v) for k, v in im.items()}

# ---------- S3 refit ----------
r = RUNS['s3']
fu = load(r, 'fused')
mks = [(t, x) for t, x in markers(r) if x.startswith('STEP') or x.startswith('RUN-DONE')]
tf, d = fu['t'], fu['depth_m']
ds = savgol_filter(d, 21, 3)
steps = []
for i, (mt, mtext) in enumerate(mks[:-1]):
    tn = mks[i+1][0]
    tgt = mtext.split('target=')[1]
    t0c = float(tgt.strip("[]' ").split("', '")[0])
    steps.append((mt, tn, t0c, mtext.split()[2]))
def cmd_at(t):
    for mt, tn, c, _ in steps:
        if mt <= t < tn: return c
    return 0.0
t_lo = [mt for mt, tn, c, n in steps if n == 'down20'][0]
t_hi = [mt for mt, tn, c, n in steps if n == 'recover-buoyant'][0]
mfit = (tf >= t_lo) & (tf < t_hi) & (ds < 1.95)
tw, zw = tf[mfit], ds[mfit]
vz = savgol_filter(d, 21, 3, deriv=1, delta=np.median(np.diff(tf)))
vw = vz[mfit]
KA = 1.0; M_EFF = MASS*(1+KA); B0 = 1.0

def make_resid(kb):
    def resid(p):
        T20, T30, T40, cq = p
        Tmap = {20.0: T20, 30.0: T30, 40.0: T40}
        z, v = zw[0], vw[0]
        out = []
        tprev = tw[0]
        for j in range(len(tw)):
            dt = tw[j] - tprev; tprev = tw[j]
            n = max(1, int(dt/0.01))
            for _ in range(n):
                h = dt/n
                cmd = cmd_at(tw[j])
                T = Tmap.get(abs(cmd), 0.0)*np.sign(-cmd) if cmd else 0.0
                acc = (T - (B0 - kb*max(z, 0)) - cq*v*abs(v))/M_EFF
                v += acc*h; z += v*h
            out.append(z - zw[j])
        return np.array(out)
    return resid

R['s3'] = {}
for kb in (0.40, 0.53, 0.70):
    fit = least_squares(make_resid(kb), [2.0, 4.0, 6.0, 60.0],
                        bounds=([0, 0, 0, 5], [20, 30, 40, 400]))
    T20, T30, T40, cq = fit.x
    R['s3'][f'kb_{kb}'] = dict(T20=round(float(T20),3), T30=round(float(T30),3),
                               T40=round(float(T40),3), cq=round(float(cq),1),
                               rms_mm=round(float(np.sqrt(np.mean(fit.fun**2)))*1000,1))
# adopt kb=0.53 as central
C = R['s3']['kb_0.53']
T40_motor = C['T40']/2.0; T20_motor = C['T20']/2.0

# quadratic-with-deadband thrust curve through (20,30,40): T_motor = a*(cmd-db)^2
def tc_resid(p):
    a, db = p
    return [a*max(c-db,0)**2 - t for c, t in
            ((20, C['T20']/2), (30, C['T30']/2), (40, C['T40']/2))]
tcfit = least_squares(tc_resid, [0.005, 8.0], bounds=([1e-5, 0], [1, 19]))
a_t, db_t = tcfit.x
R['s3']['thrust_curve_motor'] = dict(a=float(a_t), deadband=float(db_t),
    T100_extrap=float(a_t*(100-db_t)**2),
    note='T_motor(cmd)=a*(cmd-db)^2 through kb=0.53 fit; T100 is a LONG extrapolation')

# ---------- S7 roll-pulse -> Ix_eff ----------
r = RUNS['s7']
im = mask_corrupt(load(r, 'imu'))
mks7 = [(t, x) for t, x in markers(r) if x.startswith('STEP')]
ti = im['t']; wroll = -im['gy']
ix_est = []
for i, (mt, mtext) in enumerate(mks7):
    name = mtext.split()[2]
    mm = re.match(r't([01])([+-])(\d+)', name)
    if not mm: continue
    thr, sgn, amp = int(mm.group(1)), mm.group(2), int(mm.group(3))
    # initial roll accel: slope of wroll over [mt+0.1, mt+0.7]
    m = (ti >= mt + 0.1) & (ti <= mt + 0.7) & ~np.isnan(wroll)
    if m.sum() < 6: continue
    slope = np.polyfit(ti[m]-mt, wroll[m], 1)[0]
    cmd = amp
    Tm = a_t*max(cmd-db_t, 0)**2       # motor thrust magnitude at this cmd
    tau = Tm*ROLL_ARM
    if abs(slope) > 1e-3 and tau > 0:
        ix_est.append(dict(name=name, slope=round(float(slope),3),
                           tau=round(float(tau),3), Ix_eff=round(float(tau/abs(slope)),3)))
R['s7_ix'] = ix_est
ix_vals = [e['Ix_eff'] for e in ix_est if e['name'].endswith('40')]  # 40s: better SNR
IX_EFF = float(np.median(ix_vals)) if ix_vals else None
R['Ix_eff_median_from_40s'] = IX_EFF

# ---------- S2 normalized decay ----------
r = RUNS['s2']
im2 = mask_corrupt(load(r, 'imu'))
mks2 = markers(r)
ti2 = im2['t']
ok2 = ~((im2['eroll_deg']==0)&(im2['epitch_deg']==0)&(im2['eyaw_deg']==0))
def unwrap_deg(a): return np.degrees(np.unwrap(np.radians(a)))

def norm_decay(tt, ang, rate):
    """fit theta'' = -K*theta - Cc*theta'  (K=k/I_eff 1/s^2, Cc=c/I_eff 1/s)"""
    i0 = int(np.nanargmax(np.abs(ang)))
    a0 = ang[i0]
    settled = np.where(np.abs(ang[i0:]) < 0.12*np.abs(a0))[0]
    i1 = i0 + (settled[0] + 15 if len(settled) else len(ang)-i0-1)
    i1 = min(i1, len(ang)-1)
    if i1 - i0 < 10: return None
    tw2, aw, rw = tt[i0:i1]-tt[i0], ang[i0:i1], rate[i0:i1]
    ok = ~np.isnan(aw) & ~np.isnan(rw)
    tw2, aw, rw = tw2[ok], aw[ok], rw[ok]
    if len(tw2) < 10: return None
    def resid(p):
        K, Cc = p
        a, v = aw[0], rw[0]
        out = []; tprev = tw2[0]
        for j in range(len(tw2)):
            dt = tw2[j]-tprev; tprev = tw2[j]
            n = max(1, int(dt/0.01))
            for _ in range(n):
                h = dt/n
                v += (-K*a - Cc*v)*h; a += v*h
            out.append(a - aw[j])
        return np.array(out)
    try:
        f = least_squares(resid, [1.0, 1.0], bounds=([0.005, 0.0], [200, 100]))
        return dict(K=float(f.x[0]), C=float(f.x[1]),
                    peak=float(np.degrees(a0)),
                    rms=float(np.degrees(np.sqrt(np.mean(f.fun**2)))),
                    dur=float(tw2[-1]))
    except Exception:
        return None

s2n = {'roll': [], 'pitch': []}
for mt, mtext in mks2:
    nxt = min([t for t, _ in mks2 if t > mt], default=ti2[-1])
    lo, hi = mt + 4.0, min(mt + 30.0, nxt - 15.0, ti2[-1])
    m = (ti2 >= lo) & (ti2 <= hi) & ok2
    if m.sum() < 30: continue
    if 'roll' in mtext:
        ang = np.radians(im2['epitch_deg'][m] - LEVEL_EPITCH); rate = -im2['gy'][m]
        f = norm_decay(ti2[m], ang, rate)
        if f: f['rep'] = mtext; s2n['roll'].append(f)
    else:
        er = unwrap_deg(im2['eroll_deg'][m])
        er = er - 360.0*np.round((np.nanmedian(er)-LEVEL_EROLL)/360.0)
        ang = np.radians(-(er-LEVEL_EROLL)); rate = -im2['gx'][m]
        f = norm_decay(ti2[m], ang, rate)
        if f: f['rep'] = mtext; s2n['pitch'].append(f)

for axn, L in s2n.items():
    R[f's2_{axn}'] = dict(
        K_all=[round(f['K'],3) for f in L], C_all=[round(f['C'],3) for f in L],
        rms_deg=[round(f['rms'],2) for f in L], peaks=[round(f['peak'],1) for f in L],
        K_median=float(np.median([f['K'] for f in L])) if L else None,
        C_median=float(np.median([f['C'] for f in L])) if L else None)
if IX_EFF and R['s2_roll']['K_median']:
    R['k_roll_abs'] = dict(
        k=float(R['s2_roll']['K_median']*IX_EFF),
        c=float(R['s2_roll']['C_median']*IX_EFF),
        note='k/I from S2 surface decay x Ix_eff from S7 known-torque pulses; '
             'SURFACE waterplane-stiffened -> upper bound for submerged righting')

print(json.dumps(R, indent=1))
json.dump(R, open(os.path.join(OUT, 'fit_results2.json'), 'w'), indent=1)
