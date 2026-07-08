#!/usr/bin/env python3
"""Water session 1 fits: S1 trim, S2 righting/damping, S3 heave, S4 yaw, S7 pulses.
Axis map (LOCKED, RESUME.md):
  heading = eyaw CCW+, heading rate = -gz  (F11)
  nav roll = epitch - level, PORT-DOWN +, rate = -gy  (F10)
  body pitch = -(eroll - level(~-170)), BOW-UP + in that convention, rate = -gx
Conventions here: depth down+, heave cmd on t0/t1 (+ = thrust UP).
"""
import numpy as np, os, sys, json
from scipy.optimize import least_squares
from scipy.signal import savgol_filter
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from explore import load, markers, RUNS
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fits_out')
os.makedirs(OUT, exist_ok=True)
R = {}   # results accumulator

MASS = 9.98
IZ, IY, IX = 0.434, 0.366, 0.280   # box-model priors
LEVEL_EROLL = -170.17              # S1 measured
LEVEL_EPITCH = -18.19

def mask_corrupt_gyro(im):
    c = (im['gx'] == 0) & (im['gy'] == 0) & (im['gz'] == 0)
    return {k: (np.where(c, np.nan, v) if k in ('gx','gy','gz') else v) for k, v in im.items()}

def unwrap_deg(a):
    return np.degrees(np.unwrap(np.radians(a)))

# ================= S1: static trim =================
r = RUNS['s1']
im = load(r, 'imu'); fu = load(r, 'fused')
tend = fu['t'][-1]
use = (im['t'] < tend - 33) & ~((im['eroll_deg']==0)&(im['epitch_deg']==0)&(im['eyaw_deg']==0))
R['s1'] = dict(
    depth_float_reading=float(np.nanmean(fu['depth_m'][fu['t'] < tend-33])),
    level_eroll=float(np.nanmedian(im['eroll_deg'][use])),
    level_epitch=float(np.nanmedian(im['epitch_deg'][use])),
    heading_drift_deg_per_min=float(np.polyfit(im['t'][use]-im['t'][0], unwrap_deg(im['eyaw_deg'][use]), 1)[0]*60),
)

# ================= S2: tilt releases =================
r = RUNS['s2']
im = load(r, 'imu'); im = mask_corrupt_gyro(im)
mks = markers(r)
ti = im['t']
euler_ok = ~((im['eroll_deg']==0)&(im['epitch_deg']==0)&(im['eyaw_deg']==0))

def fit_release(tt, ang, rate, I, label):
    """From release (max |ang|) fit  I*a'' = -k*a - c*a'  (linear 2nd order).
    Returns k (N*m/rad), c (N*m*s/rad), quality info. ang in rad from level."""
    # find release: last time |ang| > 80% of max, then take decay window
    i0 = int(np.nanargmax(np.abs(ang)))
    a0 = ang[i0]
    # decay window: from peak until settled (|ang| < 15% peak) or end
    settled = np.where(np.abs(ang[i0:]) < 0.15*np.abs(a0))[0]
    i1 = i0 + (settled[0] + 10 if len(settled) else len(ang) - i0 - 1)
    i1 = min(i1, len(ang)-1)
    if i1 - i0 < 8: return None
    tw = tt[i0:i1] - tt[i0]; aw = ang[i0:i1]; rw = rate[i0:i1]
    ok = ~np.isnan(aw) & ~np.isnan(rw)
    tw, aw, rw = tw[ok], aw[ok], rw[ok]
    if len(tw) < 8: return None
    def resid(p):
        k, c = p
        # integrate from (a0, r0) with measured-rate initial condition
        a, v = aw[0], rw[0]
        out = []
        tprev = tw[0]
        for j in range(len(tw)):
            dt = tw[j] - tprev; tprev = tw[j]
            for _ in range(max(1, int(dt/0.01))):
                h = dt/max(1, int(dt/0.01))
                acc = (-k*a - c*v)/I
                v += acc*h; a += v*h
            out.append(a - aw[j])
        return np.array(out)
    try:
        fit = least_squares(resid, [0.5, 0.5], bounds=([0.01, 0.0], [50, 50]))
        k, c = fit.x
        rms = float(np.sqrt(np.mean(fit.fun**2)))
        return dict(label=label, k=float(k), c=float(c), peak_deg=float(np.degrees(a0)),
                    rms_deg=float(np.degrees(rms)), n=len(tw), dur=float(tw[-1]-tw[0]))
    except Exception as e:
        return None

s2 = {'roll': [], 'pitch': []}
for mt, mtext in mks:
    nxt = min([t for t, _ in mks if t > mt], default=ti[-1])
    lo, hi = mt + 4.0, min(mt + 32.0, nxt - 15.0, ti[-1])
    m = (ti >= lo) & (ti <= hi) & euler_ok
    if m.sum() < 30: continue
    tt = ti[m]
    if 'roll' in mtext:
        # nav roll channel: epitch - level, rad; rate = -gy
        ang = np.radians(im['epitch_deg'][m] - LEVEL_EPITCH)
        rate = -im['gy'][m]
        f = fit_release(tt, ang, rate, IX, mtext)
        if f: s2['roll'].append(f)
    else:
        # body pitch channel: -(eroll - level) bow-up+; rate = -gx
        er = unwrap_deg(im['eroll_deg'][m])
        # re-center near level (unwrap can land on +190 shelf)
        er = er - 360.0*np.round((np.nanmedian(er) - LEVEL_EROLL)/360.0)
        ang = np.radians(-(er - LEVEL_EROLL))
        rate = -im['gx'][m]
        f = fit_release(tt, ang, rate, IY, mtext)
        if f: s2['pitch'].append(f)

R['s2'] = {ax: dict(
    k_median=float(np.median([f['k'] for f in L])) if L else None,
    c_median=float(np.median([f['c'] for f in L])) if L else None,
    k_all=[round(f['k'],3) for f in L], c_all=[round(f['c'],3) for f in L],
    peaks_deg=[round(f['peak_deg'],1) for f in L],
    rms_deg=[round(f['rms_deg'],2) for f in L]) for ax, L in s2.items()}

# ================= S3: heave =================
r = RUNS['s3']
fu = load(r, 'fused')
mks = [(t, x) for t, x in markers(r) if x.startswith('STEP') or x.startswith('RUN-DONE')]
tf, d = fu['t'], fu['depth_m']
# smooth depth + velocity
ds = savgol_filter(d, 21, 3)
vz = savgol_filter(d, 21, 3, deriv=1, delta=np.median(np.diff(tf)))
# command trace per time from markers: verticals common command (down = negative cmd)
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

# fit window: down20 start -> contact (depth<1.95 & before step5), plus we
# integrate the ODE  (m*(1+ka)) v' = -B(z) + T_dn(cmd) + ... down+ frame:
#   m_eff v' = -(B0 + dBdz*z_neg...) ... define B(z)=B0 - kb*z (net UP force)
#   m_eff v' = T(cmd_dn) - B(z) - cq*v|v|
# T(cmd): per-PAIR effective down-thrust at |cmd| (10,20,30,40) sign=down when cmd<0
t_lo = [mt for mt, tn, c, n in steps if n == 'down20'][0]
t_hi = [mt for mt, tn, c, n in steps if n == 'recover-buoyant'][0]
mfit = (tf >= t_lo) & (tf < t_hi) & (ds < 1.95)
tw, zw, vw = tf[mfit], ds[mfit], vz[mfit]
KA = 1.0   # heave added-mass factor prior (flat body)
M_EFF = MASS*(1+KA)
B0 = 1.0   # N, June trim measurement prior (just-submerged buoyant force, up)

def s3_resid(p):
    T20, T30, T40, cq, kb = p
    Tmap = {10.0: 0.0, 20.0: T20, 30.0: T30, 40.0: T40, 0.0: 0.0, 22.0: 0.0}
    z, v = zw[0], vw[0]
    out_z = []
    tprev = tw[0]
    for j in range(len(tw)):
        dt = tw[j] - tprev; tprev = tw[j]
        n = max(1, int(dt/0.01))
        for _ in range(n):
            h = dt/n
            cmd = cmd_at(tw[j])
            T = Tmap.get(abs(cmd), 0.0)*np.sign(-cmd)  # cmd<0 -> down+ thrust
            B = B0 - kb*max(z, 0.0)
            acc = (T - B - cq*v*abs(v))/M_EFF
            v += acc*h; z += v*h
        out_z.append(z - zw[j])
    return np.array(out_z)

fit3 = least_squares(s3_resid, [1.0, 2.0, 4.0, 60.0, 0.5],
                     bounds=([0, 0, 0, 5, 0], [20, 30, 40, 400, 2.0]))
T20, T30, T40, cq, kb = fit3.x
R['s3'] = dict(T20_pair_N=float(T20), T30_pair_N=float(T30), T40_pair_N=float(T40),
               drag_quad_N_per_ms2=float(cq), dB_dz_N_per_m=float(kb),
               rms_m=float(np.sqrt(np.mean(fit3.fun**2))),
               assumptions=dict(B0_N=B0, added_mass_factor=KA, mass=MASS),
               notes='fit window down20..down40 pre-contact (z<1.95); T = per-PAIR (2 motors)')

# neutral depth implied
zn = B0/kb if kb > 0 else None
R['s3']['neutral_depth_m'] = float(zn) if zn else None

# plot
fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
ax[0].plot(tf - tf[0], d, lw=0.7, label='fused depth')
for mt, tn, c, name in steps:
    ax[0].axvline(mt - tf[0], color='k', alpha=0.15)
    ax[0].text(mt - tf[0], 2.15, name, rotation=90, fontsize=7, va='top')
# simulate best fit over window
z, v = zw[0], vw[0]; sim_z = []
tprev = tw[0]
Tmap = {10.0: 0.0, 20.0: T20, 30.0: T30, 40.0: T40}
for j in range(len(tw)):
    dt = tw[j] - tprev; tprev = tw[j]
    n = max(1, int(dt/0.01))
    for _ in range(n):
        h = dt/n
        cmd = cmd_at(tw[j])
        T = Tmap.get(abs(cmd), 0.0)*np.sign(-cmd)
        acc = (T - (B0 - kb*max(z,0)) - cq*v*abs(v))/M_EFF
        v += acc*h; z += v*h
    sim_z.append(z)
ax[0].plot(tw - tf[0], sim_z, 'r--', lw=1.2, label='ODE fit')
ax[0].axhline(2.03, color='brown', ls=':', label='bottom contact ~2.03')
ax[0].set_ylabel('depth m (down+)'); ax[0].invert_yaxis(); ax[0].legend(fontsize=8)
ax[1].plot(tf - tf[0], vz, lw=0.7); ax[1].set_ylabel('vz m/s'); ax[1].set_xlabel('t s')
fig.suptitle('S3 heave staircase — fit T20/T30/T40(pair), quad drag, dB/dz')
fig.savefig(os.path.join(OUT, 's3_heave_fit.png'), dpi=110, bbox_inches='tight')
plt.close(fig)

# ================= S4: yaw =================
r = RUNS['s4']
im = load(r, 'imu'); im = mask_corrupt_gyro(im)
mks4 = [(t, x) for t, x in markers(r) if x.startswith('STEP')]
ti = im['t']; wz = -im['gz']   # heading rate rad/s, CCW+
ss = {}
for i, (mt, mtext) in enumerate(mks4):
    tn = mks4[i+1][0] if i+1 < len(mks4) else ti[-1]
    name = mtext.split()[2]
    m = (ti >= mt + 2.5) & (ti < tn)
    if m.sum() > 10:
        ss[name] = float(np.nanmedian(wz[m]))
# steady map cmd -> omega
lvl = dict(ccw10=10, ccw20=20, ccw30=30, ccw40=40, cw10=-10, cw20=-20, cw30=-30, cw40=-40)
pairs = [(lvl[k], v) for k, v in ss.items() if k in lvl]

# spin-down decay (step 5 + tail): tau=0, I*w' = -c2*w|w| - c1*w
def decay_fit(t0, t1):
    m = (ti >= t0) & (ti < t1)
    tt, ww = ti[m], wz[m]
    ok = ~np.isnan(ww); tt, ww = tt[ok], ww[ok]
    ws = savgol_filter(ww, 11, 2)
    dw = np.gradient(ws, tt)
    A = np.vstack([-ws*np.abs(ws), -ws]).T
    sol, *_ = np.linalg.lstsq(A, dw, rcond=None)
    return sol  # [c2/I, c1/I]

sd = [(mt, mks4[i+1][0]) for i, (mt, mtext) in enumerate(mks4[:-1]) if 'spin-down' in mtext][0]
c2_I, c1_I = decay_fit(sd[0]+0.2, sd[1])
c2 = c2_I*IZ; c1 = c1_I*IZ
# steady torque at each level: tau = c2*w|w| + c1*w
tau = {c: float(c2*w*abs(w) + c1*w) for c, w in pairs}
R['s4'] = dict(steady_rate_degps={c: round(np.degrees(w),1) for c, w in pairs},
               c2_over_I=float(c2_I), c1_over_I=float(c1_I),
               yaw_drag_c2=float(c2), yaw_drag_c1=float(c1), Iz_prior=IZ,
               tau_at_cmd_Nm={c: round(t,4) for c, t in tau.items()},
               note='tau from steady balance with decay-fitted drag; scales with Iz prior')

fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
ax[0].plot(ti - ti[0], np.degrees(wz), lw=0.6)
for mt, mtext in mks4:
    ax[0].axvline(mt - ti[0], color='k', alpha=0.12)
ax[0].set_xlabel('t s'); ax[0].set_ylabel('heading rate deg/s (CCW+)'); ax[0].set_title('S4 yaw staircase')
cs = sorted(tau)
ax[1].plot(cs, [tau[c] for c in cs], 'o-')
ax[1].set_xlabel('corner cmd (all four)'); ax[1].set_ylabel('steady yaw torque N·m (Iz prior)')
ax[1].grid(alpha=0.3); ax[1].set_title('torque vs cmd')
fig.savefig(os.path.join(OUT, 's4_yaw_fit.png'), dpi=110, bbox_inches='tight')
plt.close(fig)

# ================= S7: pulses =================
r = RUNS['s7']
im = load(r, 'imu'); im = mask_corrupt_gyro(im)
fu = load(r, 'fused')
mks7 = [(t, x) for t, x in markers(r) if x.startswith('STEP')]
ti = im['t']
wz = -im['gz']; wroll = -im['gy']; wpitch = -im['gx']
ds7 = savgol_filter(fu['depth_m'], 21, 3)
vz7 = savgol_filter(fu['depth_m'], 21, 3, deriv=1, delta=np.median(np.diff(fu['t'])))
pulses = []
import re
for i, (mt, mtext) in enumerate(mks7):
    parts = mtext.split()
    name = parts[2]
    mm = re.match(r't(\d)([+-]\d+)', name)
    if not mm: continue
    thr, amp = int(mm.group(1)), int(mm.group(2))
    tn = mks7[i+1][0] if i+1 < len(mks7) else ti[-1]
    win = (ti >= mt) & (ti < tn)
    pre = (ti >= mt-1.0) & (ti < mt)
    depth_here = float(np.interp(mt, fu['t'], ds7))
    on_bottom = depth_here > 1.85
    # signed peak rates (max |x| with sign)
    def speak(v, m):
        vv = v[m]; vv = vv[~np.isnan(vv)]
        if not len(vv): return np.nan
        return float(vv[np.argmax(np.abs(vv))])
    # impulse: integrate rate change over pulse (rate_end - rate_start)*I ~ tau*dt
    # simpler: signed peak minus pre-mean
    p = dict(name=name, thr=thr, amp=amp, depth=depth_here, bottom=bool(on_bottom),
             yaw_pk=speak(wz, win) - float(np.nanmean(wz[pre])) if pre.sum() else speak(wz, win),
             roll_pk=speak(wroll, win), pitch_pk=speak(wpitch, win))
    # vertical thrusters: vz response (m/s) during pulse vs pre
    if thr in (0, 1):
        mfu = (fu['t'] >= mt+0.3) & (fu['t'] < tn)
        mfu_pre = (fu['t'] >= mt-1.0) & (fu['t'] < mt)
        if mfu.sum() and mfu_pre.sum():
            p['dvz'] = float(np.nanmean(vz7[mfu]) - np.nanmean(vz7[mfu_pre]))
    pulses.append(p)
R['s7'] = pulses

# corner yaw signs table
print(json.dumps(R, indent=1, default=str))
with open(os.path.join(OUT, 'fit_results.json'), 'w') as f:
    json.dump(R, f, indent=1, default=str)

# S7 plot: yaw peak per corner pulse
fig, ax = plt.subplots(figsize=(11, 4))
names = [p['name'] for p in pulses if p['thr'] >= 2]
vals = [p['yaw_pk'] for p in pulses if p['thr'] >= 2]
cols = ['tab:red' if p['bottom'] else 'tab:blue' for p in pulses if p['thr'] >= 2]
ax.bar(range(len(names)), np.degrees(vals), color=cols)
ax.set_xticks(range(len(names))); ax.set_xticklabels(names, rotation=60, fontsize=8)
ax.set_ylabel('peak yaw-rate delta deg/s (CCW+)')
ax.set_title('S7 corner pulses (red = ON BOTTOM, friction-attenuated)')
ax.grid(alpha=0.3, axis='y')
fig.savefig(os.path.join(OUT, 's7_corner_pulses.png'), dpi=110, bbox_inches='tight')
plt.close(fig)

# S2 plot: releases overlay
fig, axs = plt.subplots(2, 1, figsize=(11, 7))
for mt, mtext in mks:
    nxt = min([t for t, _ in mks if t > mt], default=im['t'][-1])
    lo, hi = mt + 4.0, min(mt + 32.0, nxt - 15.0)
    m = (load(RUNS['s2'],'imu')['t'] >= lo) & (load(RUNS['s2'],'imu')['t'] <= hi)
im2 = load(RUNS['s2'], 'imu')
ok2 = ~((im2['eroll_deg']==0)&(im2['epitch_deg']==0)&(im2['eyaw_deg']==0))
for mt, mtext in mks:
    nxt = min([t for t, _ in mks if t > mt], default=im2['t'][-1])
    lo, hi = mt + 4.0, min(mt + 30.0, nxt - 15.0)
    m = (im2['t'] >= lo) & (im2['t'] <= hi) & ok2
    if m.sum() < 20: continue
    if 'roll' in mtext:
        axs[0].plot(im2['t'][m]-lo, im2['epitch_deg'][m]-LEVEL_EPITCH, lw=0.8, label=mtext)
    else:
        er = unwrap_deg(im2['eroll_deg'][m])
        er = er - 360.0*np.round((np.nanmedian(er)-LEVEL_EROLL)/360.0)
        axs[1].plot(im2['t'][m]-lo, -(er-LEVEL_EROLL), lw=0.8, label=mtext)
axs[0].set_ylabel('nav roll deg (port-down+)'); axs[0].legend(fontsize=7); axs[0].grid(alpha=0.3)
axs[1].set_ylabel('body pitch deg (bow-up+)'); axs[1].set_xlabel('t since window start s')
axs[1].legend(fontsize=7); axs[1].grid(alpha=0.3)
fig.suptitle('S2 tilt releases (AT SURFACE — waterplane-stiffened)')
fig.savefig(os.path.join(OUT, 's2_releases.png'), dpi=110, bbox_inches='tight')
plt.close(fig)
print('\nplots + fit_results.json in', OUT)
