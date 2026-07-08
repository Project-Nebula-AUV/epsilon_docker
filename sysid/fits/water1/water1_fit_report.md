# Water Session 1 — fit report (2026-07-07)

Runs: `20260707-023942-s1_static_trim-hand`, `20260707-025309-s2_tilt_release-hand`,
`20260707-065749-s3_heave_staircase-live` (batt 15.8 V),
`20260707-071456-s4_yaw_staircase_trim-live` (15.40 V),
`20260707-072200-s7_single_thruster_trim-live` (15.40 V).
Analysis scripts + raw fit JSONs alongside this file; plots: `s3_heave_fit.png`,
`s4_yaw_fit.png`, `s7_corner_pulses.png`, `s2_releases.png`.
User context: S3 and S7 touched bottom (pool too small); S4 finger-steadied just
below the surface (minimal force, yaw-only relevant); S2 reps floated to the top.

## Headline numbers (now live in sim_calibration.yaml)

| Quantity | Value | Source | Confidence |
|---|---|---|---|
| Vertical thrust / motor | 0.77 / 1.46 / 2.29 N at 20/30/40% | S3 ODE fit, rms 4.7 mm | HIGH in-band |
| Thrust curve | T ≈ 0.0015·cmd² N (deadband <10%) | S3 | curve HIGH, T(100)≈15 N is a LONG extrapolation |
| Heave quad drag | 84 N/(m/s)² (80–89 over kb sweep) | S3 | HIGH |
| Buoyancy | +1 N at surface → ~0 at 1.9 m (slope ≈0.53 N/m) | S3 free-sink/rise + F13 | MED (B0 prior from June) |
| Yaw torque | τ ≈ 8.4e-4·cmd² N·m; τ(40%)=1.28 N·m | S4 steady + decay | HIGH (scales with Iz prior) |
| Yaw quad drag | c2 = 0.352·(Iz/0.434) N·m/(rad/s)²; +c1 0.083 linear | S4 spin-down | HIGH ratio, abs tied to Iz |
| Steady yaw rates | ±18/50/79/103 °/s at 10/20/30/40% | S4 | MEASURED, CCW/CW symmetric ≤3% |
| Roll eff. inertia Ix_eff | 0.58 kg·m² (0.51–0.96) | S7 known-torque pulses | MED-HIGH |
| Submerged righting k (roll = pitch) | ≈4.6 N·m/rad (K 5.6–12.8 s⁻², med ~8) ⇒ BG ≈ 4.7 cm | S7 post-pulse decays | MED (small angles 2–5°) |
| Roll damping | c/Ix ≈ 1.13 s⁻¹ ⇒ ζ≈0.19 at ωn≈3 rad/s | S7 decays | MED |
| Hover trim | −22% cmd ≈ 1.45 N pair ≈ B + margin ✓ | S3/S4/S7 rests | consistent with June −22.8 |

## Findings (new ledger entries)

- **F13 DECIDED (user, 2026-07-07): accept + retrieval plan** — pool-floor
  failures are OK, a retrieval plan exists. No foam/purge for now; revisit
  before comp (at comp depth a disarmed sub sinks).
- **F13 (HIGH, SAFETY): fail-safe-to-surface FAILS below ~1.5–2 m.** After S3's
  auto-zero the sub sank from 1.88 m back to the floor and stayed; same in S7.
  Buoyancy decreases with depth (~0.5 N/m ⇒ ~0.5–0.7 L compressible volume —
  trapped air in frame tubes / enclosure flex). At comp depth (2.2–5 m) a
  disarmed sub SINKS. Options: add fixed foam (raises surface trim too),
  find/purge the trapped air, or accept + plan retrieval. Decide before comp.
- **F14: ESP32 depth temp transient ≈0.2–0.25 m on air↔water transitions**,
  settles over ~20–30 s (post-run in-air readings drifted −0.03→−0.24 m).
  Depth SCALE verified sane (driver 100/(997·9.80665); ambient plausible).
  Mitigation: float the sub in water BEFORE launching the stack (baseline
  captures wet at water temp); consider logging the firmware temperature
  field in depth_raw.csv (one-line logger change).
- **F15: pool floor at ~2.0–2.1 m sensor depth** — pool venue's 1.52 m
  assumption is wrong for this pool (user confirmed "deeper than 1.5 m";
  run meta water_depth values were defaults, not measurements). Measure the
  real depth for the venue config when convenient.
- **4b CLOSED: ESP32 chain in-water = excellent.** 7.14 Hz steady, max gap
  0.15 s, zero >0.5 s gaps, raw-vs-fused p50 4–6 mm, p99 39–55 mm, zero
  >10 cm outliers across S3+S7 live windows. Boot artifact: none this time
  (first samples clean); filter never stressed.

## Bottom-contact handling
S3: contact from late down40 (~z≥2.0); recover-buoyant AND up10 did NOT lift
off the bottom (suction/friction + F13 near-neutral buoyancy); up20 broke free.
Fit window = down20→down40 pre-contact only (z<1.95).
S7: sub settled on the floor during the t0/t1 vertical pulses (rest trim −22
lets it sink between pulses); ALL t2–t5 corner pulses happened ON the floor —
signs + rough relative magnitudes only (peak yaw-rate deltas 18–35 °/s at 40%,
friction-attenuated ~2.5–4× vs free-water expectation). All corner yaw SIGNS
match allocation.yaml (+cmd ⇒ CCW, all four). Vertical roll signs match the
[+1,−1] differential (t0=LV +cmd ⇒ starboard-down = nav-roll −, t1 mirror).
Vertical up/down asymmetry: −40 (down) roll response ~1.5–1.8× the +40 (up)
response on t0 — prop fwd/rev asymmetry, worth a note for fine depth control.

## S2 tilt releases — qualitative only (surface physics)
Surface releases are waterplane + trapped-water dominated: righting K scattered
×17 rep-to-rep, equilibrium offsets −11…−13° (the sub re-floats listing after
each dunk — water drains/refills the frame). NOT usable for submerged righting;
superseded by the S7 post-pulse decays. Keep S2-style releases OFF the future
protocol; replace with at-depth pulse-and-release (S2b below).

## Fit-method notes
- S3: ODE least-squares m_eff·v̇ = T(cmd) − B(z) − c·v|v|, m_eff = 9.98·(1+1.0
  added mass), B0 = 1.0 N fixed (June trim), dB/dz fixed 0.53 from the
  free-sink; kb sweep 0.40/0.53/0.70 moved T's <5% (insensitive). The descent
  window alone is degenerate in (T, dB/dz) — free-sink pins it.
- S4: heading rate = −gz (F11 sign); steady medians per step; spin-down
  window fits ω̇ = −(c2/Iz)ω|ω| − (c1/Iz)ω by least squares.
- S7 Ix_eff: initial ω̇ from a 0.6 s window at pulse start, τ = T(cmd)·0.184
  with T from the S3 curve. ±20 pulses too noisy; ±40 used.
- Righting: post-pulse decay windows (1.3–2.3 s, 18 Hz) fit to
  θ̈ = −K(θ−θ₀) − Cθ̇; usable reps: excursion >3°, rms <0.25°.

## Linear-sim caveats (until epsilon-plant mode)
thrusterMaxForce 5.3 N matches the real quadratic curve only in the 30–40%
band: sim is ~2× too strong at ≤20% cmd (small corrections too authoritative;
gains tuned in sim will feel weak on hardware at small errors) and ~2.8× too
weak at 100% (style roll, escape maneuvers). Yaw same shape (τ∝cmd² real).
Epsilon-plant mode (bridge inverse-mix + these quadratic curves as the sim's
actuation path) is the queued fix and should now be built with these numbers.

## Water session 2 additions (proposed; PROTOCOL.md not edited — vim swap present)
1. Measure the actual pool depth (F15) — tape or the sub's own sensor lowered
   on a line (settle 60 s at the bottom for the temp transient).
2. Launch procedure change: float the sub in the water ≥60 s before launch
   (F14 — wet baseline; fits the 90 s ARM_DELAY workflow).
3. S2b at-depth righting: mid-water, pulse one vertical ±40–60% for 2–3 s,
   then rest ≥5 s (longer decay window than S7's 1.5 s rests) ×6 reps.
   Confirms k=4.6 at bigger angles; do at ~1 m depth.
4. S8 deadband ramps MID-WATER (needs the trim variant; keep off the floor).
5. S9 roll authority: with k≈4.6 expect direct-roll stall ~50° at 80%; try
   the 0.45–0.5 Hz pump if W6 lands it first.
6. S5/S6 surge/sway: also disambiguates the yaw-arm vs corner-thrust question
   (effective arm 0.151 ⇒ either ~55–63° toed thrust lines or weaker corner
   props than verticals).
