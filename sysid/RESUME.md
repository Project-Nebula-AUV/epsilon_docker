# SYSID RESUME — canonical state. Update BEFORE ending every session.

2026-07-13 (post-session) — **STALE INSTALL RECURRED + ROOT-CAUSED: colcon
defaults were never actually wired.** A plain `colcon build` (~20:15 UTC)
re-copied robosub+epsilon_bridge into site-packages, silently freezing the
runtime again (params gate would have refused to arm at the next session).
Root cause: ws colcon_defaults.yaml was INERT — colcon only reads
~/.colcon/defaults.yaml or $COLCON_DEFAULTS_FILE, neither existed, so every
build depended on remembering flags. FIXED durably: (1) symlink-install +
merge-install added to colcon_defaults.yaml (.pre-symlink.20260713);
(2) wired via ~/.colcon/defaults.yaml symlink + COLCON_DEFAULTS_FILE export
in container ~/.bashrc — a bare `colcon build` now does the right thing
(verified: flag-less rebuild of all 4 python pkgs). Stale copies purged;
this colcon develop-mode install creates NO site-packages dirs (nothing to
shadow). VERIFIED end-to-end: NO_ARM bench holdtest 20260713-210255 —
params gate PASS (imports -> /src/), "control params loaded", mission
STARTED. Loaded yaml = build/robosub/config/pid_params.yaml -> symlink to
src (poolside edit + node restart workflow intact, yaw_rate_p 0.7 live).
NOTE: container rebuild would lose the ~/.colcon + .bashrc wiring — re-run
the two wiring lines (or check the params gate) after any container recreate.


2026-07-13 (latest) — **YAW LIMIT CYCLE DEAD (yaw_rate_p 0.7 verified in water)
+ FIRST CLOSED-LOOP STRAIGHT SHOTS ANALYZED.** Runs: 075059 hold, 080244
straight@0.3, 080624 straight@0.8-depth1.0 (floor-grind, aborted), 081040
(power loss at launch, no data), 081327 straight@0.8 (the good one).
- HOLD: heading std 0.6° (was 2.9), gyro_z p50 0.01 rad/s (was 0.60), yaw
  duty 0% (was 90%), roll std 0.6°, depth 0.81±0.03. The "slight up drift" =
  depth integrator bleeding the initial-dive overshoot (0.93→0.81 over 50 s,
  converged) — expected, not a defect. 75 s MISSION_COMPLETE.
- STRAIGHT@0.8 (081327): all four corners DO carry surge (RL −88 FL −91
  FR +66 RR +65 PWM; 2% saturation; sway cmd 0.000; yaw quiet). Left pair
  runs REVERSE by mount geometry. Two measured defects:
  (1) HEADING DROOP: sub parks ~+7° right-of-line during thrust — steady
      asymmetry torque needs yaw cmd −0.14; P-only cascade err_ss =
      0.14/(1.6·0.7) = 7.2° ✓ exact. Gyro-integration confirms PHYSICAL
      rotation (not compass distortion).
  (2) LEFT DRIFT: residual sway force from per-motor fwd/rev thrust
      inequality (uncommanded, unsensed). Both fix at the root with a
      PER-MOTOR FWD/REV COMPENSATION table in thruster_bridge from a bench
      bollard test (log battery_v!). Stopgap: yaw feedforward −0.175·surge.
- SPEED: 0.3 surge = corner PWM 24–35 ≈ prop deadband → ~1 m/20 s (useless;
  quadratic props ⇒ surge floor ~0.5). 0.8 surge ≈ est 20 N forward model
  but observed only ~3 m/20 s (~0.2 m/s) — thrust model (fit on the VERTICAL
  pair, water-1) is optimistic for corners; reverse-pair efficiency + likely
  battery sag are prime suspects. Bollard test resolves quantitatively.
- STRAIGHT2 LESSON (target 1.0 in the shallow end): depth below the physical
  floor ⇒ verticals pinned −1.00 grinding the floor for 38 s, settle gate
  never passes, mission stuck until human abort. TODO: depth-unreachable
  valve in StabilizeTask (vert saturated + no progress ≥10 s → hold current
  depth + flag). Known-depth scale ritual STILL OWED (under-read makes
  floor-relative targets dangerous).
- 081040 "crash": abrupt power loss during stack launch (supervisor.log
  NUL-filled, CSVs empty). Cause unconfirmed (battery sag under launch
  spike suspected). straight4 ran fine right after. Start logging battery_v
  in meta (EDIT ME field exists).


2026-07-13 (later) — **FIRST LIVE-PARAMS WATER RUNS: ROLL SOLVED. Yaw limit
cycle = the last defect; yaw_rate_p 2.5→0.7 applied (.pre-yawcalm.20260713).**
Runs 20260713-071305 (shoves) + 20260713-071652 (no shoves; power cut 3 s
into the FINAL SURFACE PHASE — status.stream lost to buffering, CSVs fine):
- ROLL: deadzone + linear 0.30/0.20 WORK. Settled std 1.3–2.1°, vert
  differential duty 0–1.8% (0.0% inside |roll|<30°). Shoves to 34.5° and
  78.8° arrested and back <6° in 2.3/3.4 s, no re-ring. Limit cycle GONE.
- DEPTH: 0.79–0.81 indicated (±0.01–0.07) vs 0.8 target — ff stall gone;
  held 0.77–0.84 THROUGH the turn (turn-depth fix verified). Run A
  MISSION_COMPLETE in 84 s (was 175 s) — settle gates latch on schedule.
- BUT the user saw the hull near the bottom of the 4.5 ft (1.37 m) section
  at 0.8 indicated → the sensor under-read is REAL (~×1.2–1.4). Next
  session: KNOWN-DEPTH ritual — static hold at 0.8 indicated, tape from
  waterline to the SENSOR PORT, scale = true/indicated; set depth_scale to
  the measured value. NOTE scale k also multiplies depth-loop gain by k —
  recheck hold quality after applying.
- YAW: ~2.5 Hz limit cycle measured (gyro_z p50 0.6 rad/s × yaw_rate_p 2.5
  → cmd pinned ±1, 87–90% duty, 5.3 sign flips/s, all 4 corners). It caused
  ALL THREE user observations: visible yaw rocking; heavy SIDEWAYS DRIFT
  (rectified fwd/rev-asymmetric thrust of full-scale flipping corners —
  commanded sway/surge were exactly 0.000); turn settle scatter (+3° run A,
  −8.7° run B — the 2.5° settle gate latches on limit-cycle noise, then the
  next StabilizeTask captures the wobbled heading). Fix mirrors the roll
  cure (leave saturation): yaw_rate_p 0.7, verified loaded.
- NEXT RUN: same holdtest @0.8. Expect: no visible yaw rock (corner duty
  <10%), drift mostly gone, turn parks ±3°, ~84 s total. If dither
  persists: LPF gyro_z or small rate-error deadband — NOT more gain moves.


2026-07-13 — **STALE-INSTALL INCIDENT FOUND + FIXED: NOTHING TUNED SINCE
2026-07-09 EVER RAN IN THE WATER.** Tonight's hold+90 run (20260713-061539)
exposed it: launch.log "pid_params.yaml not found, using built-in defaults".
- Root cause: the 2026-07-09 ~19:30 UTC colcon build was a plain COPY build
  (not --symlink-install). Since then the runtime was frozen: pid_params.yaml
  NEVER loaded (built-in defaults ran: roll_p 0.6 / roll_d 3.0 / buoyancy_ff
  0.61), control.py deadzone/pulse NEVER installed, sensor_bridge depth_scale
  NEVER installed. ALL 2026-07-12 roll-tuning water conclusions are VOID —
  the four "configs" were the same frozen controller. The roll deadzone has
  never actually been water-tested.
- Tonight's data confirms: roll ±25-40° @ ~2.6 s the whole run = the old
  saturated bang-bang limit cycle (vert differential pinned ±0.4 cap 82%
  duty; 85% even at |roll|<20° → no deadzone in the loop); yaw rate loop
  saturated by rock-coupled gyro_z (p50 0.54 rad/s; 62% of samples) → all 4
  horizontal motors slamming ±100% at the rock period = the heavy drift +
  battery burn; depth held 0.94 m raw ±0.008 vs 0.8 target = the known
  ff=0.61 integrator-clamp stall.
- Near-surfacing DURING the 90° turn = mission bug, not yaw: task_base.py
  injected context target_depth **0.1 m** for bare Task([...]) wrappers, so
  the turn phase held 0.1 m BY COMMAND. FIXED (.pre-turndepth.20260713):
  bare wrappers no longer inject a depth; subtasks hold current depth.
- REBUILT robosub+epsilon_bridge with --symlink-install (stale site-packages
  copies deleted first — a copied package SHADOWS the symlink install; as-ran
  code preserved: sysid/installed-as-ran-20260713.tgz). VERIFIED: imports
  resolve to /src/, yaml loads (INFO line in launch.log), live gains 0.30 /
  0.20 / deadband 35 / ff 0.12 / depth 3.0/0.15/2.0. Bench NO_ARM holdtest
  PASSED (run 20260713-070149).
- NEW PARAMS GATE in watertest_supervisor.sh (.pre-paramsgate.20260713):
  refuses to ARM unless imports resolve to /src/ AND the yaml resolves AND
  launch.log has no "using built-in defaults". Trips loudly with fix text.
- depth_scale 1.4 RETIRED UNUSED → 1.0 (.pre-scalerevert.20260713): the
  eyeball evidence is internally inconsistent (4 ft pool with "true depth
  1.52 m"), above-water estimates suffer refraction + sensor-vs-hull offset,
  and every valid calibration (water-1 fits, sim plant, depth gains) is on
  RAW depth. Known-depth ground-truth ritual still owed before any scale.
- NEXT WATER RUN: same holdtest = the FIRST live test of deadzone + linear
  low-gain roll (0.30/0.20) + W6 depth gains. If rocking persists while the
  roll differential stays quiet, the remaining pump is the YAW loop
  (gyro_z-coupled) — then cap/LPF yaw as the next single change.


2026-07-09 (later) — **PHASE 0 BENCH PREP FOR THE COURSE ARC DONE** (plan:
2 water sessions to full-course auton; laptop plan file curried-wibbling-
toucan). READ **sysid/SESSION1_RUNBOOK.md** for the next water session
(S10 reshoot + holdtest/rolltest/gatetest checkout, in that order).
- One-command water tests: .devcontainer/holdtest.sh / rolltest.sh /
  gatetest.sh (watertest.sh core: motortest bring-up + sysid logger every
  run + NO_ARM=1 bench mode). Missions holdtest/rolltest in mission.py.
- Depth hold re-verified in sim under water-2 cal: 1.501 m +-2.1 cm steady
  (no PID retune needed). rolltest rolls AT DEPTH (StyleRollSubtask grew an
  explicit target_depth — it was rolling at the surface when the pre-dive
  had not finished; fixed + a leaked-global-replace crash fixed, see git).
- SCALED COURSE rehearsal works: ROBOSUB_GATE_X/SLALOM_START/SLALOM_STEP
  (sim world), ROBOSUB_WORLD_DEPTH, ROBOSUB_GATE_CLEAR_S (post-gate blind
  clearance = the one true course-scale distance in the tasks). Backyard
  scale (4 m gate, +2.5 m slalom, 2 m step, 2.1 m deep) MISSION_COMPLETE
  138.5 s; full-scale regression 161.3 s w/ 720 roll.
- surge/sway drag now flat-plate ESTIMATES 25/55 (nominal 1.5/8 let the sim
  coast >5 m = broke rehearsal; S5/S6 unfittable) — refine from course video.

2026-07-09 — **WATER SESSION 2 ANALYZED + CALIBRATION LANDED (backups
.pre-w2cal.20260709; run metas annotated .pre-w2meta*).** Runs: S5 surge,
S6 sway (ESP32 depth DEAD whole run — header-only CSVs; est ~0.9 m), S8 trim
-0.076 sink, S9 roll authority. Pool ~9 ft per user (S8 sensor flatlined
+2.11 m at floor contact — discrepancy recorded, fits unaffected).
- **S8 (bin fit, rms 0.08 N): (W-B)(z) = -0.21 + 0.41*z N → the sub is
  NEUTRAL to ~4 g at the surface** (that is why trim tuning could not
  converge) and buoyancyDepthSlope refines 0.53 → 0.41. No fixed trim can
  hold depth (unstable gradient) — closed-loop depth is the durable fix.
- **S9 HEADLINE: the +100% roll pulse did a REAL 360 BARREL ROLL in the
  pool** (gyro-integrated 390 deg; -100% reached ~164, stalled just past
  inverted). The attitude filter attenuates fast rolls ~35% (max 88 deg
  reported) — use -gy (imu.csv, rad/s) for fast roll, NOT attitude roll.
  Rate fit (6 episodes, known tau): Ix 0.42, k 3.3, drag c1 0.18 + c2q 0.42
  (shallow valley; wn 2.8 matches 2.2-2.3 s period). NO brownout at 100%.
- **Landed in sim** — sim_calibration.yaml (slope 0.41, subVolume 0.0099834
  = +0.04 N surface, Ix 0.42, Iy 0.53, righting 3.3/3.3, roll drag 0.42
  quad + 0.18 linear via NEW angularDragLinearCoeff_X/Y in config.py +
  simulator.py) and StyleRoll roll_power 0.80 → 1.00 (F12 cleared by S9).
- **VERIFIED: full course WITH style roll MISSION_COMPLETE 150.2 s** (was
  194-237 s with timeout valves) — roll completes +351 deg in ~4-6 s, drift
  <0.7 m, all 6 gatelets THROUGH both ways. Log /tmp/w2cal_full_roll.log
  (container /tmp — copy to keep).
- S5/S6 translational drag: UNFITTABLE from IMU (pitch-gravity leak ~ signal;
  sub also yawed 147-174 deg during runs). NEXT WATER SESSION: stopwatch a
  ~3 m straight transit at fixed cmd (one number → v_term → drag). Soft sway
  numbers (m_eff~44, c2~450+, at grid edge) NOT landed in sim.
- S2b never run in water (S8 sink substituted for the slope). S10 vision
  IGNORED per user (dusk/murky; redo in the brighter pool later).
- Analysis state + scripts: ~/.claude/plans/water2-fits/ on the laptop.

2026-07-07 PM — **W6 SESSION (Session D): epsilon-plant + depth re-tune +
roll pump + vision/centering overhaul + camera match — ALL DEPLOYED
(backups `.pre-20260707-w6`), see "## W6 session" below.** F13 DECIDED by
user: pool-floor failures ACCEPTED + retrieval plan (revisit before comp).
UPDATE 2026-07-08 AM (W7, this morning): BOTH open items FIXED+VERIFIED —
(1) return-gate acquisition: _is_slalom_red bands tightened to true gatelet
geometry (ratio 0.7-1.45, sep 0.8-2.4x mean h, bottoms 0.35 — the old loose
bands excluded EVERY red on the return leg; backup .pre-slalomred.20260708;
re-probe: posts survive, pair=Y) → STYLE_ROLL=0 full course **CLEAN
COMPLETION 145.2 s, zero valves**. (2) W7 pacing: Center valve 40→20 s +
ADAPTIVE roll segments (a segment that times out short sets
context['style_roll_skip']; later segments complete instantly; backup
.pre-w7pace.20260708) → full course WITH roll **MISSION_COMPLETE 194.4 s**
(valves remaining: 1 honest pump timeout + 2 post-roll re-centers while
still swinging; pump best swing +101°). REPEATABILITY CONFIRMED 2026-07-08 PM:
with-roll full course 4/4 COMPLETE — 194.4/204.5/186.8 s HW-faithful +
236.7 s ideal (valves = honest pump timeouts + post-roll re-centers only);
no-roll CLEAN 145.2 s. Logs archived: sysid/fits/water1/w6_runs/.
s2b_depth_release DRY PASS (3368 ticks, dev 0.000) — its logger run ALSO
verified temp.csv end-to-end (449 rows ~7 Hz, F14 check done);
s9b_roll_pump DRY PASS (3067 ticks, dev 0.000). NOTE: sysid_run.sh must be
invoked INSIDE the container (docker exec, cd /home/robosub/robosub_ws) —
the host has no robosub_ws path (failed silently-ish from the host 2026-07-08).
**WATER SESSION 2 IS FULLY UNBLOCKED — user checklist sysid/WATER2_ADDENDUM.md.**
[superseded 2026-07-07 text follows:] FULL COURSE NOT YET CLEAN — two open items: (1) return-gate AcquireTarget
fails looking back through the slalom field (probe staged:
sysid/fits/water1/return_probe.py), (2) style-roll safety valves (~300 s)
overrun the 600 s wall clock → W7 re-pacing. Verification logs archived in
sysid/fits/water1/w6_runs/. Water-2 user checklist: sysid/WATER2_ADDENDUM.md.

STATUS: SESSION A COMPLETE (2026-07-05). W0 audited+hardened, W1 built+DRY-verified
(all 7 sequences PASS), PROTOCOL.md v1 live, fit-side starter scripts in sysid/fit/.
2026-07-06: **DEPTH SENSOR WORKS — new ESP32 architecture, integrated + verified**
(see "Depth architecture" below; fusion out of the active path).
2026-07-06 PM: **W5 HONEST SIM BUILT + VERIFIED** (see "W5 honest sim" below):
calibration mechanism, passive pitch DOF, measured sensor models, esp32 depth
emulation, pool|comp venues — full course clean under all of it; style roll now
honestly stalls (S2/S9 + W6 will resolve).
2026-07-07: **WATER SESSION 1 COMPLETE + FITS LANDED** (see "Water session 1
fits" below + sysid/fits/water1/water1_fit_report.md): S1/S2 hand, S3/S4/S7
live all logged; S3+S7 touched bottom (pool small — handled in fits), S4
finger-steadied near surface (clean anyway). First real dynamics now live in
sim_calibration.yaml (16 params overlay-verified); simulator grew
rollMomentArm + buoyancyDepthSlope (backups .pre-water1cal.20260707-030925).
THREE NEW FINDINGS: **F13 fail-safe-to-surface FAILS below ~1.5-2 m
(compressible buoyancy — SAFETY, decide foam/purge before comp)**, F14 ESP32
depth ~0.2 m temp transient on air↔water (launch with sub already floating),
F15 pool actually ~2.0-2.1+ m deep (1.52 venue value wrong; user confirmed
deeper than 1.5, exact depth unmeasured). 4b CLOSED (in-water chain excellent).
NEXT (model): W6 = style-roll RESONANCE PUMP (~0.45-0.5 Hz; direct 90° not
reachable at safe power — numbers in the fit report) + epsilon-plant mode
(quadratic thrust curves T=0.0015·cmd² — the linear-sim band-match at 30-40%
is the biggest remaining sim lie) + nav re-tune under the calibrated sim.
NEXT (user): water session 2 (S5,S6,S8,S9,S10 + S2b at-depth releases +
measure pool depth); see "Water session 2 additions" in the fit report.

## Depth architecture (2026-07-06 — the Pi 5 was the problem all along)
The MS5837 never worked on the Pi 5's I2C buses (0 valid reads across ~16k attempts,
both buses, every pacing, 2026-07-05). It now hangs off a **Seeed Xiao ESP32-C3**
(sensor on the Xiao's I2C; Xiao → Pi over USB-C, CDC serial `/dev/ttyACM0`,
Espressif 303a:1001). Firmware streams ~7 Hz JSON lines:
`{"pressure":978.90,"temperature":25.44,"depth":-0.35}` (mbar, °C; its depth field
uses a sea-level baseline — ignored). **Fusion is no longer needed**; nav consumes
this stream via the new `esp32_depth` driver (epsilon_sensors):
- QUIRK: stream only flows after a DTR/RTS chip reset on port-open (driver does this;
  plain `cat /dev/ttyACM0` shows nothing). Firmware tries MS5837 init ONCE per boot;
  on failure prints `{"error":"Init failed! Check wiring."}` and goes silent — the
  driver's silence-triggered reconnect (≈7 s cycle) retries init forever.
- Driver publishes: `/depth_raw` (per-parse, surface-referenced, unfiltered),
  `/depth` (FILTERED, 20 Hz zero-order hold of the ~7 Hz stream — user wants nav at
  20 Hz), `/esp32_depth/velocity_z` + `/esp32_depth/stale` (launch remaps
  sensor_bridge's old `/depth_fusion/*` subs to these — bridge code untouched).
- Filtering (bad values still occasionally appear): JSON parse → pressure gate
  900–1800 mbar + temp gate 0–45 °C → median-of-3 → rate gate (1.5 m/s + margin)
  with 3-read consensus re-anchor. Surface pressure captured at boot (median of
  first 3 s; sub starts at surface/in air).
- Launches: `depth_source:=esp32` (DEFAULT) `| i2c` (legacy chain, intact on disk).
  VERIFIED 2026-07-06: /sensors/depth 20.0 Hz, −0.008 m in air, depth_ok TRUE,
  velocity.z alive; full sysid HAND run logged fused.csv@19.9 Hz + depth_raw.csv.
- WATCH-ITEMS: (a) MS5837↔Xiao wiring is marginal at boot — init is a lottery
  (self-heals via reset-retry, but SECURE THE CONNECTOR before water);
  (b) ambient baseline drifted ~11 mbar over 30 min of bench runs (surface capture
  re-zeros per launch; harmless within a run, matters only for multi-hour soaks);
  (c) `sysid/esp32_check.py` = user health check.

## Session ritual (mandatory)
1. Read: auto-memory `handoff-sysid` → this file → PLAN.md.
2. SPOT-VERIFY one prior claim before building on it (e.g. re-run
   `python3 sysid/fit/verify_dry_run.py <a-dry-run-dir> <its-yaml>` — must PASS).
3. Execute the queue below, top-down. Water runs: USER executes, arm-gated, countdown,
   fail-safe-to-surface. No git commits — `.pre-*` timestamped backups. Never
   `devcontainer up` over SSH.
4. Update this file + the `handoff-sysid` memory before ending.

## What exists now (all on the sub, all verified 2026-07-05)
- `sysid_runner` + `sysid_logger` nodes in epsilon_bridge (symlink-installed),
  `sysid.launch.py` (DRY default; NEVER include thruster_bridge here),
  `sysid/sysid_run.sh` (DRY | LIVE=1 | HAND=1), `sysid/sequences/*.yaml`
  (dry_smoke, wrench_smoke, s3,s4,s5,s6,s7,s8,s9), `sysid/PROTOCOL.md` v1,
  `sysid/fit/{verify_dry_run,imu_rest_stats,hand_signs_report}.py`.
- DRY verification: cmd.csv of every sequence matches its YAML exactly
  (worst dev 0.000 across 28k+ ticks; ±150 ms logger receive-jitter slack —
  the COMMANDS are exact, logger timestamps at step edges jitter under load).
- Runner contract: 50 Hz /thrust_control always (zeros when idle/disarmed/done);
  own ~/arm gate (default DISARMED); hard time bound; auto-zero tail; markers
  on /sysid/marker; status idle|running:n/m|done|aborted on /sysid/status.
- Logger CSVs: imu (raw quat+gyro+accel+euler; EXACT-ZERO group = corrupt read,
  treat as missing), gravity, depth_raw, fused (+innov/stale), attitude
  (heading/roll), cmd (= what omni_control got), markers; frames/*.jpg at 4 Hz.

## Audit state (W0) — DONE except user-dependent items
- [x] thruster_bridge inverse-mix == exact algebraic inverse of sim control.py
      `_mix` (direction-preserving under its uniform normalization). COS45 match.
- [x] Safety chain sound end-to-end; watchdog-to-zero now at BOTH hops (see F1).
- [x] omni_control + launches + motortest path audited; F1–F3 fixed + DRY-verified.
- [x] sensor_bridge code audit done.
- [x] sensor_bridge PHYSICAL hand-motion verification DONE 2026-07-06 (run
      20260706-073327-a1_hand_signs-hand; integral test: gyro integrals match
      angle excursions within 2 deg on every maneuver). AXIS MAP LOCKED:
      heading = euler-yaw CCW+ (sim-consistent, sign +1); nav roll = euler-pitch
      minus level offset, PORT-DOWN = + (sim-consistent, sign +1); BODY PITCH
      (F6) = euler-roll channel minus level offset (~-170 deg), BOW-UP =
      NEGATIVE delta, rate = raw gx (same sign as euler-roll; for a
      bow-up-positive convention: pitch = -(eroll-level), rate = -gx).
      TWO SIGN BUGS FOUND+FIXED+RE-VERIFIED on the bench (F10/F11 below).
- [x] IMU live rate measured: 18.3 Hz mean (nominal 20), gaps to 0.43 s.

## Findings ledger (W0, 2026-07-05)
- F1 FIXED (HIGH): omni_control had NO watchdog (bridge death while armed latched
  last PWM forever). Now: >0.25 s silent /thrust_control → all PWM zeroed.
- F2 FIXED: short /thrust_control message indexed out-of-bounds → now rejected.
- F3 FIXED: 300 log-lines/s at 50 Hz removed from omni_control.
- F4 FIXED: stale COS45 doc pointer (submarine.py → control.py).
- F5 OPEN→LEGACY-ONLY (2026-07-06): depth_fusion near-zero re-anchor REAL on hardware:
  envelope [-1.0, 8.0] admits ~0 m; 4 bit-identical stuck-low reads ≤2 s apart
  re-anchor from mission depth to ~0. depth_fusion is now OUT of the active path
  (ESP32 architecture; depth_source:=esp32 default) — fix only if the legacy
  i2c path is ever revived.
- F6 NOTE (sysid-critical): body PITCH not published anywhere; sensor_bridge
  discards raw-quat euler-roll `_r` (sensor_bridge.py:140) — presumptive
  body-pitch channel (90°-rotated mount). A1 lift-bow test confirms channel+sign;
  the logger already records the full quaternion + eulers.
- F7 NOTE: /sensors/imu linear_acceleration = RAW body frame; fits use raw /imu.
- F8 NOTE (footgun): ws-root motortest.sh = legacy UNGATED teleop tester; the safe
  script is .devcontainer/motortest.sh.
- F9 NOTE (residual risk): omni_control dying mid-PWM-pulse can latch a GPIO HIGH
  (full thrust) — no software fix below it; keep node trivial + physical kill
  switch during water work.
- A2 DONE (2026-07-06, photos pending): full record in
  sysid/bench_measurements.yaml. Headlines: **mass 9.98 kg (22 lb) — sim had
  4.0 kg (2.5× off)**; envelope 0.559×0.457×0.356 m; vertical-pair roll arm
  0.184 m (sim's roll torque uses submarineWidth/2 = 0.23 → sim roll authority
  ~25% OPTIMISTIC — queued one-line fix with the S9 fit); corner rect x±0.229
  y±0.140 m (45° nominal; yaw arm is mounting-orientation dependent, 0.063 vs
  0.261 m — S4+S7 decide; sim's 0.32 above even the optimistic case). Applied
  to sim_calibration.yaml: subMass 9.98, subVolume 0.010078 (derived: mass +
  June's ~+1 N trim, S1/S3 refits), box-model inertias Iz .434 Iy .366 Ix .280
  (priors pending S2). Gate NOT measured (user: adapt for now — pool 0.5 m bar
  stays an assumption).
- 2026 OFFICIAL COURSE SPECS applied (2026-07-06, from the RoboSub team
  handbook, robonation.gitbook.io/robosub-resources 3.2 + robosub.org/programs/2026):
  gate 3.0×1.5 m (was 2.0), buoyant/floating with top ~0.2 m below surface
  (posts NO LONGER rendered to the floor — 1.5 m long, changes vision post-
  height cues); placards 0.305 m; slalom pipes fixed 0.9 m × 1 in (was
  randomized 1.2–1.5 m), floor-standing in shallow water / tops at 1.2 m
  (ASSUMPTION — mooring depth unspecified) in deep. Slalom lateral spacing
  1.524 m remains an ASSUMPTION (officially unspecified). Pool venue keeps
  the user's ~2 m practice-gate width + 0.5 m bar (both ASSUMED until
  measured). 2026 gate maker colors (black/red boxes, RED divider plate,
  Red-Right-Above) NOT yet modeled — sim posts are red; real posts likely
  white PVC: **vision keying on red may latch the divider/boxes at comp —
  W6/vision-refit item.** Backups .pre-2026spec.20260706-064948.
  Style scoring confirmed: every 90° counts, roll+pitch worth more than yaw.
- **FIRST HONEST-SIM GATE RUN vs 2026 SPECS: AcquireTarget FAILS (expected).**
  HW-faithful + full calibration (mass 9.98, FOV 26.9°): depth held 1.55
  ±0.08 the whole run, reference locked, but the gate pair never acquired
  from the 3 m start — a 26.9° camera sees only ~1.4 m of the 3 m gate
  (pair needs ≥6.3 m standoff). W6 MUST rework acquisition/standoff logic
  (single-post acquisition, start farther back, or search translation).
  This is the calibrated sim earning its keep — do not weaken it.
- PART A COMPLETE 2026-07-06 (A1 ✓ A2 ✓ A3 ✓ A4 ✓ A5 ✓; photos DELIVERED —
  sysid/photos/, confirm X-config corners + port/stbd verticals; corner toe
  angle → S7).
- 2026-07-06 PM4: **WATER-RUN WORKFLOW** (user flagged: no Pi access once wet).
  Final design PER USER — simple countdown, nothing to log mid-run:
  LIVE default ARM_DELAY now **90 s** (was 10), runs started DETACHED
  (`docker exec -d`, run parented to the container → WiFi loss mid-run is
  harmless, everything logs onboard). Recipe = top of PROTOCOL Part B.
  [A depth-triggered-arming variant was built, bench-verified both paths,
  then REMOVED at user request — countdown preferred. In git history at
  51741d6 if ever wanted.] Buoyancy trim variants (s4/s7/s8 _trim, verticals
  −22) DRY-verified same day; verifier arm-boundary fix in.
- 2026-07-06 PM3 follow-ups: **ESP32 wake-up issue RE-TESTED: RESOLVED** —
  5/5 clean first-attempt boots + 60 s driver soak (0 init-fails, 0
  reconnects, ±1 mm at rest); the driver's reset-retry stays as backstop.
  **t5 vibration: USER ACCEPTS AS-IS** (thrust equal by ear; watch-item
  closed, no action before water).
  NOTE FOR NEXT MODEL SESSION: the sim now auto-loads mass 2.5× heavier and
  FOV 2.6× narrower than every prior verification run — RE-RUN the M-series
  full course before trusting any nav behavior; expect standoff/FOV-loss logic
  and thrusterMaxForce (0.8 N nominal, clearly low for a 10 kg vehicle — S3/S7
  fit) to need W6 attention.
- A3 SUPERSEDED 2026-07-06 PM4 — **camera reconfigured to 640×320, HFOV = 74°
  (user-measured manually)**. History: the original A3 result (26.9°) was a
  REAL measurement OF THE OLD 320×240 capture mode, which the OV9782 driver
  center-crops. camera.py defaults now 640×320; the driver only offers
  full-res modes, so frames capture 1280×720 (MJPG) and downscale — full
  sensor width → full FOV. TWO camera-node fixes landed with this:
  (a) resize-before-rotate (~4× cheaper); (b) **rclpy publish fast path:
  msg.data = array("B", bytes) — assigning raw bytes runs rclpy's per-element
  validator, 93 ms/frame, which alone capped the node at 10 Hz.** Verified:
  640×320 @ 29.8 Hz published; upside-down mount handled (ROTATE_180 in the
  node, orientation confirmed on a live frame). cameraFov 74.0 in
  sim_calibration.yaml. The earlier "AcquireTarget fails at 3 m" W6 finding is
  RESOLVED by the real FOV — HW-faithful gate run COMPLETES again (274.7 s,
  crossed at z=1.55; style roll still honestly stalls ~49°). WATCH for W6
  vision refit: sim renders 320×240 while hardware publishes 640×320 —
  fraction-based logic is unaffected, but any PIXEL-count threshold (min blob
  area, px tolerances) sees 2× linear scale on hardware. Backup:
  camera.py.pre-res640.*.
- A5 DONE (2026-07-06, LIVE bench, props free): motor map USER-CONFIRMED all 6
  (0=LV 1=RV 2=RL 3=FL 4=FR 5=RR, correct prop every block); airflow directions
  EXACTLY match allocation.yaml (verticals: + = air down = thrust up, consistent
  with heave_sign=−1; left corners t2/t3: + = air bow ↔ negative surge coeffs;
  right corners t4/t5: + = air stern ↔ +1.0 coeffs); strength equal per pair by
  ear at ±100×15 s (runs a5_full_power + a5_full_power_t4t5). YAW LOOP CLOSED
  END-TO-END: all-corners+ = CCW spin = heading increasing = gyro+ (post-F11).
- F12 NEW (HIGH, power): **full-power corner thrust can BROWN OUT the Pi 5** on
  a drained battery — the 2nd full-power bench run hard-rebooted the Pi around
  t3/t4 (1st run on fresher charge completed). In water this ends the mission +
  risks SD corruption. Mitigations to pick before comp: supply isolation/cap,
  or cap |thrust| ≤ ~80 in thruster_bridge, or strict fresh-battery discipline.
  Sim note: thrusterMaxForce fits from S-runs should use ≤80% commands anyway.
- NOTE t5 (rear-right) vibration outlier, REPEATABLE: full-power IMU rms 0.46 →
  0.51 in reverse (2 runs) and 0.34 forward vs ~0.19–0.28 for every other motor.
  Thrust sounds equal (user) → likely prop imbalance/shaft play/loose mount.
  HAND-CHECK the t5 prop+mount before water day.
- F10 FIXED+VERIFIED (2026-07-06 A1, HIGH): roll_rate_sign was +1 but raw gy is
  OPPOSITE the published roll angle (integrals: −34.6 vs +36.6, +31.0 vs −32.3).
  The roll D-term would have been POSITIVE FEEDBACK in water. Default now −1 in
  sensor_bridge.py + hardware.launch.py (backups .pre-a1signs.20260706-024330);
  re-verified live post-fix: +30.1° port-down excursion vs +31.3° corrected
  integral.
- F11 FIXED+VERIFIED (2026-07-06 A1, HIGH): yaw_rate_sign was +1 but raw gz is
  OPPOSITE heading (CW turn: heading −90.9°, gz integral +90.9°). The heading
  rate loop would have been positive feedback. Default now −1 (same files);
  re-verified live post-fix: +75° CCW excursion vs +74.4° corrected integral.
  NOTE: the 2026-06-21 P4 "all rate signs +1 confirmed" claim was WRONG on both
  rate bits — angle signs were right, rate signs were not.
- **HW: MS5837 DEAD on bench 2026-07-05** ("PROM init failed (bus errors/CRC)",
  worse than June marginal). USER: reseat/resolder bus-2 wiring before water day;
  re-probe with i2c_test.py. Fallback: WITH_DEPTH=false (wider heave error bars).
  → **RESOLVED 2026-07-06 by the ESP32 architecture** (see "Depth architecture"
  above; the Pi 5's I2C, not the sensor, was the root cause). Remaining hardware
  item: SECURE the MS5837↔Xiao connector (boot-time init lottery).
- Backups this session: omni_control.cpp / thruster_bridge.py / setup.py
  `.pre-sysid.20260705-115746`.
- Backups 2026-07-06 (ESP32 integration): hardware/prequal/sysid launch files,
  sysid_logger.py, motortest.sh, sysid_run.sh, docker-compose.override.rpi.yml,
  epsilon_sensors setup.py — all `.pre-esp32.20260706-003515`.

- F13 NEW (HIGH, SAFETY, 2026-07-07 water 1): **fail-safe-to-surface FAILS
  below ~1.5-2 m.** Post-S3-auto-zero the sub sank 1.88→floor and stayed;
  same in S7. Buoyancy falls ~0.53 N/m of depth (~0.5-0.7 L compressible
  volume: trapped air in frame tubes / enclosure flex). At comp depths a
  disarmed sub SINKS. Fix before comp: fixed foam, find+purge the air, or
  accept + retrieval plan. Sim now models it (buoyancyDepthSlope).
- F14 NEW (2026-07-07): ESP32 depth reads a ~0.2-0.25 m TEMPERATURE transient
  on air↔water transitions, settling ~20-30 s (post-run in-air readings walk
  −0.03→−0.24 m). Conversion/scale verified sane. Mitigation: launch the
  stack with the sub already FLOATING (wet baseline at water temp — fits the
  90 s ARM_DELAY workflow); consider logging firmware temperature in
  depth_raw.csv.
- F15 NEW (2026-07-07): pool floor at ~2.0-2.1 m SENSOR depth ⇒ pool deeper
  than the 1.52 m venue assumption (user confirmed; meta water_depth values
  in the water-1 runs were defaults, not measurements). Measure + update the
  pool venue worldDepth.
- 4b CLOSED (2026-07-07): ESP32 chain in-water EXCELLENT — 7.14 Hz steady,
  max gap 0.15 s, zero >0.5 s gaps, raw-vs-fused p50 4-6 mm / p99 39-55 mm,
  zero >10 cm outliers (S3+S7 live windows). Filter gates never stressed.
- Backups 2026-07-07 (water-1 calibration): simulator.py, sub/config.py,
  sim_calibration.yaml — all `.pre-water1cal.20260707-030925`. New sim
  params: rollMomentArm (legacy default 0.23; calibrated 0.184),
  buoyancyDepthSlope (default 0.0; calibrated 0.53).

## W6 session (2026-07-07 PM) — epsilon-plant + re-tune + pump + vision
All backups `.pre-20260707-w6`: sub/config.py, simulator/simulator.py,
sub/data_structures.py, sub/tasks/common_subtasks.py, sub/tasks/gate_task.py,
config/pid_params.yaml, run_course.sh, esp32_depth.py, sysid_logger.py.
- **EPSILON-PLANT MODE (W5 remainder, DONE):** per-thruster force = MEASURED
  quadratic T=0.0015·(cmd%)² up to 40% (edge of data), tangent line above
  (conservative 9.6 N@100 vs unverified quadratic 15). Verified F(0.22) =
  0.73 N/motor = the real −22% hover trim exactly. New config params:
  epsilonPlant / thrustCurveQuadA / thrustCurveLinearizeAbove /
  epsilonPlantYawArm (0.133 = S4 τ(40)/4F(40)); env ROBOSUB_EPSILON_PLANT
  wins. run_course.sh: HW_FAITHFUL=1 defaults plant ON (EPSILON_PLANT=0 =
  legacy linear). Roll keeps tape-measured arm 0.184 (S7 cross-check ✓).
- **DEPTH RE-TUNE (M-series stall fix, DONE):** root cause buoyancy_ff 0.61
  — ≈1 N under old 0.8 N thrusters but ~6.5 N down-thrust under the real
  plant; integrator range ±0.144 couldn't cancel → parked 0.13 m low.
  pid_params now: buoyancy_ff 0.12, depth_i 0.15, depth_i_clamp 2.0,
  depth_p 3.0. Hold-verified: settle ±5 cm by 58 s, final err 3.5 mm at the
  near-neutral 1.5 m. SIM-tuned — hardware hold check REQUIRED before any
  water course (motortest hold 60 s).
- **STYLE-ROLL RESONANCE PUMP (DONE, honest):** StyleRollSubtask bang-bang
  pumps torque in phase with roll rate (self-locks at ωn), commits to
  rotation at ≥2.4 rad/s near upright, un-commits + re-pumps on stall,
  45 s timeout still COMPLETED. Offline + in-sim behavior: completes 720°
  in ~11 s if high-rate roll drag ~linear; swings ±89° + safe timeout under
  the (equally unverified) quadratic model. **S9b water 2 = the decider.**
  roll_power 0.95→0.80 (F12 margin).
- **VISION/CENTERING OVERHAUL (DONE — first verification batch exposed 4
  latent comp-relevant bugs):** (a) min-area thresholds scale by image area
  (ref 320×240; hw 640×320 = 2.67×); (b) gate pairing = ALL candidate pairs
  + separation gate 0.8–3.0× mean height (real gate w/h = 2.0) — without it
  in-line slalom reds paired as a phantom gate → 15 m overshoot; (c) blind
  centering fallback steers by get_gate_post_blobs() only — it was chasing
  the 2026 RED MAKER BOX; (d) Center completion tolerances re-derived for
  the MEASURED compass noise (1.32°): 10 px / rate 0.10 / 8 ticks (pre-roll
  0.06×12; old 6 px/0.04×12 & 0.02×24 were unattainable — every Center
  burned its full 40 s valve, all modes); (e) range-hold surge caps ±0.15/0.2
  sat inside the plant deadband (sub couldn't back up) → ±0.35/0.25 gain
  0.6; (f) sim renders the 2026 gate honestly: RED divider + BLACK/RED
  maker boxes; (g) **sim camera 320×240 → 320×160 (USER REQUEST)** = hw 2:1
  aspect, VFOV 41.3° (old 4:3 saw 18° more vertically). In-process
  closed-loop probes: sysid/fits/water1/center_probe.py + center_probe2.py
  (PASS 4–13 s). Gate mode: 280 s valve-burning → **166 s complete**.
- **VERIFICATION STATE (honest):** hold ✓, gate mode ✓ (only the 2 honest
  pump timeouts). FULL COURSE OPEN: STYLE_ROLL=0 flies (gate + slalom out +
  back in 211 s) then FAILS return-gate AcquireTarget (120 s) looking back
  through the slalom field — debug via sysid/fits/water1/return_probe.py;
  with roll, valves overrun 600 s → W7 re-pacing (Center 40→20 s, adaptive
  2nd roll segment, or DUR 900). Logs: sysid/fits/water1/w6_runs/.
- Water-2 package: sequences/s2b_depth_release.yaml + s9b_roll_pump.yaml
  (≤80 power; DRY-verify PENDING); esp32_depth publishes
  /esp32_depth/temperature → logger temp.csv (F14; bench check PENDING);
  sysid/WATER2_ADDENDUM.md = user checklist (PROTOCOL.md untouched — stale
  .swp from the user's vim; fold in after they recover/delete it).
- OPERATIONAL CAUTIONS (cost real time twice): pkill -f patterns self-match
  the ssh/bash command text — kill by PID or bracket patterns; TWO
  run_course instances concurrently pkill each other's nodes — never
  overlap batches/sysid runs (sysid_run.sh also pkills submarine_node).

## W5 honest sim — BUILT + VERIFIED 2026-07-06 (Session B model-side)
All sim upgrades live on the sub (backups `.pre-w5.20260706-003515`):
- **sim_calibration.yaml mechanism**: config.py overlays `sim:` onto
  SimulationConfig; simulator_node takes `sensors:` values as param defaults
  (CLI -p wins). File seeded with ALL measured sensor stats + provenance;
  dynamics section EMPTY (nominal priors in code until S2/S5/S7 fits).
- **Passive pitch DOF** (simulator.py): surge→bow-up coupling (0.06 N·m/N
  NOMINAL) + righting moment (0.5 N·m/rad NOMINAL) + quadratic drag + thrust-
  line lift on fz; **roll righting moment added** (0.3 N·m/rad NOMINAL).
- **Sensor emulation**: 18.2 Hz cadence + measured noise/corrupt-group rates
  (all modes, via calibration file); depth_mode truth|esp32|fused —
  esp32 (~7.2 Hz filtered, 20 Hz ZOH, driver-style vz) is the HW-faithful
  default; fused = legacy, still runnable (LEGACY_FUSED=1).
- **Venues**: ROBOSUB_VENUE=pool|comp (world 1.52|5.8); pool defaults
  mission depth 0.8, gate-leg depth 1.0, gate bar 0.5 m **ASSUMED — replace
  with A2 measurement**. mission.py: ROBOSUB_MISSION_DEPTH + ROBOSUB_GATE_DEPTH
  env overrides. run_course.sh: HW_FAITHFUL=1 → esp32 (no fusion), VENUE=.
- **VERIFIED**: HW-faithful FULL COURSE clean (311.7 s: gates −0.04/+0.01 m,
  slalom 6/6 ≤0.15 m, depth 1.48–1.62 vs 1.55, surface+complete); ideal gate
  clean; pool-venue HW gate clean (z=0.99 through the 0.5–1.52 opening) after
  fixing hardcoded GATE_CENTER_DEPTH (dove to 1.6 m in a 1.52 m pool → floor-
  clip timeout — venue bug found by the pool run itself).
- **KEY FINDING — style roll now stalls at ~36–46°** (all modes): with the
  nominal righting moment and the heave-priority thrust budget, direct roll
  authority runs out (≈0.14 N·m available vs 0.3 N·m righting at 90°). The
  720° roll times out loudly ×2 and the mission recovers + completes every
  run. THIS IS THE INTENDED HONESTY: S2 (righting/damping) + S9 (authority)
  measure the real numbers → W6 chooses direct vs resonance-pump. Do NOT
  "fix" by weakening the sim.
- W5 remainder queued: **epsilon-plant mode** (bridge inverse-mix + allocation
  + fitted per-thruster curves as the sim's actuation path) — build AFTER S7
  gives real curves. Camera FOV/rate model refit after A3.

## Bench numbers (W2, measured 2026-07-05, run 20260705-172215-a4_imu_rest-hand)
10-min rest, full stack load (camera+logger running = mission-representative):
- IMU rate 18.19 Hz mean (dt p50 55 ms, p99 56 ms); ONE gap >0.2 s in 640 s (max 0.45 s).
- Corrupt-read rates (zeroed-field convention): quat 0.0%, gyro 1.2%, accel 8.1%.
- Gyro noise std (rad/s): x 0.0020, y 0.0026, z 0.0014; bias ≈0, no drift.
- Accel (gravity-removed) noise std (m/s²): x 0.014, y 0.009, z 0.023;
  z bias +0.012 m/s² drifting +0.004/min (fusion's bias-EMA absorbs this scale).
- Euler-angle noise std: eroll 1.47°, epitch 0.55°, eyaw 1.32°; drift ≈0.03°/min.
- Gravity |g| = 9.7995 ± 0.002. Camera: 2504 frames @4 Hz over 640 s, zero stalls.
→ these feed sim sensor-model params in sim_calibration.yaml (Session B).
Still USER-owned (PROTOCOL PART A): A1 hand-signs, A2 mass/dims/photos, A3 FOV.

## Runs collected (sysid/runs/)
- 2026-07-05: 9 DRY runs (dry_smoke, wrench_smoke, s3–s9) — verification artifacts,
  not physics data. 1 bench rest capture `*-a4_imu_rest-hand` (10 min, W2 A4).
- 2026-07-06: `20260706-054048-esp32_chain_check-hand` (ESP32 depth chain verify:
  fused.csv 19.9 Hz ±2 cm in air, depth_raw.csv ~7 Hz) + 1 DRY dry_smoke coexistence.
- 2026-07-07 WATER SESSION 1 (pool, floor at ~2.0-2.1 m sensor depth):
  `20260707-023942-s1_static_trim-hand` (discard last 33 s),
  `20260707-025309-s2_tilt_release-hand` (5 roll + 5 pitch, AT SURFACE),
  `20260707-065749-s3_heave_staircase-live` (15.8 V; bottom from late down40),
  `20260707-071456-s4_yaw_staircase_trim-live` (15.40 V; CLEAN),
  `20260707-072200-s7_single_thruster_trim-live` (15.40 V; t2-t5 ON BOTTOM).

## Fits accepted (sysid/fits/ + sim_calibration.yaml)
WATER 1 (2026-07-07) — full detail sysid/fits/water1/water1_fit_report.md;
all values live in sim_calibration.yaml (overlay-verified, 16 params):
- Vertical thrust/motor 0.77/1.46/2.29 N at 20/30/40% → T≈0.0015·cmd² N
  (deadband <10%; T(100)≈15 N/motor is a LONG extrapolation). S3 ODE fit
  rms 4.7 mm. thrusterMaxForce 5.3 = linear band-match at 30-40% ONLY.
- Heave quad drag 84 N/(m/s)² (sim had 8).
- Buoyancy: +1 N surface → ~0 at 1.9 m → buoyancyDepthSlope 0.53 N/m (F13).
- Yaw: τ≈8.4e-4·cmd² N·m (τ(40)=1.28); steady ±18/50/79/103 °/s at
  10-40%, CCW/CW symmetric ≤3%; c2=0.352·(Iz/0.434)+c1 0.083 →
  angularDragCoeff_Z 0.40 (sim had 3.0 — 8x overdamped); yawMomentArm 0.151
  (⇒ corner lines ~55-63° toed OR weaker corner props; S5/S6 decide).
- Ix_eff 0.58 kg·m² MEASURED (S7 known-torque pulses; added-mass ×2.1).
- SUBMERGED righting k_roll=k_pitch≈4.6 N·m/rad (S7 post-pulse decays,
  K med ~8 s⁻², BG≈4.7 cm pendulum), ωn≈3 rad/s ζ≈0.19. S2 surface releases
  waterplane/trapped-water junk (K scatter ×17) — qualitative only.
- STYLE ROLL NUMBERS: direct 90° needs ~5.2 N·m vs 3.5 available at 80%
  (stall ~50°), 5.5 at 100% (marginal + F12 brownout). Resonance pump at
  0.45-0.5 Hz (gain ~1/(2ζ)≈2.6) clears 90° at 80% → W6 builds the pump.
- Corner pulse SIGNS all match allocation.yaml (+cmd=CCW all four);
  vertical −40 (down) response ~1.5-1.8× the +40 (up) — fwd/rev prop
  asymmetry, note for fine depth control.

## Queue — USER (water session 2; PROTOCOL Part B S5/S6/S8/S9/S10 + additions
## from sysid/fits/water1/water1_fit_report.md — PROTOCOL.md itself NOT edited,
## a vim .swp was present 2026-07-07)
1. ~~DECIDE F13~~ DECIDED 2026-07-07: pool-floor failures accepted +
   retrieval plan. Revisit before comp (disarmed sub sinks at comp depth).
2. Measure the actual pool depth (F15) + NEW LAUNCH RITUAL: float the sub in
   the water ≥60 s BEFORE launching the stack (F14 wet baseline).
3. Hardware sanity FIRST (new pid gains are sim-tuned only): motortest.sh
   hold — 60 s depth hold at ~1 m — before any course attempt.
4. Water 2 runs (full checklist sysid/WATER2_ADDENDUM.md): S5, S6, S8 (trim
   variant, MID-WATER), S9, **S9b roll pump (style-roll decider)**, S2b
   at-depth releases, S10. Fill meta.yaml battery_v every run.

## Queue — model (next session)
[W6 items all DONE 2026-07-07 PM — see "## W6 session".]
1. Ritual spot-verify, then RETURN-GATE AcquireTarget bug: run
   sysid/fits/water1/return_probe.py (in-container, SDL dummy) — dumps
   reds/posts/_is_slalom_red exclusions/pair from return-leg viewpoints.
   Suspect: _is_slalom_red falsely excluding the real far gate posts, or the
   far pair below thresholds. Fix + verify STYLE_ROLL=0 full course.
2. W7 valve re-pacing: Center timeout 40→20 s; ADAPTIVE second roll segment
   (skip if the first timed out); then full course WITH roll, DUR 900 → aim
   ×3 HW-faithful + ×1 ideal complete.
3. DRY-verify s2b_depth_release + s9b_roll_pump (verify_dry_run PASS both) —
   NEVER concurrently with course runs (sysid_run.sh pkills submarine_node).
4. Bench HAND run → confirm temp.csv populates (esp32 attached).
5. After water 2: S5/S6 fits (surge/sway drag, surgePitchCoupling, yaw-arm
   disambiguation), S8 deadband → thrust-curve low end, S9/S9b → high-rate
   roll drag (linear vs quad = pump verdict), S2b → righting at angle, F15
   pool depth → venue config; consider reverse-thrust asymmetry in the plant
   curve (S7 saw ~1.5× on verticals). Then W7 closed-loop verify + overlays.
- Session E: residual-gap ranking, Opus handoff, PROTOCOL final.

## Inviolables
Disarmed default · arm service gate · watchdog-to-zero (both hops) · countdown
before power · fail-safe-to-surface · user executes all water runs · no commits ·
no devcontainer up.

## 2026-07-12 — PERCEPTION ARC SPLIT OUT (camera + object detection)

Per the 2026-07-11 architecture decision (layers isolated; sim = control-only),
camera/CV tuning now lives on the laptop at /home/evan/workspaces/camerawork/
(start: CLAUDE.md + CONTEXT.md there; S10 analysis kit moved to its analysis/).
Origin handoff: ~/.claude/plans/handoff-camera-vision-tuning.md (laptop).
Resolved: launch-time gray_world — launch files default false but motortest.sh/
watertest.sh default GRAY_WORLD=1 (nav flies gray-world ON); sysid_run.sh
defaults 0 (S10 corpus is raw). Camera module being REPLACED 2026-07-12 (pink
edge cast expected gone); new-unit bench characterization is the perception
arc P0 — do not trust old-unit constants (cast map, blur, prop HSV) until then.
Control-side heads-up for the contract: real gate detection (old cam, untuned)
died by ~3 m vs the 4-5 m acquisition standoff control assumes.

## 2026-07-12 — DDS INTAKE FREEZE at wlan-hostile venues (perception arc finding)

At the 192.168.0.x venue, sysid_logger silently lost ALL topic intake
(imu+camera+depth) 10-15 s after launch in 4 of 5 bench HAND runs — process
alive, publishers alive, CSVs/jpgs just stop. Cause consistent with venue
AP/Wi-Fi dropping multicast (DDS liveliness); ROS_LOCALHOST_ONLY=1 does NOT
fix it on Humble/FastDDS 2.6. FIX (verified 2x, 195 s rate-perfect): launch
with FASTRTPS_DEFAULT_PROFILES_FILE=/home/robosub/robosub_ws/sysid/lo_unicast.xml
(loopback-unicast, no multicast; file added beside this note). CONTROL-ARC
HEADS-UP: motortest.sh/watertest.sh have no such profile — at a venue like
this, nav sensor intake could freeze the same way mid-run. Symptom check:
watch sysid_logger "logged:" counters for 60 s after launch. Note the
profile confines ROS to loopback (laptop-side echo/rviz will not see the
graph). Full evidence: laptop camerawork workspace CONTEXT.md 5b.

## 2026-07-12 — REPLACEMENT CAMERA LENS IS NARROWER (perception arc P0)

Bench-measured: new camera HFOV 58 deg +/- 4 (old unit was 74). ~25% less
lateral coverage — affects search patterns / gate acquisition geometry.
Upside: sharper lens (blur sigma ~2 px vs 3.5-4) + more px per prop:
poles should resolve to ~5 m vs old ~3 m. Pink edge cast GONE (verified,
white target). Capture resolution now 640x480->640x320 (FOV unchanged by
that change). Constants + evidence: camerawork workspace CONTEXT.md 2b +
analysis/reports/p0_close_out.md.

2026-07-13 — comp_v5 VISION RETUNE LANDED (user-approved). Bands re-aimed
at measured comp-water prop colors (red props render magenta; white PVC
renders pale blue), magenta-zone post guards, morph close, pole aspect
floor. Changed: src/robosub/robosub/sub/{config,vision,data_structures}.py
+ GRAY_WORLD default 1->0 in .devcontainer/{motortest,watertest}.sh —
the new bands are RAW-only. .pre-comp_v5.20260713 backups beside each.
CORRECTION to the entry above: "pink edge cast GONE" was FALSIFIED on
comp water — the cast is in the shared housing/dome optics (gain map
corr +0.99 with the old unit), left-edge R/G 2.30. Nav-facing summary:
camerawork workspace PERCEPTION_CONTRACT.md; evidence
analysis/reports/{p4_retune_delta,report2_landed_*}.

2026-07-13 (later) — loop2 gatelet update LANDED (user-approved).
Gatelet pairing envelope re-derived from human boxes (old one assumed
1.3 m poles — they are 0.914 m; it rejected 74% of TRUE pairs) + marker
V floor 85->75. Comp-corpus gatelet recall ~tripled (33/25/33/62% by
distance bin). Changed: src/robosub/robosub/sub/{config,data_structures}.py
(.pre-loop2.20260713 backups). Contract updated in camerawork workspace:
gatelet channel now usable for slalom steering WITH temporal smoothing;
best_pole remains untrusted (16.6% FP on comp empties).

2026-07-13 (final) — gatelet sep floor 0.5->0.4 landed (user-approved;
+6 pts gatelet recall at 1-2 m, zero FP cost; .pre-seplo04.20260713
backup on data_structures.py). Offline tuning is now at its measured
ceiling — next CV gains come from the exposure A/B (camerawork
HUMAN_TASKS 7c) + fresh capture/labels. Deployed gatelet contract:
34/31/34/62% by distance bin, FP 1.7%.

2026-07-13 (gate nav fix, user-authorized) — GATE LEG REWORKED to match
the comp perception reality (both outer posts co-visible on 1/1343
labeled frames): acquisition keys on POSTS not the pair; new
gate_geometry() steers on one-post+divider bearings; standoff is an
ORDINAL height band (metric range measured 2-6x far-biased — rejected).
SEP_TARGET/SEP_BAND removed. Changed: sub/tasks/common_subtasks.py
(.pre-gatefix.20260713). SIM VERIFIED read-only: gate + FULL course
complete post-change (headless run, calibrated physics + live pids).
CONTROL ARC NOTES: (1) sim runs need ROBOSUB_HFOV_DEG=70 and
ROBOSUB_FY_OVER_FX=1.0 (nav optics defaults are hardware); (2) marker
band is comp-water blue-white — sim vision checks need MARKER pointed
back at WHITE_HSV_RANGE; (3) evidence + handoff detail: camerawork
analysis/reports/p7_gate_fix.md. AUTON: slalom GO (comp venue), gate GO
pending water validation; capture 2 min of pool footage before any
practice-pool session (new-camera pool colors unmeasured).
