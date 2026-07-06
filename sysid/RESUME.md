# SYSID RESUME — canonical state. Update BEFORE ending every session.

STATUS: SESSION A COMPLETE (2026-07-05). W0 audited+hardened, W1 built+DRY-verified
(all 7 sequences PASS), PROTOCOL.md v1 live, fit-side starter scripts in sysid/fit/.
2026-07-06: **DEPTH SENSOR WORKS — new ESP32 architecture, integrated + verified**
(see "Depth architecture" below; fusion out of the active path).
2026-07-06 PM: **W5 HONEST SIM BUILT + VERIFIED** (see "W5 honest sim" below):
calibration mechanism, passive pitch DOF, measured sensor models, esp32 depth
emulation, pool|comp venues — full course clean under all of it; style roll now
honestly stalls (S2/S9 + W6 will resolve).
NEXT: **user executes PART A of PROTOCOL.md (bench: A1 hand-signs, A2 measurements
incl. REAL GATE BAR DEPTH in the pool, A3 camera FOV; A4 done 2026-07-05) and
secures the MS5837↔Xiao wiring**, then water session 1 (S1,S2,S3,S4,S7).
Model then runs Session B fits (axis map → S3/S4/S7 fits → calibration values
replace nominals → epsilon-plant mode → W6 re-tune + style-roll strategy).

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
- 2026-07-06 PM4: **WATER-RUN WORKFLOW built + verified** (user flagged: no Pi
  access once wet; old fixed 10-15 s countdown + SSH-tethered run were both
  unusable in water). sysid_run.sh SUBMERGE=1 mode: start detached on land
  (`docker exec -d`), readiness gate, then ARM ON SUSTAINED DEPTH
  (>0.35 m held 3 s from the ESP32 sensor) + 5 s let-go grace; never-submerged
  → 7-min timeout, teardown DISARMED. Bench-verified BOTH paths detached
  (zero-thrust sequence): trigger→arm→done with the launching SSH session
  irrelevant, and timeout→NO-ARM. Recipe = top of PROTOCOL Part B. Buoyancy
  trim variants (s4/s7/s8 _trim, verticals −22) DRY-verified same day.
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
- A3 DONE (2026-07-06): **camera horizontal FOV MEASURED = 26.9°** (3 ft PVC at
  8 ft spans 251/320 px → f=669 px; cross-checked by predicted+observed frame
  overflow at 5 ft, two captures). The sim assumed 70° — the 320×240 mode
  center-crops the sensor. Now in sim_calibration.yaml (`sim: cameraFov`).
  MISSION IMPLICATIONS: all range-from-width was 2.6× off; the 2 m gate fills
  the real frame at ~4.3 m standoff (not ~1.4 m) — CenterOnGateHalf standoffs,
  FOV-loss fallbacks, and S5/S6 gate-visibility distances must be rechecked
  under 26.9°; W6 re-tune runs the sim with this value (loads automatically).
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

## Fits accepted (sysid/fits/ + sim_calibration.yaml)
(none yet — first fits come from water session 1 data)

## Queue — USER (bench + water day 1; exact steps in PROTOCOL.md)
1. ~~Fix depth sensor~~ DONE via ESP32 architecture (2026-07-06). Remaining:
   SECURE the MS5837↔Xiao connector (init lottery at boot); quick health check
   any time: `python3 sysid/esp32_check.py` in the container.
2. PART A bench: A1 hand-signs (10 min, blocks the axis map) · A2 measurements +
   photos · A3 camera FOV · (A5 prop-spin optional).
3. Water session 1: S1, S2 (hand), S3, S4, S7 (LIVE). Fill meta.yaml per run.

## Queue — model Session B (after water 1 data exists)
1. Spot-verify: one verify_dry_run PASS + read a1 hand-signs report → lock axis map
   (update F6 with the confirmed channel/sign).
2. imu_rest_stats.py on the a4 run → sensor noise/rate numbers → calibration file.
3. Fit vertical thrust/buoyancy/drag (S3), yaw authority/drag (S4), per-thruster
   vectors + allocation refit (S7). Overlay plots into sysid/fits/.
4. Depth-fusion offline replay vs S3 truth; fix F5; re-verify same logs.
   [2026-07-06: fusion now LEGACY-ONLY (ESP32 path is default) — do this step only
   if the i2c path is revived; otherwise skip to 4b.]
4b. NEW (ESP32 depth): 10-min in-water/bench soak of the esp32 chain from the S3
   logs — accept rate, outlier count+signature pre/post filter (driver status
   counters + depth_raw.csv vs fused.csv), tune filter gates if the "obviously
   bad" values slip through. S3 staircase now ALSO validates the ESP32 depth path
   + filter end-to-end (its fusion-verification purpose is legacy-only).
5. First sim_calibration.yaml (provenance comments per param).
6. Sim upgrade W5: passive pitch DOF + righting moments + epsilon-plant mode +
   venue configs (pool 1.52 m) + sensor models from measured stats.
   NEW (ESP32 depth): refit the sim's depth-sensor emulation to the NEW stream
   statistics (~7 Hz JSON, high success rate, occasional outliers, 20 Hz ZOH
   republish) instead of the old 10–25 % MS5837-on-Pi model.
- Session C: S5/S6/S8/S9 → complete fits → nav re-tune + feedforward → style-roll
  strategy with numbers → W7 closed-loop verify + overlays.
- Session D: residual-gap ranking, Opus handoff, PROTOCOL final.

## Inviolables
Disarmed default · arm service gate · watchdog-to-zero (both hops) · countdown
before power · fail-safe-to-surface · user executes all water runs · no commits ·
no devcontainer up.
