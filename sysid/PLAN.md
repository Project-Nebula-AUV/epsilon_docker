# SYSID: make the sim match the physical sub, then re-tune nav on the true sim

> Supersedes the (completed) sim-perfection plan that previously lived in this file.
> That effort's outcome: full course clean ×5 in sim, both sensing modes; fix ledger in
> memory `handoff-sim-task12-refactor`. THIS plan is the next arc.

## Context

Original intent was Fable porting the nav stack onto the physical sub by iterative water
testing. Fable is available ~2 more days, so the plan inverts: **use Fable's full context
of the sim + nav stack to (1) design and run physical system-identification tests, (2)
calibrate the sim until it reproduces the real sub's imperfections, (3) re-tune the nav
stack against the calibrated sim.** Then any future model (Opus) develops new task stacks
against a *trustworthy* sim, and porting to hardware becomes a small step instead of a
leap. Execution spans multiple sessions with different context windows — the plan, the
on-sub state directory, and the living handoff memory are structured so a fresh session
can rediscover and continue at full power.

**User decisions (interview, 2026-07-05):**
- Water access: **whenever we want** (own pool, 5 ft deep; comp is up to 19 ft).
- Comms during runs: **none** — scripted autonomous runs, log onboard, surface, pull.
- Session style: **many short runs** (10–20 × 30 s–3 min per session) is fine.
- Props: **real red gate + red/white poles exist**; placement will be eyeballed (prop
  WIDTHS are exact knowns — camera range-from-width is the underwater position sensor).
- Style roll: **vital — half the points.** The sub has ALREADY been ballasted to make
  rolling possible (righting moment deliberately small; this also explains the easy
  bow-up pitch under surge). Measure what the ballasting achieved; develop the roll
  maneuver (possibly multi-phase/pumped, given the weighting) in the calibrated sim.
- **The sub-side port layer is NOT trusted.** It is the residue of a long multi-session
  port that never achieved stabilization (hence the hardcoding fallback). Treat
  `epsilon_bridge` (sensor_bridge, thruster_bridge, launches) and its integration
  exactly like the sim code was treated: assume error from bad agents/practice, audit
  systematically, rewrite where unsound. The safety REQUIREMENTS (disarmed default,
  arm service, watchdog-to-zero, countdown starts) are preserved as requirements — the
  implementations are auditable.
- Fable-window bar: **≥1 full collect→fit→calibrate cycle AND ≥1 closed-loop physical
  verification** executed before Fable ends. Everything else built + documented.
- Depth-fusion (`epsilon_sensors/depth_fusion`) is **in scope to verify and fix**
  (user asked explicitly). Prior "don't touch" boundaries on the sub-side stack are
  lifted (see above) — only the safety requirements and the physical-run gating are
  inviolable.

## Ground truth about the physical sub (from user + prior sessions — VERIFY by test, don't trust)

- Pi 5 2 GB, container `robosub_dev`, ws `/home/robosub/robosub_ws` == `~/epsilon_docker`.
  NEVER `devcontainer up` over SSH (OOM). No commits — timestamped `.pre-*` backups.
- Motors (omni_control idx): **0=left-vertical, 1=right-vertical, 2=rear-left,
  3=front-left, 4=front-right, 5=rear-right**. PWM cmd −100..100, 2 GPIO pins/thruster.
- Signs: corners all-positive → **CCW yaw**; verticals **negative = descend**
  (matches heave_sign=−1). Static depth hold ≈ **−23 on both verticals** (≈ +1 N buoyant).
- **Surge causes bow-up pitch** needing much more down-thrust — the #1 reported
  real-vs-sim gap. There are NO pitch actuators (verticals are left/right) — pitch is
  passive and must be MODELED, not controlled.
- Motors not perfectly diagonal; equal power → curved motion. Existing crude asymmetry
  in `epsilon_bridge/config/allocation.yaml` (corners 0.92/0.96/1.00/1.00 from teleop
  basis logs). Deadband/PWM→thrust nonlinearity unmeasured.
- MS5837 depth: 10–25 % read success; `depth_fusion` (IMU dead-reckon + innovation gate
  + 4-read consensus) fills gaps. **Known vulnerability found in sim: stuck-low ~0 m
  corrupt reads can form consensus → re-anchor to 0 → transient ~1.5 m depth excursion**
  (observed in 2/6 HW-faithful sim runs). Verify + harden with real data.
- No leaks; recently swum; open-loop "hardcoded prequal" attempts failed (expected —
  that's why we're here).
- Nav chain: robosub `/thruster_commands` (BEST_EFFORT) → `thruster_bridge` (inverse
  mix → wrench → allocation.yaml → ±100) → `/thrust_control` → omni_control PWM.
  Arm-gated (`~/arm`), 50 Hz watchdog, disarmed default. `motortest.sh` pattern:
  launch disarmed → countdown → arm.

## Assets to reuse (do not rebuild)

- **Calibrated-sim seed**: `robosub/sub/control.py` (MotionController), rewritten task
  engine + tasks, `sim_gui_node` (live view :8765), `run_course.sh`, `course_monitor.py`,
  `judge.py` — all on the sub ws root, all working (5 clean full-course runs).
- **Fast-time harness pattern** (scratchpad `harness.py`, session-local — REBUILD from
  the recipe in memory): pure-python port of `applyPhysics` + MotionController loop.
  This becomes the sysid fitting engine.
- `recorder_node` (MP4 + HUD overlay) for vision footage; `~/robosub_recordings`.
- Teleop basis logs (surge/sway/yaw patterns) — historical reference only; supersede
  with proper sysid fits.

## Canonical state (multi-session continuity — the core of this plan)

All sysid state lives ON THE SUB at `~/epsilon_docker/sysid/`:

```
sysid/
  RESUME.md            # machine-scannable state: checklist of runs collected,
                       # fits done, params accepted, next queue. EVERY session
                       # updates it before ending. A fresh session reads this
                       # + the plan + memory, then SPOT-VERIFIES one claim
                       # (e.g. re-runs one fit) before building further.
  PROTOCOL.md          # the water-test menu: per-run purpose, script command,
                       # duration, operator steps, expected artifacts. Written
                       # for the USER to execute without the model present.
  runs/<YYYYMMDD-HHMM-name>/   # one dir per physical run: cmd.csv, imu.csv,
                       # depth.csv, fused.csv, frames/*.jpg, meta.yaml (what,
                       # venue, water depth, battery, notes)
  fits/                # fitting outputs, overlay plots, residual reports
  sim_calibration.yaml # THE product: every fitted parameter, with provenance
                       # comments (run IDs + fit script + residual). The sim
                       # loads this file; nothing calibrated lives anywhere else.
```

Living memory: new file `handoff-sysid.md` (same discipline as
`handoff-sim-task12-refactor`): updated at every milestone, indexed in MEMORY.md.
Per-session loop: (1) read memory + RESUME.md, (2) spot-verify one prior claim,
(3) execute next queue items, (4) update RESUME.md + memory. Sessions must assume
NOTHING from prior context windows except these artifacts.

## Workstreams

### W0 — Audit the sub-side port layer (FIRST; fits built on unaudited sensing are void)
Treat `epsilon_bridge` + integration as hostile code (the sim-refactor posture):
- **sensor_bridge**: verify axis mapping/signs/offsets by PHYSICAL hand-motion tests
  (tilt starboard-down → `/sensors/roll` sign; rotate CW → heading/gyro; lift bow →
  which euler channel moves = the body-pitch channel the sysid logger needs). Verify
  the level-capture-at-boot behavior and that published rates/latencies are real.
- **thruster_bridge**: verify the inverse-mix math analytically against the (now fully
  understood) sim mixer — Fable knows both sides; check the wrench reconstruction is
  actually direction-preserving, check allocation load, saturation, watchdog paths.
- **omni_control + launches + motortest/prequal path**: config/pin map sanity, arming
  order, failure modes. DRY-verify command patterns with motors unpowered (established
  P3 pattern: omni_control not running, echo `/thrust_control`).
- Rewrite what fails audit (`.pre-*` backups); safety requirements preserved. Findings
  ledger in `sysid/RESUME.md` — every downstream fit cites which audit state it ran on.

### W1 — Instrumentation: sysid runner + logger (new pkg code, sim untouched)
New `epsilon_bridge` node `sysid_runner`: executes a named test sequence from a YAML
script (list of {t, thrust_control[6]} or {t, wrench[5]} steps), **arm-gated with
countdown exactly like motortest.sh**, watchdog-bounded, always ends at zero thrust
(positive buoyancy = surfacing failsafe). Logs onboard to `sysid/runs/<id>/`:
commanded values, raw `/imu` (20 Hz, incl. quaternion — body pitch must be derivable),
`/depth_raw`, fused `/sensors/depth` + `/depth_fusion/velocity_z`, `/sensors/{heading,
roll}`, camera JPEGs at 3–5 Hz. One wrapper script `sysid_run.sh <test-name>` = place
sub, run, stand clear, retrieve. CSV+JPEG (no rosbag — 2 GB Pi).

### W2 — Bench & static protocol (no water time cost)
- Tape-measure + scale: mass, hull dims, each thruster's position + axis angle
  (photos → angles), vertical-pair lateral arm. → priors for the fit.
- IMU at rest 10 min: noise floors, bias drift, actual publish rate.
- Camera: real FOV check (known-width object at taped distance in AIR), frame rate;
  print checkerboard if convenient (nice-to-have).
- Prop spin-direction sanity at low duty (out of water, brief).

### W3 — In-water excitation menu (each 30 s–3 min, PROTOCOL.md is the deliverable)
Pool = 1.52 m: test depth targets 0.6–1.0 m (mission depth override for pool venue).
- **S1 static trim**: motors off, float: rest pitch/roll (IMU), surface trim.
- **S2 tilt-release** (hand-tilt ~20–30° roll, release, motors off; repeat pitch):
  oscillation freq + decay → righting stiffness, inertia, angular damping for roll AND
  pitch. THE test that decides the style-roll strategy. Also doable day 1, no scripts.
- **S3 heave staircase**: verticals at −10/−20/−30/−40, 5 s each, then +steps up:
  depth-rate & steady offsets → vertical thrust curve incl. deadband + buoyancy +
  heave drag. Doubles as the **fusion verification dataset** (fused vs staircase truth).
- **S4 yaw staircase**: corners uniform ±10/20/30/40 → gyro_z plateaus → yaw
  authority + drag + deadband.
- **S5 surge steps at the gate**: start ~4–5 m facing the real gate, surge 20/35/50
  bursts: camera gate-width time series = range track → surge thrust/drag; IMU pitch
  channel captures the bow-up coupling vs speed (the key coupling parameter).
- **S6 sway steps** at the gate: bearing/width drift → sway thrust/drag.
- **S7 single-thruster pulses**: each of 6 thrusters, ±20/±40, 2 s, at depth:
  6×(gyro_x, gyro_z, accel, depth-rate) response matrix → TRUE per-thruster direction
  vectors + moment arms → refit allocation (this is where "not perfectly diagonal"
  and "curved motion" get numbers).
- **S8 deadband ramps**: slow 0→±15 ramps per axis; motion-onset command.
- **S9 roll authority**: vertical differential pulses ±40/±70/±100 → roll angle
  reached vs righting moment (feeds style-roll strategy directly).
- **S10 vision filming**: scripted slow approach/orbit of gate and poles + hand-carried
  passes; recorder_node MP4 + logger JPEGs → offline vision fitting corpus.
Ordering for session 1: S1, S2, S3, S4, S7 (highest information/minute).

### W4 — Fitting pipeline (offline; runs in container or locally)
`sysid/fit/` scripts (extend the fast-time harness pattern):
- Parameterized plant: per-thruster {gain_fwd, gain_rev, deadband, axis unit vector,
  position}, rigid body {mass, I_x/I_y/I_z, CG-CB offset → righting moments in roll AND
  pitch, buoyancy}, drag {linear+quadratic per axis}, actuator lag (1st order),
  sensor models {rates, latency, noise; depth read-success/corruption stats from S3}.
- Replay each run's commanded sequence through the plant, least-squares the residuals
  vs logged IMU/depth/camera-range; per-run overlay plots into `fits/`.
- **Fusion verification**: replay logged raw `/depth_raw`+`/imu` through depth_fusion
  offline vs staircase truth; fix the near-zero-consensus vulnerability (guard
  consensus against clusters at ≈0 m / require physical plausibility vs prediction);
  re-verify on the same logs + in sim HW-faithful mode. (`.pre-*` backup first.)
- **Vision refit**: run recorded real footage through `robosub/sub/vision.py` +
  `data_structures.Vision` offline (they take BGR arrays directly); refit HSV ranges /
  add normalization so detection rates on labeled real clips match sim-level
  reliability; also fit the sim camera's color/contrast rendering toward real footage
  (sim honesty: make the sim LOOK like the real camera, not vice versa).

### W5 — Sim upgrade: the calibrated plant
`simulator.py` + `config.py` changes (each flagged, sim never made easier):
- Load `sim_calibration.yaml` (nominal defaults when absent — current behavior).
- **Passive pitch DOF**: pitch state + righting moment + surge→pitch coupling + pitch
  drag (camera projection already consumes pitch). Roll gets its righting moment too
  (this will break the current trivially-easy sim roll — INTENDED).
- **Epsilon plant mode** (`epsilon_plant:=true`): instead of ideal mix→forces, apply
  the REAL chain: bridge inverse-mix + allocation.yaml + fitted per-thruster curves
  (deadband, asymmetric gains, true axis vectors, lag) → forces. Nav-in-sim then equals
  nav-on-sub bit-for-bit at the command level.
- Venue configs: `ROBOSUB_VENUE=pool|comp` (worldDepth 1.52 vs 5.8, mission depth
  targets, prop layout spans). Keep the staggered-lane feature.
- Sensor models per calibration: IMU noise/rate, heading noise, depth emulation params
  refitted from S3 statistics (attempt rate, success prob, corruption modes measured,
  not guessed).
- Camera: real FOV, 320×240@28 Hz cadence, fitted color/contrast transform.

### W6 — Nav re-tune + style-roll strategy on the calibrated sim
- Re-tune `pid_params.yaml` gains against the calibrated plant (fast-time harness
  first, then live sim); add **deadband/asymmetry feedforward compensation in
  thruster_bridge** (inverse of fitted curves — makes the plant look linear to nav;
  lives in epsilon_bridge, robosub contract unchanged).
- Depth law must handle the surge-pitch coupling (feedforward heave vs surge, or
  accept+verify the PID handles it).
- **Style roll (half the points — top priority after basic stability)**: the sub is
  already ballasted for it. S2 (tilt-release) + S9 (roll-authority pulses) measure what
  the ballasting achieved; the calibrated sim (righting moment + true thruster curves)
  then develops the maneuver: direct roll if authority suffices, else a **resonance
  pump** (oscillate roll torque at the measured natural frequency to build amplitude,
  then commit through the rotations — StyleRollSubtask grows a pump phase). Because
  righting stiffness is deliberately low, ALSO verify in sim + water that post-roll
  self-leveling and depth-hold-under-pitch still behave (the weighting trades passive
  stability for rollability — nav's roll/pitch handling must carry the difference).
  If numbers say more ballast trim is needed, quantify exactly how much; user decides.
- Re-run the full M-series verification (gate/roll/slalom/full, judge.py evidence) on
  the calibrated sim, pool venue AND comp venue.

### W7 — Closed-loop physical verification (the loop-closer, within Fable window)
- Scripted closed-loop tests mirroring sim primitives: heading step ±40°, depth step
  0.4→0.9 m, station-hold 60 s, gate approach+drive-through at the real gate.
  Each: run on sub (logged) AND in calibrated sim from same initial state; overlay
  judge report (rise time, overshoot, steady error, pitch excursion) — **the
  match-quality metric is these overlays**, iterate fit↔sim until envelopes agree.
- Ship-shape end state each session: `motortest.sh`/prequal path still runnable.

### W8 — What Opus inherits (write-out at Fable end)
- `sysid/RESUME.md` + PROTOCOL.md current; `handoff-sysid.md` memory complete with:
  validated collect→fit→verify loop instructions, calibration provenance, style-roll
  strategy state, known residual gaps ranked, and the per-session rediscovery ritual.
- A "first Opus session" script: verify environment, spot-check one fit, run one
  calibrated-sim full course, then continue the queue.

## Fable-window schedule (aggressive but each step degrades gracefully)

- **Session A (audit + build)**: W0 audit of the sub-side layer (hand-motion sensor
  verification with the user, analytic bridge check, DRY patterns); W1 runner+logger
  built + bench-verified dry; W2 bench numbers; PROTOCOL.md v1; user does S1/S2 (no
  scripts needed) same day if water available.
- **Session B (water 1 + fit)**: user executes S1–S4, S7 (+S10 filming); pull logs;
  fit vertical/yaw/thruster-matrix; fusion verify+fix; first sim_calibration.yaml;
  pitch DOF + plant mode into sim.
- **Session C (water 2 + tune + verify)**: S5/S6/S8/S9 (+repeats of anything noisy);
  complete fits; nav re-tune; style-roll strategy chosen with numbers; closed-loop
  physical tests (W7) executed + overlaid; iterate once.
- **Session D (wrap)**: residual-gap ranking, Opus handoff (W8). Any leftover water
  items queued in PROTOCOL.md with exact commands.

## Risks / fallbacks
- **5 ft pool vs 19 ft comp**: thrust/drag/attitude fits transfer; surface/floor
  proximity effects at 1.5 m (surface suction, floor wash) noted per-run in meta.yaml
  and kept out of fits where visible (mid-depth windows only). Comp-depth behavior is
  extrapolation — flagged honestly in the calibration file.
- **Depth sensor worsens**: S3/fusion work degrades to IMU+camera-range only; heave fit
  quality drops — recorded as wider uncertainty, not fake precision.
- **Roll truly infeasible even pumped**: sim proves it with numbers → ballast-trim
  recommendation with exact required CG shift; yaw-spin style as mission stopgap.
- **Pi load**: logger is CSV+JPEG, sequences short; never run sim + sysid together.
- **Session dies mid-arc**: RESUME.md + memory + this plan are sufficient to resume;
  every fit is re-runnable from `runs/` (raw data is the source of truth, never the
  fitted numbers).

## Inviolables (everything else is auditable/rewritable)
Safety REQUIREMENTS: disarmed default, arm service gate, watchdog-to-zero, countdown
before any powered run, fail-safe-to-surface, all water runs executed by the user.
Plus: local frozen backup dir untouched, no git commits (`.pre-*` backups), no
`devcontainer up` over SSH.

## Verification
- W1: dry-run each sequence with omni_control NOT running (thrust_control echo only) —
  the established DRY pattern.
- Fits: every parameter ships with overlay plot + residual in `fits/`; a fit that
  doesn't visibly track the log is rejected, not shipped.
- Sim match: closed-loop overlay envelopes (W7) are THE acceptance test.
- Nav: M-series judge evidence on calibrated sim, both venues; physical closed-loop
  tests match sim predictions within the overlay envelopes.
- Continuity: at end of every session, a cold-start checklist confirms RESUME.md +
  memory alone are sufficient (no reliance on session context).
