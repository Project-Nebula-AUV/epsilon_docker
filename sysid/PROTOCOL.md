# SYSID PROTOCOL v1 — the water-test menu (written to run WITHOUT the model present)

Every run: place the sub, start the command, stand clear, retrieve, done.
All data logs onboard to `sysid/runs/<timestamp>-<name>-<mode>/`. Nothing streams.

## Safety (non-negotiable)
- Motors only ever run via `LIVE=1 ./sysid/sysid_run.sh <seq>` — it launches
  DISARMED, health-checks the stack, counts down 10 s, then arms. **Stand clear
  during the countdown.** Every sequence ends at zero thrust and the sub is
  positively buoyant → it surfaces on its own.
- Abort anytime: **Ctrl-C** in the run shell (disarms + kills stack), or from
  a second shell:
  `ros2 service call /sysid_runner/arm std_srvs/srv/SetBool "{data: false}"`
- If a run looks wrong (wrong direction, hitting wall/floor), abort. The log up
  to the abort is still useful data — note what happened in meta.yaml.

## How to run anything
```
ssh robosub@epsilon
docker exec -it robosub_dev bash
cd /home/robosub/robosub_ws
./sysid/sysid_run.sh                      # no args = list sequences
```
Modes: default = DRY (no motors, commands logged only) · `LIVE=1` = motors ·
`HAND=1` = sensors+logger only (motors impossible), Ctrl-C to end.
After every run, edit `sysid/runs/<id>/meta.yaml`: venue, water depth,
battery voltage, and anything odd you saw.

## ⚠ Current hardware status (2026-07-05)
- **Depth sensor: WORKING — new architecture (2026-07-06).** The MS5837 now
  hangs off a Xiao ESP32-C3 (its own I2C), which streams JSON to the Pi over
  USB (`/dev/ttyACM0`). The Pi 5's I2C buses were the root cause of all the
  June/July depth failures — never move the sensor back onto the Pi. Depth is
  filtered by the new `esp32_depth` driver and reaches nav at 20 Hz; no fusion
  involved (the old fused chain remains available as `depth_source:=i2c`,
  legacy only).
  - Health check any time: `python3 sysid/esp32_check.py` (inside the
    container). Healthy = `STREAMING` + ambient ~960–1000 mbar lines.
    (`cat /dev/ttyACM0` shows nothing — the chip needs a reset-on-open, which
    all the tools do for you.)
  - **Before water day: SECURE the MS5837↔Xiao wiring.** The firmware tries
    the sensor once per boot; a marginal connector makes startup a lottery
    (the driver auto-retries every ~7 s until it wins, but a solid connector
    means it wins immediately).
  - If depth dies anyway, everything still runs: add `WITH_DEPTH=false` —
    heave fits then lean on IMU+camera with wider error bars.
- IMU: 18.3 Hz mean, gaps to 0.43 s. Working.
- Camera working (320×240).

---

# PART A — BENCH, before any water (~45 min total, sub on a table)

## A1. Hand-motion sensor verification  (≈10 min, THE blocking item)
Purpose: confirm the sign/channel of every nav sensor + find the body-PITCH
channel (needed for the bow-up coupling measurement; it is NOT published today).
```
HAND=1 ./sysid/sysid_run.sh a1_hand_signs "hand motion sign check"
```
The sub must be **LEVEL and STILL for the first 2 s** (roll offset capture).
Then do each maneuver ~5 s apart, and after each one drop a marker from a
second shell (edit the text each time):
```
ros2 topic pub --once /sysid/marker std_msgs/msg/String "{data: 'roll-starboard-down'}"
```
Maneuvers, in order (hold each tilt ~3 s, return to level between):
1. `roll-starboard-down` — tilt RIGHT side down ~20°
2. `roll-port-down` — tilt LEFT side down ~20°
3. `pitch-bow-up` — lift the NOSE ~20°  ← finds the body-pitch channel
4. `pitch-bow-down` — nose down ~20°
5. `yaw-cw` — rotate clockwise (viewed from above) ~45° and back
6. `lift-up` — lift the whole sub ~0.5 m straight up, hold, lower
Ctrl-C when done. Tell the model the run id; it reads imu.csv/attitude.csv
against markers.csv and locks in the axis map.

## A2. Physical measurements (≈20 min, tape measure + scale + phone)
Into a note or straight into `sysid/bench_measurements.yaml`:
- Total mass (kg) as-ballasted, ready to dive.
- Hull: length, width, height (m).
- For EACH thruster 0–5: position relative to the hull center (x fwd, y right,
  z down, in cm — coarse is fine) and a PHOTO along its thrust axis (these
  give the axis angles; the corners are nominally 45° — photos catch the
  "not perfectly diagonal" truth).
- Vertical pair: lateral distance between the two vertical thrusters (cm).
- Photos: top view, side view, front view of the whole sub with a ruler in frame.

## A3. Camera FOV in air (≈5 min)
Tape a known-width object (e.g. the 0.61 m gate PVC section, or a metre stick)
at a measured distance (e.g. exactly 2.0 m) facing the camera, centered:
```
HAND=1 WITH_DEPTH=false ./sysid/sysid_run.sh a3_camera_fov "metre stick at 2.0m"
```
Let it log ~10 s, Ctrl-C. Note object width + distance in meta.yaml notes.
(Model computes true FOV from pixel width in the frames.)

## A4. IMU 10-min rest (no touching — can also be run remotely by the model)
```
HAND=1 WITH_DEPTH=false WITH_CAMERA=false ./sysid/sysid_run.sh a4_imu_rest "10 min rest"
```
Leave completely still 10 min, Ctrl-C. → noise floors, bias drift, true rate.

## A5. Prop spin sanity (out of water, ≤5 s per motor, LIVE — props free!)
Only if you want visual confirmation of the motor map before water:
```
LIVE=1 WITH_DEPTH=false ARM_DELAY=15 ./sysid/sysid_run.sh s7_single_thruster "bench spin check"
```
Watch which prop spins for each marker (t0..t5) and note any that look wrong.
Abort (Ctrl-C) after the first two thrusters if you only need a spot check —
out-of-water runs should stay short.

---

# PART B — WATER SESSION 1 (pool, ~60–90 min incl. setup)
Order = highest information per minute. Charge battery, note voltage per run.

## S1. Static trim (2 min, motors off)
```
HAND=1 ./sysid/sysid_run.sh s1_static_trim "static float"
```
Sub floating free mid-pool, hands off 60 s. → rest pitch/roll, surface trim.
(If the depth sensor is alive, this also gives the pressure-noise floor.)

## S2. Tilt-release (5 min, motors off — THE style-roll measurement)
Same HAND command, name `s2_tilt_release`. Sub floating, submerged ~0.5 m.
WHY: the ballasting put CG only slightly below CB, so the sub is a weak
pendulum. Tilt + release makes it oscillate: the swing FREQUENCY gives the
righting stiffness (vs inertia), the DECAY gives angular damping — the three
numbers the sim's passive-attitude model now runs on (currently guesses).
With S9's authority numbers this decides direct-roll vs resonance-pump.
- Roll it ~20–30° by hand, LET GO CLEANLY (no push), hands off while it
  wobbles to rest. Marker `tilt-roll-1` etc. **×5.**
- Same for pitch (bow down ~20°, release): `tilt-pitch-1` … **×5.**
BUOYANCY NOTE: the sub drifting up during/after a release is FINE — only the
first 3–5 swings carry the data. Pull it back down between reps; if it breaks
the surface mid-wobble, the rep still counts unless the release was messy.

## S3. Heave staircase (45 s LIVE)
```
LIVE=1 ./sysid/sysid_run.sh s3_heave_staircase "pool, battery XX.XV"
```
Start the sub ~0.6–0.8 m deep mid-pool, level, still. It steps the verticals
down (−10/−20/−30/−40, 5 s each), recovers, steps up (+10/+20). Watch it
doesn't reach the floor (1.52 m) or break the surface — abort if so.
→ vertical thrust curve, deadband, buoyancy, heave drag; **also the end-to-end
in-water validation of the ESP32 depth path + its filter** (depth_raw.csv vs
fused.csv; the old depth-fusion verification purpose is legacy-only now).

## S4. Yaw staircase (55 s LIVE) — USE THE TRIM VARIANT
`s4_yaw_staircase_trim` — same yaw steps, plus a constant −22 hold-down on
the verticals so the buoyant hull stays at depth for the whole run (the
un-trimmed `s4_yaw_staircase` would surface mid-run). Sub mid-depth,
mid-pool; spins CCW ×4 steps then CW ×4.

## S7. Single-thruster pulses (95 s LIVE — the allocation truth test) — TRIM VARIANT
`s7_single_thruster_trim` — each thruster fires alone ±20/±40 for 2 s;
rests + corner blocks carry the −22 vertical hold-down, but the t0/t1
vertical-pulse blocks are PURE (isolation) so the sub bobs there — expected.
Sub mid-depth, ≥1 m from walls. Expect small lurches/rotations.
→ true per-thruster direction vectors + moment arms → allocation refit.

## Repeat guidance
If a run looked disturbed (wall contact, wave from your hand, early abort),
just run it again — runs are cheap, note the bad one in meta.yaml.

---

# PART C — WATER SESSION 2 (needs the gate + poles placed)

## S5. Surge steps at the gate (35 s LIVE)
Place the sub 4–5 m from the REAL gate, FACING it, mid-depth, gate centered
in view (check a frame from a HAND run if unsure). `s5_surge_steps`:
surge bursts 20/35/50 % with coasts. Watch gate clearance — abort if a
collision looks likely. → surge thrust/drag via camera range-from-width AND
the bow-up pitch coupling (the #1 sim gap).

## S6. Sway steps at the gate (40 s LIVE)
Same placement, `s6_sway_steps` — alternating left/right sway bursts.

## S8. Deadband ramps (100 s LIVE) — USE THE TRIM VARIANT
`s8_deadband_ramps_trim` — very slow ramps per axis with the buoyancy
hold-down on all non-heave steps (the heave ramp itself stays pure). The sub
barely moves by design. Mid-pool, mid-depth. → motion-onset command per axis.

## S9. Roll authority (40 s LIVE — do this one with fresh battery)
`s9_roll_authority` — vertical-differential pulses ±40/±70/±100 with free
releases between. The sub will heel hard and maybe roll past 90° at ±100 —
that is the point. Mid-depth, clear of walls. → roll thrust authority vs the
righting moment; with S2 this fixes the style-roll strategy.
BUOYANCY NOTE: the releases must stay motors-off (free decay data), so the
sub WILL drift up across the run — re-submerge it between pulses if it nears
the surface; each pulse+release pair stands alone.

## S10. Vision filming (10 min, motors off or gentle hand-carry)
```
HAND=1 GRAY_WORLD=0 ./sysid/sysid_run.sh s10_vision_raw "gate+poles pass 1"
```
Hand-carry/float the sub slowly past the gate and poles: approaches from 1 m
to 5 m, oblique angles, half-gate views, near-surface glare, near-floor.
Repeat once with `GRAY_WORLD=1 … s10_vision_gw` (the nav's color pipeline).
→ offline vision refit corpus (real HSV ranges vs sim).

---

# AFTER EVERY SESSION
1. `ls ~/epsilon_docker/sysid/runs/` — one dir per run, check row counts look
   alive (`wc -l *.csv`).
2. Fill in every meta.yaml (venue/depth/battery/notes — 30 s each).
3. Nothing else. The model pulls the dirs, fits, and updates RESUME.md.
   Raw runs are the source of truth — never delete them.
