# PROCEDURE — straight test in the SMALL pool

Closed-loop straight shot: settle at depth, drive forward at fixed surge while
compass holds the start heading and the depth loop holds depth. Verifies
stabilization UNDER forward motion. Last known-good run: 20260713-081327
(surge 0.8, 20 s, MISSION_COMPLETE).

## Known behavior going in (2026-07-13 water data — expect these, they are not new bugs)
- Sub parks ~ +7 deg RIGHT of the start line while thrusting (steady asymmetry
  torque vs P-only heading cascade). It will crab right of where the nose starts.
- Residual LEFT drift (per-motor fwd/rev thrust inequality). Root fix is the
  bench bollard test + per-motor compensation table — not yet done.
- Surge 0.3 is USELESS (corner PWM 24-35 sits in prop deadband, ~1 m in 20 s).
  Effective surge floor is ~0.5; 0.8 gives ~0.2 m/s (~3 m in 20 s).
- Depth sensor UNDER-READS ~x1.2-1.4 (until the depth calibration is done,
  every target below is INDICATED depth, and true depth is deeper).

## Small-pool constraints
- Deepest section is the 4.5 ft (1.37 m) stretch. At 0.8 indicated the hull was
  already near the floor there. NEVER set ROBOSUB_TEST_DEPTH above 0.8; in the
  shallow end use 0.5-0.6. A target below the physical floor pins the verticals
  at -1.00 and grinds the floor until you abort (run 080624 — the mission
  cannot self-recover; the depth-unreachable valve is still a TODO).
- Size the run to the pool: at surge 0.8 plan ~0.2 m/s. With L meters of clear
  water ahead, set ROBOSUB_STRAIGHT_S = (L - 1) / 0.2, rounded down.
  3 m clear -> 10 s. Do not use the 20 s default unless you have 5+ m.
- Aim allowance: start the nose pointed slightly LEFT of the intended lane
  (right droop) and leave lateral clearance on the LEFT (drift).

## Pre-run (every run)
1. FRESH battery. Note resting voltage — you will put it in meta.yaml
   (brownout killed run 081040 at launch; F12).
2. Kill switch reachable, retrieval pole at the pool edge (disarmed sub is not
   guaranteed to float up — F13).
3. Float the sub in the water >= 60 s BEFORE launching the stack (F14: the
   depth sensor reads a ~0.2 m temperature transient on air->water; the boot
   surface-pressure baseline must be captured wet).
4. Place the sub at the start end, nose down-lane (slightly left, see above).

## Launch
```bash
ssh robosub@192.168.0.123
docker exec -it robosub_dev bash
cd /home/robosub/robosub_ws
ROBOSUB_TEST_DEPTH=0.8 ROBOSUB_SURGE=0.8 ROBOSUB_STRAIGHT_S=10 \
  .devcontainer/straighttest.sh "small pool straight, battery XX.XV"
```
- The script preflights, then hands off to a DETACHED supervisor: readiness
  gate -> params gate (auto-refuses a stale install) -> mission start confirm
  -> 15 s arm countdown (ARM_DELAY=NN to change) -> arm confirm -> monitor.
- SSH/WiFi loss mid-run is safe: the run continues on the Pi and always ends
  disarmed (RUN_MAX 420 s hard cap).
- Mission timeline after arm: settle 15 s at depth -> drive STRAIGHT_S s at
  SURGE -> settle 5 s -> surface -> shutdown. Total ~40-50 s for a 10 s shot.

## Abort
- Connected: Ctrl-C in the launch shell = full abort (disarm + teardown).
- Reconnected after a drop:
  `ros2 run epsilon_bridge arming_helper disarm`   (instant motor kill)
  `pkill -TERM -f watertest_supervisor`            (full teardown)
- Sub grinding the floor / heading for the wall: disarm. Do not wait for the
  settle gate — it will not pass.

## After each run
1. Edit `sysid/runs/<ts>-straighttest-water/meta.yaml`: battery_v, water_depth_m,
   venue notes. Do it NOW, not at home.
2. Poolside sanity (in the run dir):
   - `tail supervisor.log` — want MISSION_COMPLETE, "THRUSTER ARMED (confirmed",
     no "PARAMS GATE FAILED".
   - CSV row counts printed at teardown — imu/attitude/depth_raw all nonzero
     and growing through the run (venue AP drops multicast; the lo_unicast DDS
     profile in watertest.sh handles it, but verify rows anyway).
3. Note by eye: how far it traveled, which way it veered, anything it hit.

## PASS bar (one clean rep = pass; 2-3 reps if time)
- MISSION_COMPLETE, zero valve (` T!`) states, arm confirmed on attempt 1-2.
- Depth held within ~±0.1 of target through the drive (depth_raw.csv).
- No visible yaw rocking; corner-motor duty from cmd.csv < 10% for yaw.
- Heading droop <= ~8 deg right and steady (matches the known defect); heading
  std about the droop < 2 deg. WORSE than that = new problem, stop and log.
- Forward progress consistent with ~0.2 m/s at surge 0.8.

## What this feeds
- The droop/drift numbers per run build the case file for the per-motor
  fwd/rev compensation table (bench bollard test, log battery_v).
- If the yaw stopgap (yaw feedforward ~ -0.175 * surge) gets implemented, this
  exact procedure re-verifies it: droop should drop from ~7 deg toward ~0-2.
