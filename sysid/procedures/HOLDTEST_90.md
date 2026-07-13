# PROCEDURE — holdtest + 90 degree turn

THE checkout gate: depth hold -> +90 deg compass turn -> re-hold -> surface.
Run it first at every water session; nothing else runs until it passes.
Last known-good: 20260713-075059 — MISSION_COMPLETE 75 s, heading std 0.6 deg,
roll std 0.6 deg, depth 0.81 ± 0.03 (indicated), yaw duty 0%.

## Mission sequence (mission.py `holdtest`)
Settle + hold 60 s at ROBOSUB_TEST_DEPTH -> +90 deg turn (TurnToHeading,
holds current depth through the turn — the 0.1 m inject bug is fixed) ->
8 s re-hold -> surface -> shutdown. ~75-85 s armed time when healthy.

## Depth target
- Default is 1.2 m — TOO DEEP for the small pool. Always set it explicitly.
- Small pool: ROBOSUB_TEST_DEPTH=0.8 in the 4.5 ft section, 0.5-0.6 in the
  shallow end. Remember: INDICATED depth; sensor under-reads ~x1.2-1.4 until
  the depth calibration lands, so the hull sits deeper than the number.

## Pre-run
1. Fresh battery, note resting V (goes in meta.yaml after the run).
2. Kill switch + retrieval pole staged (F13: disarmed sub can sink).
3. Float the sub >= 60 s before launching the stack (F14 wet baseline).
4. Water calm; film from the side — the video is half the evidence.

## Launch
```bash
ssh robosub@192.168.0.123
docker exec -it robosub_dev bash
cd /home/robosub/robosub_ws
ROBOSUB_TEST_DEPTH=0.8 .devcontainer/holdtest.sh "small pool holdtest, battery XX.XV"
```
Detached supervisor: preflight -> readiness gate -> params gate (refuses a
stale install — if it trips, rebuild with
`colcon build --symlink-install --packages-select robosub epsilon_bridge`
after deleting any real-dir copies in install/.../site-packages, then rerun)
-> mission start confirm -> 15 s countdown -> arm confirm -> monitor.
WiFi loss mid-run is safe; RUN_MAX 420 s hard cap; always ends disarmed.

## Abort
Ctrl-C while connected. After a drop:
`ros2 run epsilon_bridge arming_helper disarm` then
`pkill -TERM -f watertest_supervisor`.

## Expected picture (post-yaw-fix baseline — deviations are findings)
- Initial dive overshoots a few cm then the integrator bleeds it back over
  ~50 s (e.g. 0.93 -> 0.81). This slow "up drift" is EXPECTED, not a defect.
- Visually: dead quiet. No yaw rocking, no roll rocking, no sideways drift.
- Turn: starts after the 60 s hold, parks within ±3 deg, depth held through it.

## PASS bar
- MISSION_COMPLETE in supervisor.log, ~75-90 s armed, zero ` T!` valves.
- Depth within ±10 cm of target for the whole 60 s hold (by eye + depth_raw.csv).
- Heading std < 1.5 deg during holds; turn settles within ±3 deg of +90.
- Roll std < 2.5 deg; vertical differential quiet.
- Corner-motor yaw duty < 10% (cmd.csv) — the limit-cycle tell if it regresses.
FAIL handling: one clean re-run to rule out a fluke; still failing -> stop
water work, keep the logs, analyze at the desk. A failed run with a log is
still a win — do not tune poolside.

## After the run
1. meta.yaml: battery_v, water_depth_m, notes. Immediately.
2. supervisor.log tail: MISSION_COMPLETE + armed-confirm + CSV row counts.
3. This run doubles as the DEPTH CALIBRATION measuring window — if the tape
   measure is staged, take the depth measurement during the 60 s hold
   (see DEPTH_CALIBRATION.md).
