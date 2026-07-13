# PROCEDURE — known-depth calibration ritual (depth_scale)

The top OWED item (2026-07-13). The ESP32/MS5837 chain under-reads depth by an
estimated x1.2-1.4: at 0.8 indicated the hull sat near the floor of the 4.5 ft
(1.37 m) section. Until this is measured, floor-relative targets are dangerous.
Rules of the ritual (from RESUME):
- ONLY a static-hold + physical tape measurement counts. Eyeball estimates
  from above water were tried and retired — refraction reads ~25-30% shallow
  and the internally-inconsistent 1.4 value never deployed.
- depth_scale stays 1.0 until this procedure produces a number.
- Reference point is the SENSOR PORT, not the hull bottom.

## Equipment
Tape measure (rigid or weighted), a second person, phone camera, fresh battery,
retrieval pole. Know where the sensor port is on the hull before getting wet.

## Measurement runs
1. Float the sub >= 60 s (F14 wet baseline), then launch a holdtest at the
   deepest safe indicated target for the section:
   ```bash
   ROBOSUB_TEST_DEPTH=0.8 .devcontainer/holdtest.sh "depth cal point A, battery XX.XV"
   ```
2. During the 60 s hold, wait ~20-30 s for the integrator to settle, then
   measure: TAPE FROM THE WATERLINE STRAIGHT DOWN TO THE SENSOR PORT.
   Take 2-3 readings across the hold; film the tape. The hold is steady to
   ±3 cm indicated, so a careful reading is meaningful.
3. Record: indicated target (0.8), measured true depth, time into hold,
   battery V, water temp if known.
4. SECOND POINT (do not skip — separates scale from offset): repeat at a
   shallower target in the same section:
   ```bash
   ROBOSUB_TEST_DEPTH=0.5 .devcontainer/holdtest.sh "depth cal point B"
   ```
   Same measurement. (Two points let you fit true = a*indicated + b. A pure
   under-read gives b ~ 0; a significant b means a fixed offset — e.g.
   baseline capture error — which depth_scale CANNOT fix. If |b| > ~5 cm,
   stop: record both points and take it back to the desk instead of scaling.)

## Compute
- scale k = true / indicated at each point; expect the two to agree within
  ~5%. Prior expectation: k in 1.2-1.4. If the points disagree badly, see the
  offset note above.

## Apply (back at the desk, not poolside)
1. `depth_scale` lives in sensor_bridge (epsilon_bridge/sensor_bridge.py,
   ROS param, default 1.0; it multiplies the /depth republish that nav
   consumes). It is NOT yet plumbed as a launch argument in
   prequal.launch.py — add it to the sensor_bridge node's parameters there
   (mirroring how the other DeclareLaunchArgument knobs flow), with a
   `.pre-*` backup of the launch file. Verify at next launch: sensor_bridge
   logs "SCALED /depth x<k>" instead of "passthrough /depth".
2. KNOWN SIDE EFFECT: scaling depth by k also multiplies the depth-loop gain
   by k (the loop was tuned on raw depth). After applying, the FIRST water
   run is a plain holdtest at the same section, judged against the normal
   PASS bar. If the hold gets ringy or sluggish, the corrective is dividing
   depth_p / depth_i / depth_d by k in pid_params.yaml (one change, backup,
   re-run) — not new tuning.
3. Re-verify the calibration itself: hold at (old 0.8 target / k) indicated —
   the tape should now read ~the target. One point is enough for the recheck.
4. Recompute the safe-target table for every pool section with true floor
   depths, and update the values used in STRAIGHT_TEST_SMALL_POOL.md /
   HOLDTEST_90.md. The floor-grind failure mode (verticals pinned -1.00,
   settle gate never passes, human abort required) is still unguarded until
   the depth-unreachable valve lands in StabilizeTask.

## Bookkeeping
- meta.yaml on both cal runs: battery_v, water_depth_m (the measured truth!),
  notes with the tape readings.
- Log the measured k, both points, and the applied change in sysid/RESUME.md
  before ending the session. Note that water-1 fits / sim plant / depth gains
  were all built on RAW depth — the sim calibration keeps raw units; only the
  bridge output is scaled.
