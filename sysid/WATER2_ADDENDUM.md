# WATER SESSION 2 ADDENDUM (2026-07-07, W6 session)
# Separate file because PROTOCOL.md had a live vim .swp — fold in later.
# PROTOCOL.md Part B S5/S6/S8/S9/S10 steps still apply as written; this adds
# the water-1 lessons + two NEW sequences.

## Before getting wet
1. F13 DECIDED (user 2026-07-07): pool-floor failures are ACCEPTED — have the
   retrieval plan ready (pole/line/swimmer). No foam/purge for now. At comp
   depths a disarmed sub sinks; revisit before comp.
2. LAUNCH RITUAL CHANGE (F14): float the sub in the water >= 60 s BEFORE
   starting the stack, so the ESP32 surface baseline captures WET at water
   temperature (the air->water transient reads ~0.2-0.25 m for ~30 s).
   This fits inside the 90 s ARM_DELAY workflow.
3. Measure the ACTUAL pool depth (F15): tape or lower the sub on a line and
   read /depth after a 60 s settle. Water 1 says the floor is ~2.0-2.1 m of
   sensor depth, not the 1.52 m the venue config assumes.
4. Fresh battery (F12: full-power corner thrust browned out the Pi at 15.x V;
   note battery_v in every meta.yaml).

## Run list (in this order; all LIVE except where marked)
- S5 surge steps, S6 sway steps  — PROTOCOL as written. These also settle the
  yaw-arm question (effective arm 0.133-0.151 => corner thrust lines ~55-63
  deg toed OR weaker corner props; surge/sway magnitude decides which).
- S8 deadband ramps — USE THE TRIM VARIANT, start MID-WATER (~1 m): water 1
  showed the sub settles to the floor at rest trim; a floor-resting deadband
  ramp measures friction, not deadband.
- S9 roll authority — PROTOCOL as written. Expect direct stall ~50 deg at 80.
- S9b roll pump (NEW, sequences/s9b_roll_pump.yaml): open-loop square-wave
  vertical differential at 0.45/0.48/0.53 Hz. THE decisive question: do the
  swings GROW past 90 deg (high-rate roll drag ~linear => the nav pump
  completes full rotations) or plateau ~70 deg (quadratic => style roll needs
  another plan)? Watch /sensors/roll live if wifi holds.
- S2b at-depth releases (NEW, sequences/s2b_depth_release.yaml): replaces the
  surface S2 (waterplane physics made it unfittable). Start mid-water ~1 m.
- S10 vision filming — PROTOCOL as written; NOTE the hardware camera is
  640x320 and nav thresholds are now resolution-relative, so S10 footage
  feeds the vision refit directly.

## What changed on the vehicle since water 1 (all .pre-20260707-w6 backups)
- pid_params.yaml RE-TUNED for the calibrated plant (buoyancy_ff 0.61->0.12,
  depth_i_clamp 1.2->2.0, depth_p 2.5->3.0). Sim-verified; water 2 is the
  hardware closed-loop check (motortest.sh hold first, before any course).
- StyleRollSubtask = resonance pump (bang-bang in phase with roll rate,
  commit-to-rotation at >=2.4 rad/s near upright, un-commit + re-pump on
  stall, 45 s timeout still completes -> mission never stranded).
- esp32_depth now publishes /esp32_depth/temperature; sysid logger writes
  temp.csv (F14 diagnosis).
- Sim: epsilon-plant actuation (quadratic thrust curves), camera 320x160
  (hardware 2:1 aspect / 41 deg VFOV), 2026 gate divider RED + black/red
  maker boxes, gate pairing scans all candidate pairs.
