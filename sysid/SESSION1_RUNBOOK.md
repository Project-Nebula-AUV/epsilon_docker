# WATER SESSION 1 RUNBOOK — checkout + vision data (bright pool)

Goal: leave the pool with (a) usable gate/gatelet footage, (b) the three
never-tested loops verified in water: depth hold, heading turn, style roll,
(c) stretch: one gate approach. Everything is logged automatically — a failed
test with a log is still a win. **Budget ~90 min. Do the blocks IN ORDER.**

Before leaving home: charge battery (note resting V), pack gate + all 3
gatelet pairs + tape measure + phone (video) + retrieval pole (F13!).

Every powered launch below: sub floating in the water ≥60 s first (F14 depth
baseline), then run the command INSIDE the container:
  ssh robosub@<pi> ; docker exec -it robosub_dev bash ; cd /home/robosub/robosub_ws

Abort any run: Ctrl-C in the launch shell (disarms), or from a second shell:
  ros2 run epsilon_bridge arming_helper disarm

─────────────────────────────────────────────────────────────────────────────
BLOCK 0 — setup (~15 min, water calm for Block 1)
─────────────────────────────────────────────────────────────────────────────
- Place the GATE mid-pool, orange divider up, at ~1.5 m water depth if it
  stands; note actual depth of the top bar.
- Place ONE gatelet pair (white-red-white) ~2-3 m past it.
- Note battery V: ______   pool water depth: ______   time: ______

─────────────────────────────────────────────────────────────────────────────
BLOCK 1 — S10 vision re-shoot (NO motors, hand-carry, ~20 min)
─────────────────────────────────────────────────────────────────────────────
  HAND=1 ./sysid/sysid_run.sh s10_vision_raw "bright pool reshoot"
Hold the sub level at ~0.5-0.8 m depth, camera forward. Walk it SLOWLY
(2-3 s per position). SHOT LIST — check off:
  [ ] gate head-on at 4 m, 3 m, 2 m, 1 m   (pause ~3 s each)
  [ ] gate from ~30 deg left, ~3 m; same from right
  [ ] gate half-passage: carry the sub through the right half
  [ ] gatelet pair head-on at 3 m, 2 m, 1 m
  [ ] gatelet pair from ~30 deg, 2 m
  [ ] slow sweep past gate at cruise depth (simulates search yaw-scan)
  [ ] 30 s of open water / pool wall / far end  (false-positive material)
  [ ] 10 s pointing at the water surface glare (worst case)
Ctrl-C to end. POOLSIDE CHECK before packing the props: open 3 jpgs from
sysid/runs/<latest>/frames/ — posts clearly red/visible at 3 m? If NO:
re-shoot closer/different angle; footage is this block's only product.

─────────────────────────────────────────────────────────────────────────────
BLOCK 2 — holdtest (~10 min incl. reps) ── THE GATE FOR EVERYTHING ELSE
─────────────────────────────────────────────────────────────────────────────
  .devcontainer/holdtest.sh "session1"        (add notes arg freely)
Sequence: 15 s countdown → dive to 1.2 m → hold 60 s → turn +90° → hold 8 s
→ surface → shutdown. Watch from the side; film it.
PASS = depth visually steady (±10 cm) through the minute + one clean turn.
  battery V after: ______    verdict: ______
If it porpoises/oscillates: run once more (log is gold), then SKIP the depth-
dependent stretch goal (Block 4) and spend the time on more Block-1 footage.
If it sinks/surfaces steadily: same — logs tell me the integrator trend.

─────────────────────────────────────────────────────────────────────────────
BLOCK 3 — rolltest (~10 min)
─────────────────────────────────────────────────────────────────────────────
  .devcontainer/rolltest.sh "session1"
Sequence: dive to 1.2 m (20 s settle) → ONE 360° style roll → re-level →
re-hold 8 s → surface. Film it (this is also the style-points demo reel).
PASS = completes the rotation (any # of pump swings) and re-levels ±30° → holds.
  battery V after: ______    verdict: ______
Roll stalls part-way and times out = still fine (it self-levels, mission-safe);
the log decides whether we bump pump timing before session 2.

─────────────────────────────────────────────────────────────────────────────
BLOCK 4 — gatetest (STRETCH, ~15 min, only if Block 2 passed)
─────────────────────────────────────────────────────────────────────────────
Place the sub ~3-4 m from the gate, roughly facing it.
  ROBOSUB_TEST_DEPTH=1.0 ROBOSUB_GATE_DEPTH=1.0 .devcontainer/gatetest.sh "session1"
(no style roll in this one — roll was Block 3)
PASS = acquires the gate, centers, drives through the right half. 2-3 reps
if time allows. Partial success is EXPECTED (vision tuning comes after this
session's footage) — retrieve, note what it did, rerun.
  battery V after: ______    verdict: ______

─────────────────────────────────────────────────────────────────────────────
PACK-UP (do not skip)
─────────────────────────────────────────────────────────────────────────────
- STOP every run cleanly (Ctrl-C) BEFORE powering off — the S9 null-byte
  tails came from a power cut mid-log.
- Note final battery V + anything weird (aeration, drift, WiFi drops).
- Back home: just power the Pi; I pull all runs, analyze, and prep session 2.

Poolside tuning knobs (only if a block is unusable as-is):
- Sub too buoyant/heavy in hold: HEAVE_BIAS=±0.05..0.1 on the test command.
- Depth sensor acting up: WITH_DEPTH=false SYNTHETIC_DEPTH=1.2 (holdtest
  becomes open-loop — abandon Block 2 pass, still run Blocks 1/3).
- Camera tint wrong in frames: GRAY_WORLD=0 (raw) for one Block-1 rerun.
