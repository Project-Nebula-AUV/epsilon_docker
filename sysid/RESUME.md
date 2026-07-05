# SYSID RESUME — canonical state. Update BEFORE ending every session.

STATUS: PLANNED. EXECUTION NOT STARTED. (Plan approved 2026-07-05; see PLAN.md here.)

## Session ritual (mandatory)
1. Read: auto-memory `handoff-sysid` → this file → PLAN.md.
2. SPOT-VERIFY one prior claim before building on it.
3. Execute the queue below, top-down. Water runs: USER executes, arm-gated, countdown,
   fail-safe-to-surface. No git commits — `.pre-*` timestamped backups. Never
   `devcontainer up` over SSH.
4. Update this file + the `handoff-sysid` memory before ending.

## Audit state (W0)
- [ ] sensor_bridge: physical hand-motion sign/axis verification (roll, heading, PITCH channel)
- [ ] sensor_bridge: rates/latency/level-capture verified
- [ ] thruster_bridge: inverse-mix math verified analytically vs sim mixer
- [ ] omni_control + launches + motortest path: config/arming audit
- Findings ledger: (empty — nothing audited yet)

## Runs collected (sysid/runs/)
(none)

## Fits accepted (sysid/fits/ + sim_calibration.yaml)
(none — sim_calibration.yaml does not exist yet)

## Queue — Session A (audit + build)
1. W0 audit (above) — fits on unaudited sensing are void.
2. W1: build `sysid_runner` + logger in epsilon_bridge; DRY-verify every sequence
   (omni_control NOT running, echo /thrust_control).
3. W2 bench numbers: mass, dims, thruster positions/angles (photos), IMU 10-min rest,
   camera FOV check in air.
4. PROTOCOL.md v1 (water-test menu S1–S10, operator steps, exact commands).
5. If water same-day: user does S1 (static trim) + S2 (tilt-release roll AND pitch —
   motors off, just IMU logging) — highest info, zero script risk.

## Queue — later sessions
- Session B: S1–S4 + S7 (+S10 filming) → pull logs → fit vertical/yaw/thruster matrix
  → fusion verify+fix (near-zero consensus) → first sim_calibration.yaml → pitch DOF
  + epsilon-plant mode into sim.
- Session C: S5/S6/S8/S9 → complete fits → nav re-tune (+ thruster_bridge feedforward
  linearization) → style-roll strategy WITH NUMBERS (direct vs pump; sub already
  ballasted for roll) → W7 closed-loop physical verify + sim overlay.
- Session D: residual-gap ranking, Opus handoff, PROTOCOL.md final.

## Inviolables
Disarmed default · arm service gate · watchdog-to-zero · countdown before power ·
fail-safe-to-surface · user executes all water runs · no commits · no devcontainer up.
