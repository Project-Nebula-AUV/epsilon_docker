#!/bin/bash
# =============================================================================
# watertest_supervisor.sh — the DETACHED half of watertest.sh. Runs in its own
# session on the Pi: launch stack -> readiness gate -> countdown -> arm ->
# monitor to MISSION_COMPLETE/FAILED or RUN_MAX -> disarm + teardown.
# WiFi/SSH loss cannot stop or strand it; the run ALWAYS ends disarmed.
#
# Do not run directly — watertest.sh spawns it (setsid) and tails its log.
# Abort from anywhere (reconnected shell):
#   ros2 run epsilon_bridge arming_helper disarm     (instant motor kill)
#   pkill -TERM -f watertest_supervisor              (full teardown)
# =============================================================================
WS=/home/robosub/robosub_ws
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

# All knobs arrive via environment from watertest.sh:
: "${RUN_DIR:?}" "${TEST:?}" "${MISSION_MODE:?}"
STYLE_ROLL="${STYLE_ROLL:-0}"
WITH_DEPTH="${WITH_DEPTH:-true}"
SYNTHETIC_DEPTH="${SYNTHETIC_DEPTH:--1.0}"
WORLD_Z_SIGN="${WORLD_Z_SIGN:-1.0}"
HEAVE_BIAS="${HEAVE_BIAS:-0.0}"
GW_ARG="${GW_ARG:-true}"
NO_ARM="${NO_ARM:-0}"
ARM_DELAY="${ARM_DELAY:-15}"
READY_TIMEOUT="${READY_TIMEOUT:-45}"
RUN_MAX="${RUN_MAX:-420}"     # hard cap (s) from arm to forced teardown

export ROBOSUB_MISSION="$MISSION_MODE"
export ROBOSUB_STYLE_ROLL="$STYLE_ROLL"

log() { echo "[supervisor $(date +%H:%M:%S)] $*"; }

STACK_PID=""; LOGGER_PID=""; STATUS_PID=""; FINISHED=0
cleanup() {
  [ "$FINISHED" = "1" ] && return
  FINISHED=1
  log "teardown: disarm + stop stack + stop logger"
  timeout 20 ros2 run epsilon_bridge arming_helper disarm >/dev/null 2>&1 \
    || timeout 10 ros2 service call /thruster_bridge/arm std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 \
    || true
  [ -n "$STATUS_PID" ] && kill -TERM -- -"$STATUS_PID" 2>/dev/null || true
  [ -n "$STACK_PID" ] && kill -TERM -- -"$STACK_PID" 2>/dev/null || true
  sleep 2
  [ -n "$LOGGER_PID" ] && kill -INT "$LOGGER_PID" 2>/dev/null || true
  sleep 2
  [ -n "$LOGGER_PID" ] && kill "$LOGGER_PID" 2>/dev/null || true
  [ -n "$STACK_PID" ] && kill -KILL -- -"$STACK_PID" 2>/dev/null || true
  # ros2 run forks the node as a child that outlives the wrapper PID (the
  # logger survived a group kill, 2026-07-09) — reap by name, then sweep any
  # orphan so the run ends with ZERO ros nodes (next run starts clean).
  pkill -9 -f "epsilon_bridge/sysid_logger" 2>/dev/null || true
  sleep 1
  pkill -9 -f -- "--ros-args" 2>/dev/null || true
  log "run data: ${RUN_DIR}"
  for f in "$RUN_DIR"/*.csv; do
    [ -f "$f" ] && log "  $(basename "$f"): $(( $(wc -l < "$f") - 1 )) rows"
  done
  [ -d "${RUN_DIR}/frames" ] && log "  frames: $(ls "${RUN_DIR}/frames" | wc -l) jpgs"
  log "DONE (vehicle disarmed)"
}
trap cleanup INT TERM EXIT

log "launching stack (disarmed): test=${TEST} mission=${MISSION_MODE} style_roll=${STYLE_ROLL} no_arm=${NO_ARM} run_max=${RUN_MAX}s"
setsid ros2 launch epsilon_bridge prequal.launch.py \
    with_depth:="${WITH_DEPTH}" synthetic_depth:="${SYNTHETIC_DEPTH}" \
    world_z_sign:="${WORLD_Z_SIGN}" heave_bias:="${HEAVE_BIAS}" gray_world:="${GW_ARG}" \
    > "${RUN_DIR}/launch.log" 2>&1 &
STACK_PID=$!

ros2 run epsilon_bridge sysid_logger --ros-args -p run_dir:="${RUN_DIR}" \
    > "${RUN_DIR}/logger.out" 2>&1 &
LOGGER_PID=$!

# Readiness = ACTUAL DATA FLOW + node liveness, NOT `ros2 topic echo` probes.
# Those short-lived CLI probes are unreliable at DDS discovery on this Pi and
# failed even when the stack was fully healthy (2026-07-09: the logger, a real
# persistent node, captured 83 IMU rows while every CLI probe reported the
# topic missing). The logger IS the honest sensor witness — if its CSVs and
# frame dir are growing, the sensors + camera are genuinely live, discovery or
# not. Node liveness (nav + arm target) comes from pgrep. All discovery-free.
rows() { [ -f "$1" ] && wc -l < "$1" 2>/dev/null || echo 0; }
frames_n() { ls "${RUN_DIR}/frames" 2>/dev/null | wc -l; }
node_up() { pgrep -f "$1" >/dev/null 2>&1; }

log "waiting up to ${READY_TIMEOUT}s for a healthy stack (data-flow gate)..."
ready=0; SECONDS=0
imu0=$(rows "${RUN_DIR}/imu.csv"); att0=$(rows "${RUN_DIR}/attitude.csv")
dep0=$(rows "${RUN_DIR}/depth_raw.csv"); frm0=$(frames_n)
while [ "$SECONDS" -lt "$READY_TIMEOUT" ]; do
  sleep 3
  missing=""
  [ "$(rows "${RUN_DIR}/imu.csv")"      -gt "$imu0" ] || missing="${missing}  - IMU data not flowing (imu.csv)\n"
  [ "$(rows "${RUN_DIR}/attitude.csv")" -gt "$att0" ] || missing="${missing}  - heading/roll not flowing (attitude.csv)\n"
  [ "$(frames_n)"                       -gt "$frm0" ] || missing="${missing}  - camera frames not flowing (frames/)\n"
  # depth: real chain grows depth_raw.csv; synthetic-depth mode won't, so only
  # require it when using the real sensor.
  if [ "${SYNTHETIC_DEPTH}" = "-1.0" ]; then
    [ "$(rows "${RUN_DIR}/depth_raw.csv")" -gt "$dep0" ] || missing="${missing}  - depth not flowing (depth_raw.csv)\n"
  fi
  node_up "[s]ubmarine_node"            || missing="${missing}  - nav brain not running (submarine_node)\n"
  node_up "epsilon_bridge/[t]hruster_bridge" || missing="${missing}  - thruster_bridge (arm target) not running\n"
  [ -z "$missing" ] && { ready=1; break; }
  kill -0 "$STACK_PID" 2>/dev/null || { log "launch died."; break; }
done
if [ "$ready" -ne 1 ]; then
  log "READINESS GATE FAILED after ${SECONDS}s. Missing:"
  echo -e "$missing"
  exit 1
fi
log "stack healthy (sensors flowing + nav/thruster nodes up)."

# Persistent /sub/status subscriber -> file, for discovery-free completion
# detection (a long-lived subscriber discovers reliably where the short-lived
# `echo --once` probe does not). The monitor greps this file, never the bus.
STATUS_FILE="${RUN_DIR}/status.stream"
setsid ros2 topic echo --qos-reliability best_effort /sub/status \
    > "$STATUS_FILE" 2>/dev/null < /dev/null &
STATUS_PID=$!

if [ "$NO_ARM" = "1" ]; then
  log "NO_ARM=1: bench mode — mission runs DISARMED for ${RUN_MAX}s max."
else
  log "ARMING in ${ARM_DELAY}s. Submerge + stand clear. (SSH loss is now safe.)"
  for ((i=ARM_DELAY; i>0; i--)); do
    log "  arming in ${i}s..."
    sleep 1
  done
  armed_ok=0
  for attempt in 1 2 3; do
    if timeout 40 ros2 run epsilon_bridge arming_helper arm 2>&1 | grep -q "NOT changed"; then
      log "arm attempt ${attempt} failed; retrying..."
      sleep 2
    else
      armed_ok=1
      log ">>> ARMED — mission running (${TEST}) <<<"
      break
    fi
  done
  if [ "$armed_ok" -ne 1 ]; then
    log "!!! ARM FAILED 3x — tearing down (vehicle was never armed)."
    exit 1
  fi
fi

# ── monitor: end on MISSION_COMPLETE/FAILED, dead stack, or RUN_MAX ─────────
SECONDS=0
while [ "$SECONDS" -lt "$RUN_MAX" ]; do
  kill -0 "$STACK_PID" 2>/dev/null || { log "stack exited."; break; }
  if grep -q "MISSION_COMPLETE" "$STATUS_FILE" 2>/dev/null; then
    log "MISSION_COMPLETE (t=${SECONDS}s)"; break
  fi
  if grep -q "MISSION_FAILED" "$STATUS_FILE" 2>/dev/null; then
    log "MISSION_FAILED (t=${SECONDS}s)"; break
  fi
  sleep 3
done
[ "$SECONDS" -ge "$RUN_MAX" ] && log "RUN_MAX ${RUN_MAX}s reached — forcing teardown."
exit 0
