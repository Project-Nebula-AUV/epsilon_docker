#!/bin/bash
# =============================================================================
# watertest.sh — WATER SESSION 1 checkout runner (holdtest / rolltest / gate)
#
# Same hardened bring-up as motortest.sh (clean slate -> preflight -> stack
# disarmed -> readiness gate -> countdown -> arm) PLUS:
#   * sysid_logger records the whole run to sysid/runs/<ts>-<test>-water/
#     (imu/depth/attitude/cmd/temp csvs + camera frames) — the data is the
#     product even when a test fails.
#   * NO_ARM=1 bench mode: full stack + mission sequences, but the vehicle is
#     NEVER armed (thruster_bridge gates on arm state -> zero motor output).
#     Use for dry rehearsal of these scripts out of water.
#
# Usage (inside robosub_dev, any dir):
#   TEST=hold  .devcontainer/watertest.sh    # 60 s depth hold @1.2 m + 90 turn
#   TEST=roll  .devcontainer/watertest.sh    # settle -> 360 style roll -> hold
#   TEST=gate  .devcontainer/watertest.sh    # StabilizeTask -> GateTask (no roll)
# Wrappers: holdtest.sh / rolltest.sh / gatetest.sh (same dir).
#
#   ABORT: Ctrl-C -> disarm + teardown.   From another shell:
#          ros2 run epsilon_bridge arming_helper disarm
#
# Knobs: ROBOSUB_TEST_DEPTH (1.2), ARM_DELAY (15), NO_ARM (0), STYLE_ROLL
# (gate test forces 0), MISSION_DEPTH/GATE depth envs pass through.
# =============================================================================

TEST="${TEST:-hold}"
case "$TEST" in
  hold) MISSION_MODE=holdtest ; STYLE_ROLL="${STYLE_ROLL:-0}" ;;
  roll) MISSION_MODE=rolltest ; STYLE_ROLL="${STYLE_ROLL:-0}" ;;
  gate) MISSION_MODE=gate     ; STYLE_ROLL="${STYLE_ROLL:-0}" ;;
  *) echo "TEST must be hold|roll|gate"; exit 1 ;;
esac

HEAVE_BIAS="${HEAVE_BIAS:-0.0}"
WITH_DEPTH="${WITH_DEPTH:-true}"
SYNTHETIC_DEPTH="${SYNTHETIC_DEPTH:--1.0}"
WORLD_Z_SIGN="${WORLD_Z_SIGN:-1.0}"
ARM_DELAY="${ARM_DELAY:-15}"
GRAY_WORLD="${GRAY_WORLD:-1}"
READY_TIMEOUT="${READY_TIMEOUT:-45}"
NO_ARM="${NO_ARM:-0}"
NOTES="${1:-}"

GW_ARG=$([ "$GRAY_WORLD" = "1" ] && echo true || echo false)
WS=/home/robosub/robosub_ws
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"

RUN_ID="$(date +%Y%m%d-%H%M%S)-${TEST}test-water"
RUN_DIR="${WS}/sysid/runs/${RUN_ID}"

# ── 0. clean slate (bracket patterns: never self-match) ─────────────────────
echo "[watertest] clean slate..."
for pat in "[o]mni_control" "epsilon_bridge/[s]ensor_bridge" "epsilon_bridge/[t]hruster_bridge" \
           "epsilon_bridge/[a]rming_helper" "epsilon_sensors/[c]amera" "epsilon_sensors/[i]mu" \
           "epsilon_sensors/[d]epth_sensor" "epsilon_sensors/[d]epth_fusion" "epsilon_sensors/[e]sp32_depth" \
           "[s]ubmarine_node" "[r]ecord_video" "epsilon_bridge/[s]ysid_logger" "epsilon_bridge/[s]ysid_runner" \
           "[r]os2 launch epsilon_bridge"; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 2
sudo chmod a+rw /dev/video0 /dev/video1 /dev/ttyACM0 2>/dev/null || true
# Hard-killed nodes leave stale FastDDS shared-memory segments that BREAK
# DISCOVERY for new nodes (every readiness probe reports missing while the
# stack is healthy — bit us 2026-07-09). Safe here: everything ROS was just
# pkilled. Also bounce the CLI daemon (same failure mode, cheaper cause).
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps* 2>/dev/null || true
ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1; sleep 2

# ── 1. preflight (same checks as motortest.sh) ──────────────────────────────
preflight_fail=0
check() {
  local label="$1"; shift
  if "$@" >/dev/null 2>&1; then echo "[preflight]  OK   ${label}"
  else
    echo "[preflight] FAIL  ${label}"
    [ -n "${LAST_HINT:-}" ] && echo "             fix: ${LAST_HINT}"
    preflight_fail=1
  fi
}
LAST_HINT="reinstall WiringPi: cd /tmp && git clone --depth 1 https://github.com/WiringPi/WiringPi && cd WiringPi && ./build && sudo ldconfig"
check "WiringPi linked into omni_control" bash -c "ldd ${WS}/build/epsilon_control/omni_control 2>/dev/null | grep -qi wiring"
LAST_HINT="sudo rm -f /usr/include/python3.10/numpy"
check "numpy header symlink absent (cv_bridge safe)" bash -c "[ ! -e /usr/include/python3.10/numpy ]"
LAST_HINT="colcon build --packages-select epsilon_control"
check "omni_control executable present" bash -c "ros2 pkg executables epsilon_control 2>/dev/null | grep -q omni_control"
LAST_HINT="check the USB camera (re-enumerates as /dev/video0)"
check "/dev/video0 present + readable" bash -c "[ -r /dev/video0 ]"
LAST_HINT="free disk: docker builder prune -af"
check "disk has >=2 GB free" bash -c "[ \"\$(df --output=avail -BG / | tail -1 | tr -dc 0-9)\" -ge 2 ]"
if [ "$preflight_fail" -ne 0 ]; then
  echo "[watertest] PREFLIGHT FAILED — not launching."
  exit 1
fi

# ── 2. launch stack DISARMED + logger ───────────────────────────────────────
export ROBOSUB_MISSION="$MISSION_MODE"
export ROBOSUB_STYLE_ROLL="$STYLE_ROLL"

STACK_PID=""; LOGGER_PID=""
cleanup() {
  echo ""
  echo "[watertest] teardown -> disarm + stop stack + stop logger"
  ros2 run epsilon_bridge arming_helper disarm 2>/dev/null || true
  [ -n "$STACK_PID" ] && kill "$STACK_PID" 2>/dev/null || true
  sleep 1
  [ -n "$LOGGER_PID" ] && kill -INT "$LOGGER_PID" 2>/dev/null || true
  sleep 2
  [ -n "$LOGGER_PID" ] && kill "$LOGGER_PID" 2>/dev/null || true
  echo "[watertest] run data: ${RUN_DIR}"
}
trap cleanup INT TERM

mkdir -p "$RUN_DIR"
cat > "${RUN_DIR}/meta.yaml" <<EOF
run_id: ${RUN_ID}
sequence: watertest-${TEST}
mode: water-checkout
date: $(date -Iseconds)
venue: pool            # EDIT ME
water_depth_m: null    # EDIT ME
battery_v: null        # EDIT ME
no_arm: ${NO_ARM}
test_depth_m: ${ROBOSUB_TEST_DEPTH:-1.2}
notes: "${NOTES}"
EOF

echo "[watertest] launching stack (disarmed): test=${TEST} mission=${MISSION_MODE} style_roll=${STYLE_ROLL} no_arm=${NO_ARM}"
ros2 launch epsilon_bridge prequal.launch.py \
    with_depth:="${WITH_DEPTH}" synthetic_depth:="${SYNTHETIC_DEPTH}" \
    world_z_sign:="${WORLD_Z_SIGN}" heave_bias:="${HEAVE_BIAS}" gray_world:="${GW_ARG}" &
STACK_PID=$!

ros2 run epsilon_bridge sysid_logger --ros-args -p run_dir:="${RUN_DIR}" > "${RUN_DIR}/logger.out" 2>&1 &
LOGGER_PID=$!

# ── 3. readiness gate ───────────────────────────────────────────────────────
have_topic() { timeout -k 2 "${2:-5}" ros2 topic echo --once --qos-reliability best_effort "$1" >/dev/null 2>&1; }
REQUIRED=(
  "/camera/image_raw:camera frames"
  "/imu:IMU"
  "/sensors/heading:sensor_bridge heading"
  "/sensors/depth:depth (fused or synthetic)"
  "/sub/status:nav brain alive"
)
echo "[watertest] waiting up to ${READY_TIMEOUT}s for a healthy stack..."
ready=0; SECONDS=0
while [ "$SECONDS" -lt "$READY_TIMEOUT" ]; do
  missing=""
  for item in "${REQUIRED[@]}"; do
    topic="${item%%:*}"; label="${item#*:}"
    have_topic "$topic" 4 || missing="${missing}  - ${label} (${topic})\n"
  done
  [ -z "$missing" ] && { ready=1; break; }
  kill -0 "$STACK_PID" 2>/dev/null || { echo "[watertest] launch died."; break; }
done
if [ "$ready" -ne 1 ]; then
  echo "[watertest] READINESS GATE FAILED after ${SECONDS}s. Missing:"
  echo -e "$missing"
  cleanup
  exit 1
fi
echo "[watertest] stack healthy."

# ── 4. countdown -> arm (unless NO_ARM) ─────────────────────────────────────
if [ "$NO_ARM" = "1" ]; then
  echo "[watertest] NO_ARM=1: bench mode — mission runs, vehicle stays DISARMED."
else
  echo "[watertest] ARMING in ${ARM_DELAY}s. Submerge + stand clear. Ctrl-C aborts."
  for ((i=ARM_DELAY; i>0; i--)); do
    echo "[watertest]   arming in ${i}s..."
    sleep 1
  done
  echo "[watertest] >>> ARMING + STARTING (${TEST}) <<<"
  ros2 run epsilon_bridge arming_helper arm
fi

wait "$STACK_PID"
cleanup
