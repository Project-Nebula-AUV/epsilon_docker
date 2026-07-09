#!/bin/bash
# =============================================================================
# watertest.sh — WATER SESSION checkout runner (holdtest / rolltest / gatetest)
#
# DISCONNECT-PROOF (2026-07-09, same pattern as sysid_run.sh): this script
# only does preflight + clean slate, then hands off to a DETACHED supervisor
# (own session, log in the run dir) that runs countdown -> arm -> mission ->
# monitor -> disarm + teardown entirely on the Pi. Submerging (= guaranteed
# WiFi/SSH loss) cannot stop or strand the run; it ALWAYS ends disarmed —
# at MISSION_COMPLETE/FAILED, stack death, or the RUN_MAX hard cap.
#
# Usage (inside robosub_dev, any dir):
#   TEST=hold  .devcontainer/watertest.sh    # 60 s depth hold @1.2 m + 90 turn
#   TEST=roll  .devcontainer/watertest.sh    # settle -> 360 style roll -> hold
#   TEST=gate  .devcontainer/watertest.sh    # StabilizeTask -> GateTask (no roll)
# Wrappers: holdtest.sh / rolltest.sh / gatetest.sh (same dir).
#
#   Ctrl-C while connected = full abort (disarm + teardown).
#   After an SSH drop the run continues; reconnect and:
#     tail -f <run_dir>/supervisor.log        watch it
#     ros2 run epsilon_bridge arming_helper disarm     instant motor kill
#     pkill -TERM -f watertest_supervisor              full teardown
#
# Knobs: ROBOSUB_TEST_DEPTH (1.2), ARM_DELAY (15), NO_ARM (0), RUN_MAX (420 s
# hard cap armed->teardown), STYLE_ROLL, WITH_DEPTH, SYNTHETIC_DEPTH,
# HEAVE_BIAS, GRAY_WORLD, READY_TIMEOUT.
# =============================================================================

TEST="${TEST:-hold}"
case "$TEST" in
  hold) MISSION_MODE=holdtest ; STYLE_ROLL="${STYLE_ROLL:-0}" ;;
  roll) MISSION_MODE=rolltest ; STYLE_ROLL="${STYLE_ROLL:-0}" ;;
  gate) MISSION_MODE=gate     ; STYLE_ROLL="${STYLE_ROLL:-0}" ;;
  *) echo "TEST must be hold|roll|gate"; exit 1 ;;
esac
NOTES="${1:-}"

WS=/home/robosub/robosub_ws
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

# WiFi-INDEPENDENT ROS: the whole stack runs on the Pi, so force DDS onto
# loopback only. Otherwise DDS binds discovery to the WiFi interface and when
# the sub SUBMERGES (WiFi drops) intra-Pi service discovery STALLS -> the arm
# service call sits pending and only completes when the sub is recovered and
# WiFi returns (2026-07-09: "motors spin the moment it reconnects to WiFi").
export ROS_LOCALHOST_ONLY=1

RUN_ID="$(date +%Y%m%d-%H%M%S)-${TEST}test-water"
RUN_DIR="${WS}/sysid/runs/${RUN_ID}"

# ── 0. clean slate ──────────────────────────────────────────────────────────
echo "[watertest] clean slate..."
for pat in "[o]mni_control" "epsilon_bridge/[s]ensor_bridge" "epsilon_bridge/[t]hruster_bridge" \
           "epsilon_bridge/[a]rming_helper" "epsilon_sensors/[c]amera" "epsilon_sensors/[i]mu" \
           "epsilon_sensors/[d]epth_sensor" "epsilon_sensors/[d]epth_fusion" "epsilon_sensors/[e]sp32_depth" \
           "[s]ubmarine_node" "[r]ecord_video" "epsilon_bridge/[s]ysid_logger" "epsilon_bridge/[s]ysid_runner" \
           "[w]atertest_supervisor" "[r]os2 launch epsilon_bridge"; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 2
# Belt-and-braces: a SIGKILLed launch orphans its children and the patterns
# above can miss renamed cmdlines — sweep ANY leftover ROS node (2026-07-09:
# two stacks fought over /dev/video0 + the I2C IMU; gate failed confusingly).
if pgrep -f -- "--ros-args" >/dev/null 2>&1; then
  echo "[watertest] killing $(pgrep -cf -- "--ros-args") orphaned ROS node(s)"
  pkill -9 -f -- "--ros-args" 2>/dev/null || true
  sleep 2
fi
sudo chmod a+rw /dev/video0 /dev/video1 /dev/ttyACM0 2>/dev/null || true
# Hard-killed nodes leave stale FastDDS shared-memory segments that BREAK
# DISCOVERY for new nodes (every readiness probe reports missing while the
# stack is healthy — bit us 2026-07-09). Safe here: everything ROS was just
# pkilled. Also bounce the CLI daemon (same failure mode, cheaper cause).
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps* 2>/dev/null || true
ros2 daemon stop >/dev/null 2>&1; sleep 1; ros2 daemon start >/dev/null 2>&1; sleep 2

# ── 1. preflight ────────────────────────────────────────────────────────────
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

# ── 2. run dir + meta, then DETACHED supervisor ─────────────────────────────
mkdir -p "$RUN_DIR"
cat > "${RUN_DIR}/meta.yaml" <<EOF
run_id: ${RUN_ID}
sequence: watertest-${TEST}
mode: water-checkout
date: $(date -Iseconds)
venue: pool            # EDIT ME
water_depth_m: null    # EDIT ME
battery_v: null        # EDIT ME
no_arm: ${NO_ARM:-0}
test_depth_m: ${ROBOSUB_TEST_DEPTH:-1.2}
notes: "${NOTES}"
EOF

GW_ARG=$([ "${GRAY_WORLD:-1}" = "1" ] && echo true || echo false)

export RUN_DIR TEST MISSION_MODE STYLE_ROLL GW_ARG
export WITH_DEPTH="${WITH_DEPTH:-true}" SYNTHETIC_DEPTH="${SYNTHETIC_DEPTH:--1.0}"
export WORLD_Z_SIGN="${WORLD_Z_SIGN:-1.0}" HEAVE_BIAS="${HEAVE_BIAS:-0.0}"
export NO_ARM="${NO_ARM:-0}" ARM_DELAY="${ARM_DELAY:-15}"
export READY_TIMEOUT="${READY_TIMEOUT:-45}" RUN_MAX="${RUN_MAX:-420}"
export ROBOSUB_TEST_DEPTH="${ROBOSUB_TEST_DEPTH:-1.2}"
export ROBOSUB_MISSION_DEPTH="${ROBOSUB_MISSION_DEPTH:-1.2}"
export ROBOSUB_GATE_DEPTH="${ROBOSUB_GATE_DEPTH:-1.2}"

SUP_LOG="${RUN_DIR}/supervisor.log"
setsid bash "$(dirname "$0")/watertest_supervisor.sh" > "$SUP_LOG" 2>&1 < /dev/null &
SUP_PID=$!
echo "[watertest] supervisor detached (pid ${SUP_PID}, session-immune to SSH loss)"
echo "[watertest] log: ${SUP_LOG}"
echo "[watertest] Ctrl-C here = FULL ABORT while connected. If SSH drops, the"
echo "[watertest] run continues + self-terminates disarmed (RUN_MAX ${RUN_MAX}s cap)."

on_abort() {
  echo ""
  echo "[watertest] ABORT -> signalling supervisor (disarm + teardown)"
  kill -TERM -- -"$SUP_PID" 2>/dev/null || kill -TERM "$SUP_PID" 2>/dev/null || true
  sleep 6
  exit 130
}
trap on_abort INT TERM

# Live view; tail dying (SSH loss) does not touch the supervisor.
tail -f --pid="$SUP_PID" "$SUP_LOG" 2>/dev/null &
TAIL_PID=$!
while kill -0 "$SUP_PID" 2>/dev/null; do sleep 1; done
sleep 1
kill "$TAIL_PID" 2>/dev/null
echo "[watertest] supervisor finished — see ${SUP_LOG}"
