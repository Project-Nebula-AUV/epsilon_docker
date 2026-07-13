#!/bin/bash
# =============================================================================
# sysid_run.sh — one water/bench sysid run: launch stack -> (LIVE only:
# readiness gate -> countdown -> arm) -> wait for sequence end -> teardown.
#
#   DRY (default, NO motors -- omni_control not even started):
#       ./sysid/sysid_run.sh s3_heave_staircase
#     cmd.csv in the run dir is the verification artifact.
#
#   LIVE (motors; user present, sub placed, stand clear during countdown):
#       LIVE=1 ./sysid/sysid_run.sh s3_heave_staircase "pool, battery 16.1V"
#
#   HAND-TEST (S1/S2: sensors+logger only, motors impossible, Ctrl-C to end):
#       HAND=1 ./sysid/sysid_run.sh s2_tilt_release "roll releases x5, pitch x5"
#     Sub must be LEVEL at launch (roll offset captured in first 1.5 s).
#
# Env: LIVE=0|1  HAND=0|1  ARM_DELAY=15  READY_TIMEOUT=45  WITH_DEPTH=true
#      (ARM_DELAY default is generous: start on land, carry to the pool,
#       place, stand clear. Runs detached via docker exec -d, so WiFi loss
#       when the sub submerges cannot kill a run.)
#      WITH_CAMERA=true  JPEG_HZ=4.0  GRAY_WORLD=0
# Run inside the robosub_dev container. Ctrl-C at any point disarms + tears down.
# ABORT FROM ANOTHER SHELL:
#   ros2 service call /sysid_runner/arm std_srvs/srv/SetBool "{data: false}"
# =============================================================================
set -u

# 2026-07-07: SELF-DETACH. Root cause of tonight's failures found: PROTOCOL.md
# has you SSH in + 'docker exec -it' (an ATTACHED shell) to run this script.
# The sub's WiFi/SSH drops the moment it goes underwater -- that kills the
# attached shell via SIGHUP, which kills THIS SCRIPT mid-flight (not just one
# ros2 CLI call, as it looked like from the symptoms). The setsid-launched
# sensor/runner stack survives (own session), but the script driving the
# go-signal/countdown/teardown does not, so the runner just sits disarmed --
# no motors, no matter how the go-signal itself is sent.
# Fix: re-exec ourselves fully detached (own session, HUP ignored) on first
# invocation, regardless of whether you used -it or -d. The countdown/output
# you are used to seeing still streams live via 'tail -f' in THIS shell --
# Ctrl-C here still aborts while you are connected -- but the real run is
# now in a session nothing you do at the terminal (or losing WiFi) can kill.
if [ -z "${SYSID_DETACHED:-}" ]; then
  export SYSID_DETACHED=1
  LOG="/tmp/sysid_run.$(date +%Y%m%d-%H%M%S).log"
  echo "[sysid] detaching the real run into its own session (log: ${LOG})"
  echo "[sysid] WiFi/SSH loss from here on cannot stop it. Ctrl-C here still"
  echo "[sysid] aborts while connected; once disconnected, abort with:"
  echo "[sysid]   ros2 service call /sysid_runner/arm std_srvs/srv/SetBool "{data: false}""
  setsid nohup "$0" "$@" < /dev/null > "$LOG" 2>&1 &
  DETACHED_PID=$!
  disown
  trap 'kill -TERM -- -"$DETACHED_PID" 2>/dev/null || true' INT TERM
  tail -n +1 -f "$LOG" &
  TAIL_PID=$!
  wait "$DETACHED_PID" 2>/dev/null
  kill "$TAIL_PID" 2>/dev/null || true
  exit 0
fi

NAME="${1:-}"
NOTES="${2:-}"
LIVE="${LIVE:-0}"
HAND="${HAND:-0}"
ARM_DELAY="${ARM_DELAY:-15}"
READY_TIMEOUT="${READY_TIMEOUT:-45}"
WITH_DEPTH="${WITH_DEPTH:-true}"
WITH_CAMERA="${WITH_CAMERA:-true}"
JPEG_HZ="${JPEG_HZ:-4.0}"
GRAY_WORLD="${GRAY_WORLD:-0}"

WS=/home/robosub/robosub_ws
SEQ_DIR="${WS}/sysid/sequences"
RUNS_DIR="${WS}/sysid/runs"

if [ -z "$NAME" ]; then
  echo "usage: [LIVE=1|HAND=1] $0 <sequence-name|hand-label> [notes]"
  echo "sequences:"; ls "$SEQ_DIR" 2>/dev/null | sed 's/\.yaml$//;s/^/  /'
  exit 1
fi

SEQ_FILE="${SEQ_DIR}/${NAME}.yaml"
if [ "$HAND" != "1" ] && [ ! -f "$SEQ_FILE" ]; then
  echo "no sequence ${SEQ_FILE}"; ls "$SEQ_DIR" | sed 's/^/  /'; exit 1
fi

set +u   # ROS setup scripts reference unset guard vars
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

# WiFi-INDEPENDENT ROS (same fix as watertest.sh, 2026-07-09): force DDS
# discovery off the WiFi interface so it survives the sub submerging.
export ROS_LOCALHOST_ONLY=1
# DDS multicast intake freeze (2026-07-12, this venue 192.168.0.x): even with
# ROS_LOCALHOST_ONLY=1, loopback multicast discovery silently stopped IMU/
# depth/camera intake 10-15s into 4/5 bench runs here. Force loopback-UNICAST
# instead (verified 2x, 195s rate-perfect). No laptop-side ros2 echo/rviz
# visibility with this set -- watch RUN_DIR CSVs / this script's own log.
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/robosub/robosub_ws/sysid/lo_unicast.xml


MODE=dry; [ "$LIVE" = "1" ] && MODE=live; [ "$HAND" = "1" ] && MODE=hand
RUN_ID="$(date +%Y%m%d-%H%M%S)-${NAME}-${MODE}"
RUN_DIR="${RUNS_DIR}/${RUN_ID}"
mkdir -p "$RUN_DIR"

cat > "${RUN_DIR}/meta.yaml" <<EOF
run_id: ${RUN_ID}
sequence: ${NAME}
mode: ${MODE}
date: $(date -Iseconds)
venue: pool            # pool | bench | comp  (EDIT ME)
water_depth_m: 1.52    # EDIT ME
battery_v: null        # EDIT ME
notes: "${NOTES}"
EOF

GW_ARG=$([ "$GRAY_WORLD" = "1" ] && echo true || echo false)

echo "[sysid] clean slate: stopping any stale stack..."
for pat in "[o]mni_control" "epsilon_bridge/[s]ensor_bridge" "epsilon_bridge/[t]hruster_bridge" \
           "epsilon_bridge/[s]ysid_runner" "epsilon_bridge/[s]ysid_logger" \
           "epsilon_sensors/[c]amera" "epsilon_sensors/[i]mu" \
           "epsilon_sensors/[d]epth_sensor" "epsilon_sensors/[d]epth_fusion" "epsilon_sensors/[e]sp32_depth" \
           "[s]ubmarine_node" "[r]os2 launch epsilon_bridge"; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 2
sudo chmod a+rw /dev/video0 /dev/video1 /dev/ttyACM0 2>/dev/null || true

WITH_MOTORS=false; [ "$MODE" = "live" ] && WITH_MOTORS=true
WITH_RUNNER=true;  [ "$MODE" = "hand" ] && WITH_RUNNER=false
# in-node arming countdown (s): LIVE -> the runner self-arms after ARM_DELAY
# with NO post-submersion CLI (WiFi/SSH loss on submersion is then harmless).
# COUNTDOWN_S env overrides so the countdown path can be exercised in DRY.
CD_ARG=0.0; [ "$MODE" = "live" ] && CD_ARG="$ARM_DELAY"
CD_ARG="${COUNTDOWN_S:-$CD_ARG}"

STACK_PID=""
cleanup() {
  echo ""
  echo "[sysid] teardown: disarm + stop stack"
  [ "$WITH_RUNNER" = "true" ] && \
    timeout 5 ros2 service call /sysid_runner/arm std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
  # setsid below makes STACK_PID a session/process-group leader -- signal the
  # whole group so every launched node dies, not just the ros2 launch parent.
  [ -n "$STACK_PID" ] && kill -TERM -- -"$STACK_PID" 2>/dev/null || true
  sleep 2
  echo "[sysid] artifacts in ${RUN_DIR}:"
  for f in "$RUN_DIR"/*.csv; do
    [ -f "$f" ] && echo "  $(basename "$f"): $(( $(wc -l < "$f") - 1 )) rows"
  done
  [ -d "${RUN_DIR}/frames" ] && echo "  frames: $(ls "${RUN_DIR}/frames" | wc -l) jpgs"
}
trap cleanup INT TERM

echo "[sysid] launching (mode=${MODE}, run=${RUN_ID})"
# setsid: put ros2 launch (+ every node it spawns) in its own session, so a
# Ctrl-C on this script only interrupts the script -- not a fresh SIGINT to
# ros2 launch racing this trap's own disarm+teardown sequence.
setsid ros2 launch epsilon_bridge sysid.launch.py \
    run_dir:="${RUN_DIR}" sequence_file:="${SEQ_FILE}" \
    with_runner:="${WITH_RUNNER}" with_motors:="${WITH_MOTORS}" \
    with_depth:="${WITH_DEPTH}" with_camera:="${WITH_CAMERA}" \
    jpeg_hz:="${JPEG_HZ}" gray_world:="${GW_ARG}" countdown_s:="${CD_ARG}" &
STACK_PID=$!

if [ "$MODE" = "hand" ]; then
  echo "[sysid] HAND-TEST mode: logging until Ctrl-C. Do the maneuvers now."
  echo "        Drop a marker from another shell with:"
  echo "        ros2 topic pub --once /sysid/marker std_msgs/msg/String \"{data: 'tilt-release roll 1'}\""
  wait "$STACK_PID"
  cleanup
  exit 0
fi

have_topic() { timeout -k 2 "${2:-5}" ros2 topic echo --once --qos-reliability best_effort "$1" >/dev/null 2>&1; }

# Fire the ONE-SHOT start trigger. -w 1 waits for the runner's subscription to
# match before publishing, so delivery is guaranteed. This is the ONLY external
# command a run needs; after it the node counts down + self-arms internally,
# immune to the WiFi/SSH loss that submersion causes.
fire_go() {
  local a
  for a in 1 2 3; do
    if timeout 15 ros2 topic pub --once -w 1 /sysid_runner/go std_msgs/msg/Empty "{}" >/dev/null 2>&1; then
      return 0
    fi
    echo "[sysid] go-trigger attempt ${a} failed, retrying..."; sleep 1
  done
  return 1
}

if [ "$MODE" = "live" ]; then
  REQUIRED=("/imu:IMU" "/sensors/heading:sensor_bridge" "/sysid/status:runner")
  [ "$WITH_DEPTH" = "true" ] && REQUIRED+=("/sensors/depth:fused depth")
  [ "$WITH_CAMERA" = "true" ] && REQUIRED+=("/camera/image_raw:camera")
  echo "[sysid] readiness gate (up to ${READY_TIMEOUT}s)..."
  ready=0; SECONDS=0
  while [ "$SECONDS" -lt "$READY_TIMEOUT" ]; do
    missing=""
    for item in "${REQUIRED[@]}"; do
      have_topic "${item%%:*}" 4 || missing="${missing}  - ${item#*:} (${item%%:*})\n"
    done
    [ -z "$missing" ] && { ready=1; break; }
    kill -0 "$STACK_PID" 2>/dev/null || { echo "[sysid] launch died."; break; }
  done
  if [ "$ready" -ne 1 ]; then
    echo "[sysid] READINESS FAILED after ${SECONDS}s. Missing:"; echo -e "$missing"
    cleanup; exit 1
  fi
  echo "[sysid] stack healthy on land."
  echo "[sysid] triggering the in-node ${ARM_DELAY}s countdown NOW (WiFi still up)..."
  if ! fire_go; then
    echo "[sysid] go-trigger FAILED on land -- aborting (sub is NOT armed)"; cleanup; exit 1
  fi
  cat <<BANNER

[sysid] ====================================================================
[sysid]  RUNNER IS COUNTING DOWN ${ARM_DELAY}s, THEN SELF-ARMS ONBOARD.
[sysid]  Place the sub + STAND CLEAR. Losing WiFi/SSH from here is SAFE --
[sysid]  the sequence + logging run entirely onboard, no further commands.
[sysid]  Logs -> ${RUN_DIR}
[sysid]  ABORT before it arms (while still connected):
[sysid]    ros2 service call /sysid_runner/arm std_srvs/srv/SetBool "{data: false}"
[sysid] ====================================================================
BANNER
  # Leave the stack running: setsid gave it its own session/pgroup and a normal
  # exit does NOT fire the INT/TERM trap, so the onboard run survives this
  # script (and the SSH session) going away.
  exit 0
fi

# ---------- DRY only from here (bench; no submersion; observe to completion) --
sleep 8   # let the stack come up
echo "[sysid] >>> DRY: firing /sysid_runner/go (no motors) <<<"
if ! fire_go; then
  echo "[sysid] go-trigger FAILED"; cleanup; exit 1
fi

echo "[sysid] waiting for sequence end..."
DEADLINE=$((SECONDS + 300))
RESULT=""
while [ "$SECONDS" -lt "$DEADLINE" ]; do
  S=$(timeout -k 2 4 ros2 topic echo --once /sysid/status 2>/dev/null | grep -oP "(?<=data: ).*" | tr -d "'\"" )
  case "$S" in
    done|aborted) RESULT="$S"; break ;;
  esac
  kill -0 "$STACK_PID" 2>/dev/null || { echo "[sysid] launch died mid-run."; break; }
done
echo "[sysid] sequence result: ${RESULT:-TIMEOUT/UNKNOWN}"
cleanup
[ "$RESULT" = "done" ] && exit 0 || exit 2
