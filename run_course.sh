#!/bin/bash
# Canonical sim test runner. Live view on http://<host>:8765 for every run.
#
#   ./run_course.sh [mode] [duration_s]
#     mode: full (default) | gate | slalom | orbit | hold
#
#   Env:
#     HW_FAITHFUL=1        the hardware sensing contract (2026-07-06): ESP32
#                          depth-chain emulation (~7 Hz filtered, 20 Hz ZOH)
#                          + zeroed x/y velocity. No fusion node involved.
#     LEGACY_FUSED=1       the OLD hw mode: emulated marginal MS5837-on-Pi
#                          + depth_fusion (kept runnable for regression work)
#     VENUE=pool|comp      world depth 1.52|5.8 m; pool also defaults
#                          ROBOSUB_MISSION_DEPTH to 0.8
#     STYLE_ROLL=<deg>     barrel roll before the outbound gate (default 720
#                          for full/gate, 0 otherwise)
#     ROBOSUB_SIM_SEED, ROBOSUB_START_X/Y/HDG   course/start overrides
#
# Sensor cadence/noise (18.2 Hz IMU etc.) comes from sysid/sim_calibration.yaml
# in ALL modes once present — the sim is never easier than the vehicle.
#
# After the run: python3 judge.py /tmp/sim_truth.csv /tmp/sim_geom.json
source /opt/ros/humble/setup.bash
source /home/robosub/robosub_ws/install/setup.bash
export SDL_VIDEODRIVER=dummy
cd /home/robosub/robosub_ws

MODE="${1:-full}"
DUR="${2:-600}"
HW_FAITHFUL="${HW_FAITHFUL:-0}"

case "$MODE" in
  full)   unset ROBOSUB_MISSION; export ROBOSUB_STYLE_ROLL="${STYLE_ROLL:-720}" ;;
  gate)   export ROBOSUB_MISSION=gate; export ROBOSUB_STYLE_ROLL="${STYLE_ROLL:-720}" ;;
  slalom) export ROBOSUB_MISSION=slalom; export ROBOSUB_STYLE_ROLL=0 ;;
  orbit)  export ROBOSUB_MISSION=orbit; export ROBOSUB_STYLE_ROLL="${STYLE_ROLL:-0}" ;;
  hold)   export ROBOSUB_MISSION=hold; export ROBOSUB_STYLE_ROLL=0 ;;
  holdtest) export ROBOSUB_MISSION=holdtest; export ROBOSUB_STYLE_ROLL=0 ;;
  rolltest) export ROBOSUB_MISSION=rolltest; export ROBOSUB_STYLE_ROLL=0 ;;
  *) echo "unknown mode: $MODE"; exit 1 ;;
esac

for p in "[s]im_gui_node" "[s]imulator_node" "[s]ubmarine_node" "[d]epth_fusion" \
         "[c]ourse_monitor" "[g]ui_screenshot_runner"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 1

if [ -n "${VENUE:-}" ]; then
  export ROBOSUB_VENUE="$VENUE"
  if [ "$VENUE" = "pool" ]; then
    # 1.52 m water: cruise at 0.8, gate-opening center ~1.0 (bar assumed 0.5)
    [ -z "${ROBOSUB_MISSION_DEPTH:-}" ] && export ROBOSUB_MISSION_DEPTH=0.8
    [ -z "${ROBOSUB_GATE_DEPTH:-}" ] && export ROBOSUB_GATE_DEPTH=1.0
  fi
  echo "[run] venue=$VENUE mission_depth=${ROBOSUB_MISSION_DEPTH:-1.5} gate_depth=${ROBOSUB_GATE_DEPTH:-1.55}"
fi

SIM_ARGS=""
FUS=""
if [ "${LEGACY_FUSED:-0}" = "1" ]; then
  echo "[run] LEGACY fused sensing (emulated marginal MS5837 + depth_fusion)"
  SIM_ARGS="--ros-args -p fuse_depth:=true -p hw_velocity:=true"
  ros2 run epsilon_sensors depth_fusion --ros-args \
      -r /depth:=/sensors/depth -p world_z_sign:=-1.0 > /tmp/run_fus.log 2>&1 &
  FUS=$!
elif [ "$HW_FAITHFUL" = "1" ]; then
  echo "[run] HW-FAITHFUL sensing (ESP32 depth-chain emulation + zeroed lateral velocity)"
  # W6 2026-07-07: HW-faithful now ALSO means the measured actuation
  # (quadratic thrust curves). EPSILON_PLANT=0 forces the legacy linear map.
  export ROBOSUB_EPSILON_PLANT="${EPSILON_PLANT:-1}"
  echo "[run] epsilon-plant actuation: $ROBOSUB_EPSILON_PLANT"
  SIM_ARGS="--ros-args -p depth_mode:=esp32 -p hw_velocity:=true"
fi

ros2 run robosub sim_gui_node $SIM_ARGS > /tmp/run_sim.log 2>&1 &
SIM=$!
PYTHONUNBUFFERED=1 ros2 run robosub submarine_node > /tmp/run_sub.log 2>&1 &
SUB=$!
sleep 8

# Publish start until the mission actually leaves WAITING (the nodes boot at
# unpredictable speed on the Pi and a single early publish can fly past a
# subscriber that isn't up yet). The sim's truth log mirrors /sub/status, so
# it is the cheap race-free readiness check.
for i in $(seq 1 30); do
  ros2 topic pub --once /sim/control std_msgs/msg/String "data: 'start'" > /dev/null
  sleep 2
  if tail -1 /tmp/sim_truth.csv 2>/dev/null | grep -qvE "WAITING|Waiting|^t,"; then
    break
  fi
done
echo "[run] mode=$MODE hw_faithful=$HW_FAITHFUL started (attempt $i) — watch http://epsilon:8765"

python3 /home/robosub/robosub_ws/course_monitor.py "$DUR"
RC=$?

kill -9 $SIM $SUB $FUS 2>/dev/null
for p in "[s]im_gui_node" "[s]ubmarine_node" "[d]epth_fusion"; do
  pkill -9 -f "$p" 2>/dev/null
done

echo "=== submarine log tail ==="
grep -E "INFO:|WARN:|ERROR" /tmp/run_sub.log | tail -15
echo "=== judge ==="
python3 /home/robosub/robosub_ws/judge.py /tmp/sim_truth.csv /tmp/sim_geom.json
exit $RC
