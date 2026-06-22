#!/bin/bash
# =============================================================================
# Epsilon PREQUAL auto-run  (replaces the old thruster_tester motortest)
#
# Brings up the FULL autonomous stack, waits 10 s so the operator can submerge
# the sub and stand clear, then ARMS and STARTS the mission. The sub runs the
# pre-qual mission WITHOUT a depth sensor: sensor_bridge feeds a synthetic depth
# (= MISSION_DEPTH) so the depth loop holds neutral, and it is started already
# underwater. The nav searches (yaw-scan) for the gate, drives through it, and
# attempts the rest of the mission.
#
#   START:  ./.devcontainer/motortest.sh            (run inside robosub_dev)
#   ABORT:  Ctrl-C  -> disarms thrusters + tears the stack down
#   DISARM FROM ANOTHER SHELL:
#           ros2 run epsilon_bridge arming_helper disarm
#
# Buoyancy trim: the sub is ~1 N positively buoyant, so with HEAVE_BIAS=0.0 it
# will slowly rise (the fail-safe). To hold depth, raise HEAVE_BIAS toward ~0.5
# (start small, e.g. 0.3) once you have watched a run. +heave = descend.
# =============================================================================
set -u

HEAVE_BIAS="${HEAVE_BIAS:-0.0}"             # 0.0 = neutral/surfaces; ~0.5 = hold vs buoyancy
SYNTHETIC_DEPTH="${SYNTHETIC_DEPTH:-1.5}"   # must equal MISSION_DEPTH in robosub/mission.py
ARM_DELAY="${ARM_DELAY:-10}"                # seconds between stack-up and arming

source /opt/ros/humble/setup.bash
source /home/robosub/robosub_ws/install/setup.bash

# The USB camera node (/dev/video0,1) needs the host 'video' group, which the
# container user is not in -> open the device nodes (container sudo is passwordless).
sudo chmod a+rw /dev/video0 /dev/video1 2>/dev/null || true

STACK_PID=""
cleanup() {
  echo ""
  echo "[prequal] aborting -> disarm + stop stack"
  ros2 run epsilon_bridge arming_helper disarm 2>/dev/null || true
  [ -n "$STACK_PID" ] && kill "$STACK_PID" 2>/dev/null || true
}
trap cleanup INT TERM

# Bring up sensors + bridges (thruster_bridge DISARMED) + omni_control + nav brain.
echo "[prequal] launching full stack (disarmed): synthetic_depth=${SYNTHETIC_DEPTH} heave_bias=${HEAVE_BIAS}"
ros2 launch epsilon_bridge prequal.launch.py \
    synthetic_depth:="${SYNTHETIC_DEPTH}" heave_bias:="${HEAVE_BIAS}" &
STACK_PID=$!

# Countdown: submerge the sub and stand clear. Thrusters are still disarmed here.
echo "[prequal] stack coming up; ARMING in ${ARM_DELAY}s. Ctrl-C to abort."
for ((i=ARM_DELAY; i>0; i--)); do
  echo "[prequal]   arming in ${i}s..."
  sleep 1
done

# Arm: publishes 'start' to /sim/control (nav begins) AND arms thruster_bridge
# (motors go live). arming_helper orders these safely.
echo "[prequal] >>> ARMING + STARTING MISSION <<<"
ros2 run epsilon_bridge arming_helper arm

# Hold here running the stack until it exits or the operator aborts (Ctrl-C).
wait "$STACK_PID"
