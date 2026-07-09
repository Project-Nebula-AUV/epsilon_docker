import math
import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from std_srvs.srv import SetBool
from ament_index_python.packages import get_package_share_directory

COS45 = 0.7071  # MUST match the sim mix constant in robosub/sub/control.py


class ThrusterBridge(Node):
    """Invert the robosub sim thruster mix and re-allocate to epsilon's 6 thrusters.

    Sub /thruster_commands (Float32MultiArray [hfl,hfr,hal,har,vp,vs], -1..1,
    BEST_EFFORT to match submarine_node) -> recover the 5-DOF wrench
    [surge,sway,heave,yaw,roll] (direction-preserving: the sim normalizes each
    thruster group uniformly) -> apply epsilon's allocation A (6x5, allocation.yaml)
    -> *100 -> saturate +-100 -> pub /thrust_control (Float64MultiArray[6], [t0..t5])
    for epsilon_control::omni_control (idx 0,1 = vertical; 2..5 = 45deg corners).

    Safety: watchdog zeroes output if no fresh input within watchdog_timeout and on
    shutdown; arming gate (~/arm SetBool) forces zero when disarmed (default disarmed).
    heave_sign / roll_sign are the two undetermined bits (flip at P4).
    """

    def __init__(self):
        super().__init__('thruster_bridge')
        self.watchdog_timeout = float(self.declare_parameter('watchdog_timeout', 0.1).value)
        self.heave_sign = float(self.declare_parameter('heave_sign', 1.0).value)
        self.roll_sign = float(self.declare_parameter('roll_sign', 1.0).value)
        # Constant buoyancy-trim added to the reconstructed heave wrench (nav-heave
        # convention: +heave = descend). Replaces the steady descend thrust the depth
        # integrator normally provides (~0.6) when running WITHOUT a depth sensor and
        # holding depth open-loop. 0.0 = neutral (sub slowly surfaces, the fail-safe);
        # increase toward ~+0.5 in-water to hold against positive buoyancy.
        self.heave_bias = float(self.declare_parameter('heave_bias', 0.0).value)
        self.armed = bool(self.declare_parameter('start_armed', False).value)
        # SOFT-START slew limit (%% per second, per thruster). Commands RAMP to
        # target instead of stepping -> limits the current inrush that sagged
        # the 5V rail and browned-out the Pi (2026-07-09: instant-100%% -> crash).
        # 150 %%/s = 0->full in ~0.67 s. Disarm/watchdog still zero INSTANTLY.
        self.slew_rate = float(self.declare_parameter('slew_rate', 150.0).value)
        self._last_out = [0.0] * 6
        self._last_out_t = None
        alloc_path = str(self.declare_parameter('allocation_file', '').value)

        self.A = self._load_allocation(alloc_path)  # A[t][axis], 6 x 5
        self.last_input_t = None

        qos_in = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(Float64MultiArray, '/thrust_control', 10)
        self.create_subscription(Float32MultiArray, '/thruster_commands', self.on_cmd, qos_in)
        self.create_service(SetBool, '~/arm', self.on_arm)
        self.create_timer(0.02, self.on_watchdog)  # 50 Hz watchdog tick
        self._zero()
        self.get_logger().info(
            'thruster_bridge up (armed=%s, watchdog=%.3fs, heave_sign=%g, roll_sign=%g, heave_bias=%g)'
            % (self.armed, self.watchdog_timeout, self.heave_sign, self.roll_sign, self.heave_bias))

    def _load_allocation(self, path):
        if not path:
            path = os.path.join(get_package_share_directory('epsilon_bridge'),
                                'config', 'allocation.yaml')
        with open(path) as f:
            cols = yaml.safe_load(f)['allocation']
        order = ['surge', 'sway', 'heave', 'yaw', 'roll']
        A = [[float(cols[axis][t]) for axis in order] for t in range(6)]
        self.get_logger().info('allocation loaded from %s' % path)
        return A

    def _zero(self):
        self.pub.publish(Float64MultiArray(data=[0.0] * 6))
        self._last_out = [0.0] * 6          # so the next ramp starts from rest
        self._last_out_t = None

    def on_cmd(self, msg):
        d = list(msg.data)
        if len(d) < 6:
            self.get_logger().warn('thruster_commands had %d elts (<6), ignoring' % len(d))
            return
        hfl, hfr, hal, har, vp, vs = d[0], d[1], d[2], d[3], d[4], d[5]
        # invert the sim mix -> wrench (heave/roll signs are the two P4 unknowns).
        # heave_bias adds a constant descend trim (nav-heave convention) for
        # depth-sensorless open-loop hold; heave_sign maps it to the right motor sign.
        heave = self.heave_sign * ((vp + vs) / 2.0 + self.heave_bias)
        roll = self.roll_sign * (vp - vs) / 2.0
        surge = (hfl + hfr + hal + har) / (4.0 * COS45)
        sway = ((hfl + har) - (hfr + hal)) / 4.0 / COS45
        yaw = ((hfl + hal) - (hfr + har)) / 4.0
        wrench = (surge, sway, heave, yaw, roll)
        out = []
        for t in range(6):
            v = sum(self.A[t][a] * wrench[a] for a in range(5)) * 100.0
            out.append(max(-100.0, min(100.0, v)))
        now = self.get_clock().now()
        self.last_input_t = now
        if not self.armed:
            self._zero()
            return
        # Soft-start: ramp each thruster toward its target, capped at slew_rate.
        if self._last_out_t is None:
            dt = 0.02
        else:
            dt = (now - self._last_out_t).nanoseconds * 1e-9
            dt = min(max(dt, 1e-3), 0.2)     # guard against big/backward gaps
        max_step = self.slew_rate * dt        # %% of full per tick
        ramped = []
        for t in range(6):
            delta = out[t] - self._last_out[t]
            if delta > max_step:
                delta = max_step
            elif delta < -max_step:
                delta = -max_step
            ramped.append(self._last_out[t] + delta)
        self._last_out = ramped
        self._last_out_t = now
        self.pub.publish(Float64MultiArray(data=ramped))

    def on_watchdog(self):
        if self.last_input_t is None:
            return
        dt = (self.get_clock().now() - self.last_input_t).nanoseconds * 1e-9
        if dt > self.watchdog_timeout:
            self._zero()

    def on_arm(self, req, resp):
        self.armed = bool(req.data)
        if not self.armed:
            self._zero()
        resp.success = True
        resp.message = 'armed' if self.armed else 'disarmed'
        self.get_logger().info('arm -> %s' % resp.message)
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._zero()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
