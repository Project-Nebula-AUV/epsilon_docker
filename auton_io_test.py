#!/usr/bin/env python3
"""
auton_io_test.py — theoretical input/output verification of the epsilon nav stack.

Injects KNOWN synthetic sensor inputs and a synthetic camera frame into the LIVE
submarine_node + thruster_bridge, and checks the resulting /thruster_commands
(nav) and /thrust_control (after re-allocation, heave_sign=-1) carry the
theoretically-correct correction on each controllable axis. No omni_control runs,
so no motors move.

Run (inside robosub_dev, ROS sourced):
  # start the two nodes under test first (no omni_control):
  ROBOSUB_MISSION=hold ros2 run robosub submarine_node &
  ros2 run epsilon_bridge thruster_bridge --ros-args -p start_armed:=true \
        -p heave_sign:=-1.0 -p heave_bias:=0.0 &
  python3 auton_io_test.py            # writes verdict to auton_io_result.txt

Hold mode locks target_heading on the first tick (we hold heading=90 during
warm-up), target_depth=1.5, target_roll=0. Only yaw + roll + depth are
closed-loop here (no DVL); each case perturbs one axis and we assert the sign.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Float32MultiArray, Float64MultiArray, String
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist

RESULT_FILE = "auton_io_result.txt"

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=1)
REL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST, depth=10)

NEUTRAL = dict(heading=90.0, roll=0.0, depth=1.5, gyro_x=0.0, gyro_z=0.0,
               vx=0.0, vy=0.0, vz=0.0)


class TestNode(Node):
    def __init__(self):
        super().__init__("auton_io_test")
        # injected sensor state (mutated by the sequence)
        self.s = dict(NEUTRAL)

        # publishers (match submarine_node's subscribed QoS)
        self.pub_cam = self.create_publisher(Image, "/camera/image_raw", REL)
        self.pub_hd = self.create_publisher(Float32, "/sensors/heading", BE)
        self.pub_rl = self.create_publisher(Float32, "/sensors/roll", BE)
        self.pub_dp = self.create_publisher(Float32, "/sensors/depth", BE)
        self.pub_im = self.create_publisher(Imu, "/sensors/imu", BE)
        self.pub_vel = self.create_publisher(Twist, "/sensors/velocity", BE)
        self.pub_ctl = self.create_publisher(String, "/sim/control", REL)

        # subscribers to the outputs under test
        self.tc = None    # /thruster_commands  [hfl,hfr,hal,har,vp,vs]
        self.tcl = None   # /thrust_control     [t0..t5]
        self.create_subscription(Float32MultiArray, "/thruster_commands",
                                 self._on_tc, BE)
        self.create_subscription(Float64MultiArray, "/thrust_control",
                                 self._on_tcl, BE)

        # a valid 320x240 bgr8 mid-gray frame (content irrelevant to StabilizeTask)
        self._img = Image()
        self._img.height = 240
        self._img.width = 320
        self._img.encoding = "bgr8"
        self._img.is_bigendian = 0
        self._img.step = 320 * 3
        self._img.data = bytes([128]) * (320 * 240 * 3)

    def _on_tc(self, m):
        self.tc = list(m.data)

    def _on_tcl(self, m):
        self.tcl = list(m.data)

    def publish_all(self):
        self._img.header.stamp = self.get_clock().now().to_msg()
        self.pub_cam.publish(self._img)
        self.pub_hd.publish(Float32(data=float(self.s["heading"])))
        self.pub_rl.publish(Float32(data=float(self.s["roll"])))
        self.pub_dp.publish(Float32(data=float(self.s["depth"])))
        imu = Imu()
        imu.angular_velocity.x = float(self.s["gyro_x"])
        imu.angular_velocity.z = float(self.s["gyro_z"])
        imu.orientation.w = 1.0
        self.pub_im.publish(imu)
        tw = Twist()
        tw.linear.x = float(self.s["vx"])
        tw.linear.y = float(self.s["vy"])
        tw.linear.z = float(self.s["vz"])
        self.pub_vel.publish(tw)

    def start_mission(self):
        self.pub_ctl.publish(String(data="start"))


def step(node, duration, record=False):
    """Publish inputs and spin for `duration` s. If record, average the outputs."""
    t_end = time.time() + duration
    tc_acc, tcl_acc, n = [0.0] * 6, [0.0] * 6, 0
    while time.time() < t_end:
        node.publish_all()
        for _ in range(4):
            rclpy.spin_once(node, timeout_sec=0.0)
        if record and node.tc is not None and node.tcl is not None \
                and len(node.tc) >= 6 and len(node.tcl) >= 6:
            for i in range(6):
                tc_acc[i] += node.tc[i]
                tcl_acc[i] += node.tcl[i]
            n += 1
        time.sleep(0.02)
    if record and n:
        return [x / n for x in tc_acc], [x / n for x in tcl_acc], n
    return None, None, n


def measure(node, overrides):
    """Apply overrides, settle, then record averaged outputs; return (tc, tcl, n)."""
    node.s = dict(NEUTRAL)
    node.s.update(overrides)
    step(node, 0.8)                 # settle
    tc, tcl, n = step(node, 0.6, record=True)
    node.s = dict(NEUTRAL)
    step(node, 0.5)                 # relax between cases
    return tc, tcl, n


def main():
    rclpy.init()
    node = TestNode()
    lines = []

    def log(msg):
        print(msg)
        lines.append(msg)

    # ── warm-up: hold neutral so heading=90 locks as the target ──
    node.s = dict(NEUTRAL)
    step(node, 1.5)
    node.start_mission()
    step(node, 1.2)

    cases = [
        ("neutral",       {}),
        ("heading_high",  {"heading": 120.0}),   # err = 90-120 = -30 -> yaw_n<0
        ("heading_low",   {"heading": 60.0}),    # err = 90-60 = +30 -> yaw_n>0
        ("roll_pos",      {"roll": 1.0}),        # roll_err=-1 -> roll_n<0
        ("roll_neg",      {"roll": -1.0}),       # roll_err=+1 -> roll_n>0
        ("depth_shallow", {"depth": 1.2}),       # err +0.3 -> heave_n>0 (descend)
        ("depth_deep",    {"depth": 1.8}),       # err -0.3 -> heave_n<0 (ascend)
        ("gyro_yaw",      {"gyro_z": 0.3}),      # yaw_n = -0.9 (strong damping)
        ("gyro_roll",     {"gyro_x": 0.3}),      # roll_n = -0.3 (roll damping)
    ]

    res = {}
    for name, ov in cases:
        tc, tcl, n = measure(node, ov)
        res[name] = (tc, tcl)
        if tc is None:
            log(f"[{name}] NO OUTPUT RECEIVED (n=0)  <-- stack not producing!")
        else:
            log(f"[{name}] n={n}")
            log(f"    thruster_commands [hfl,hfr,hal,har,vp,vs] = "
                + ", ".join(f"{x:+.3f}" for x in tc))
            log(f"    thrust_control    [t0..t5]               = "
                + ", ".join(f"{x:+.1f}" for x in tcl))

    # ── assertions ──
    checks = []

    def chk(label, cond):
        checks.append((label, bool(cond)))

    def tcl_of(name):
        return res[name][1]

    if any(v[1] is None for v in res.values()):
        log("\nFATAL: at least one case produced no output. Aborting verdict.")
        _write(lines, ok=False)
        _shutdown(node)
        return

    # corners = t2..t5, vertical pair = t0,t1
    n_tcl = tcl_of("neutral")
    chk("neutral: all outputs ~0 (|t|<5)", max(abs(x) for x in n_tcl) < 5.0)

    hh = tcl_of("heading_high")
    hl = tcl_of("heading_low")
    # heading_high (err -30) -> yaw_n<0 -> corners uniform NEGATIVE
    chk("heading_high: corners t2..t5 all < 0", all(hh[i] < 0 for i in (2, 3, 4, 5)))
    # heading_low (err +30) -> corners uniform POSITIVE
    chk("heading_low: corners t2..t5 all > 0", all(hl[i] > 0 for i in (2, 3, 4, 5)))
    # heading correction barely touches the vertical pair
    chk("heading: vertical pair ~unaffected (|t0|,|t1|<5)",
        abs(hh[0]) < 5 and abs(hh[1]) < 5 and abs(hl[0]) < 5 and abs(hl[1]) < 5)

    rp = tcl_of("roll_pos")
    rn = tcl_of("roll_neg")
    # roll_pos (roll_err -1) -> roll_n<0 -> t0=roll_n*100<0, t1=-roll_n*100>0
    chk("roll_pos: t0<0 and t1>0 (differential)", rp[0] < 0 and rp[1] > 0)
    chk("roll_neg: t0>0 and t1<0 (opposite)",     rn[0] > 0 and rn[1] < 0)
    chk("roll: corners ~unaffected (|t2..t5|<5)",
        all(abs(rp[i]) < 5 and abs(rn[i]) < 5 for i in (2, 3, 4, 5)))

    ds = tcl_of("depth_shallow")
    dd = tcl_of("depth_deep")
    # SAFETY SIGN: shallow (1.2<1.5 -> descend) MUST drive vertical pair negative
    chk("depth_shallow(descend): t0<0 AND t1<0 (heave_sign=-1)", ds[0] < 0 and ds[1] < 0)
    chk("depth_deep(ascend): t0>0 AND t1>0",                     dd[0] > 0 and dd[1] > 0)
    chk("depth: corners ~unaffected (|t2..t5|<5)",
        all(abs(ds[i]) < 5 and abs(dd[i]) < 5 for i in (2, 3, 4, 5)))

    gy = tcl_of("gyro_yaw")
    # gyro_z>0 -> yaw_n = -gyro_z*3 <0 -> corners uniform negative (strong)
    chk("gyro_yaw: corners t2..t5 all < 0 (D-term damps spin)",
        all(gy[i] < 0 for i in (2, 3, 4, 5)))
    chk("gyro_yaw: response is strong (|t2|>20)", abs(gy[2]) > 20)

    gr = tcl_of("gyro_roll")
    # gyro_x>0 -> roll_n = -gyro_x*1 <0 -> t0<0, t1>0
    chk("gyro_roll: t0<0 and t1>0 (roll D-term)", gr[0] < 0 and gr[1] > 0)

    log("\n===== ASSERTIONS =====")
    all_ok = True
    for label, ok in checks:
        log(f"  [{'PASS' if ok else 'FAIL'}] {label}")
        all_ok = all_ok and ok
    log(f"\nRESULT: {'ALL PASS' if all_ok else 'FAILURES PRESENT'}  "
        f"({sum(1 for _, o in checks if o)}/{len(checks)})")

    _write(lines, ok=all_ok)
    _shutdown(node)


def _write(lines, ok):
    with open(RESULT_FILE, "w") as f:
        f.write("\n".join(lines))
        f.write(f"\n\nVERDICT={'PASS' if ok else 'FAIL'}\n")


def _shutdown(node):
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
