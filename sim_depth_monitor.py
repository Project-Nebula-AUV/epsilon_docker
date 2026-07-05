#!/usr/bin/env python3
"""Score sim fused depth against ground truth while driving vertical motion.

Runs as the sole /thruster_commands publisher (no submarine_node): descends,
holds, ascends, holds -- so the depth_fusion dead-reckoning is exercised
through real vertical motion, then reports how well /sensors/depth (fused,
from emulated sparse MS5837 + IMU) tracks /sim/true_depth.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Float32MultiArray

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
TAG = sys.argv[2] if len(sys.argv) > 2 else ''


class Mon(Node):
    def __init__(self):
        super().__init__('sim_depth_monitor')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.fused = []      # (t, z)
        self.true = []       # (t, z)
        self.raw = []        # (t, z)
        self.create_subscription(Float32, '/sensors/depth',
                                 lambda m: self.fused.append((time.monotonic(), m.data)), qos)
        self.create_subscription(Float32, '/sim/true_depth',
                                 lambda m: self.true.append((time.monotonic(), m.data)), 10)
        self.create_subscription(Float32, '/depth_raw',
                                 lambda m: self.raw.append((time.monotonic(), m.data)), 10)
        self.cmd = self.create_publisher(Float32MultiArray, '/thruster_commands', qos)

    def drive(self, vertical):
        m = Float32MultiArray()
        m.data = [0.0, 0.0, 0.0, 0.0, float(vertical), float(vertical)]
        self.cmd.publish(m)


def true_at(true, t):
    """Nearest ground-truth depth to time t."""
    best, bd = None, 1e9
    for (tt, z) in true:
        d = abs(tt - t)
        if d < bd:
            bd, best = d, z
    return best


def main():
    rclpy.init()
    node = Mon()
    start = time.monotonic()
    while time.monotonic() - start < DURATION:
        t = time.monotonic() - start
        # Sustained vertical motion so the dead-reckoning (and the sign) is
        # actually exercised: drive down long enough to reach the pool floor,
        # then let buoyancy + reverse thrust bring it back up.
        half = DURATION / 2.0
        node.drive(0.8 if t < half else -0.8)
        rclpy.spin_once(node, timeout_sec=0.02)
    node.drive(0.0)
    elapsed = time.monotonic() - start

    # Score fused vs true over their overlap.
    errs = []
    for (t, zf) in node.fused:
        zt = true_at(node.true, t)
        if zt is not None:
            errs.append(abs(zf - zt))
    raw_gaps = [node.raw[i][0] - node.raw[i - 1][0] for i in range(1, len(node.raw))]

    print('=== sim depth fusion score %s ===' % TAG)
    print('duration %.0fs' % elapsed)
    print('raw /depth_raw: n=%d rate=%.2f Hz%s'
          % (len(node.raw), len(node.raw) / elapsed,
             ('  max_gap=%.2fs' % max(raw_gaps)) if raw_gaps else ''))
    print('fused /sensors/depth: n=%d rate=%.2f Hz' % (len(node.fused), len(node.fused) / elapsed))
    if node.true:
        tz = [z for _, z in node.true]
        print('true depth range: %.3f .. %.3f m' % (min(tz), max(tz)))
    if errs:
        errs_sorted = sorted(errs)
        n = len(errs)
        print('fused-vs-true error: mean=%.3f m  p95=%.3f m  max=%.3f m'
              % (sum(errs) / n, errs_sorted[int(0.95 * (n - 1))], errs_sorted[-1]))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
