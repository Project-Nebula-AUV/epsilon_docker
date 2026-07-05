#!/usr/bin/env python3
"""Watch a headless sim mission run with depth fusion in the loop.

Records /sub/status (task|state) transitions from submarine_node and samples
fused /sensors/depth vs /sim/true_depth, so a full mission dry-run can be
scored over SSH without a GUI. Sends 'start' once after a warm-up so the nav
brain begins the mission.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, String

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0


class Mon(Node):
    def __init__(self):
        super().__init__('sim_mission_monitor')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.fused = []
        self.true = []
        self.status_log = []      # (t, status_string)
        self._last_status = None
        self.create_subscription(Float32, '/sensors/depth',
                                 lambda m: self.fused.append((time.monotonic(), m.data)), qos)
        self.create_subscription(Float32, '/sim/true_depth',
                                 lambda m: self.true.append((time.monotonic(), m.data)), 10)
        self.create_subscription(String, '/sub/status', self._status_cb, 10)
        self.ctrl = self.create_publisher(String, '/sim/control', 10)

    def _status_cb(self, msg):
        if msg.data != self._last_status:
            self._last_status = msg.data
            self.status_log.append((time.monotonic(), msg.data))

    def start(self):
        self.ctrl.publish(String(data='start'))


def nearest(series, t):
    best, bd = None, 1e9
    for (tt, z) in series:
        d = abs(tt - t)
        if d < bd:
            bd, best = d, z
    return best


def main():
    rclpy.init()
    node = Mon()
    start = time.monotonic()
    started = False
    while time.monotonic() - start < DURATION:
        t = time.monotonic() - start
        if not started and t > 4.0:
            node.start()          # kick off the mission once, after warm-up
            started = True
        rclpy.spin_once(node, timeout_sec=0.05)
    elapsed = time.monotonic() - start

    print('=== sim mission dry-run (fusion in loop) ===')
    print('duration %.0fs' % elapsed)
    print('--- mission task|state transitions (/sub/status) ---')
    for (tt, s) in node.status_log:
        print('  t=%6.1fs  %s' % (tt - start, s))
    if node.true:
        tz = [z for _, z in node.true]
        print('true depth range: %.3f .. %.3f m' % (min(tz), max(tz)))
    print('fused /sensors/depth: n=%d rate=%.1f Hz' % (len(node.fused), len(node.fused) / elapsed))
    errs = [abs(zf - nearest(node.true, t)) for (t, zf) in node.fused if node.true]
    if errs:
        errs.sort()
        print('fused-vs-true error: mean=%.3f m  p95=%.3f m  max=%.3f m'
              % (sum(errs) / len(errs), errs[int(0.95 * (len(errs) - 1))], errs[-1]))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
