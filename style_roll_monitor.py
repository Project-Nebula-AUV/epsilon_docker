#!/usr/bin/env python3
"""Monitor /sub/status + /sensors/roll during a style-roll sim run.

Prints every status transition, tracks roll extremes while StyleRoll is
active, and exits early once the mission advances past the first GateTask
(status shows OrbitTurnTask) or on timeout.

    python3 style_roll_monitor.py [duration_s]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, String


class Monitor(Node):
    def __init__(self):
        super().__init__('style_roll_monitor')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(String, '/sub/status', self.on_status, 10)
        self.create_subscription(Float32, '/sensors/roll', self.on_roll, qos)
        self.t0 = time.time()
        self.last_status = None
        self.roll = 0.0
        self.roll_min = 0.0
        self.roll_max = 0.0
        self.in_roll = False
        self.roll_seen = False
        self.level_roll = None
        self.done = False

    def on_roll(self, msg):
        self.roll = float(msg.data)
        if self.in_roll:
            self.roll_min = min(self.roll_min, self.roll)
            self.roll_max = max(self.roll_max, self.roll)

    def on_status(self, msg):
        s = msg.data
        if s != self.last_status:
            print(f"t={time.time()-self.t0:7.2f}s  {s}  (roll {self.roll:+.1f})", flush=True)
            was_in_roll = self.in_roll
            self.in_roll = 'StyleRoll' in s
            if self.in_roll:
                self.roll_seen = True
            if was_in_roll and not self.in_roll:
                self.level_roll = self.roll
            if 'OrbitTurnTask' in s:
                self.done = True
            self.last_status = s


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
    rclpy.init()
    n = Monitor()
    while rclpy.ok() and not n.done and (time.time() - n.t0) < dur:
        rclpy.spin_once(n, timeout_sec=0.2)
    print("---")
    if n.roll_seen:
        print(f"StyleRoll observed: roll swept {n.roll_min:+.1f}..{n.roll_max:+.1f} deg "
              f"(wrapped), leveled at {n.level_roll if n.level_roll is not None else n.roll:+.1f} deg")
    else:
        print("StyleRoll NEVER appeared in /sub/status")
    print(f"{'PASS: mission advanced past GateTask' if n.done else 'TIMEOUT before OrbitTurnTask'}")
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
