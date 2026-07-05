#!/usr/bin/env python3
"""Monitor /sub/status through the FULL course sim run.

Prints deduplicated task/state transitions (digits stripped so per-tick
counters don't spam), and exits PASS once ShutdownTask or MISSION_COMPLETE
appears, FAIL immediately if a task reports MISSION_FAILED, or FAIL on
timeout.

    python3 full_course_monitor.py [duration_s]
"""
import re
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Monitor(Node):
    def __init__(self):
        super().__init__('full_course_monitor')
        self.create_subscription(String, '/sub/status', self.on_status, 10)
        self.t0 = time.time()
        self.last_key = None
        self.done = False
        self.failed = False
        self.tasks_seen = []

    def on_status(self, msg):
        s = msg.data
        key = re.sub(r'[-+]?\d+(\.\d+)?', '#', s)   # collapse changing numbers
        if key != self.last_key:
            print(f"t={time.time()-self.t0:7.2f}s  {s}", flush=True)
            self.last_key = key
            task = s.split('|', 1)[0]
            if not self.tasks_seen or self.tasks_seen[-1] != task:
                self.tasks_seen.append(task)
        if 'MISSION_FAILED' in s:
            self.failed = True
            self.done = True
        elif 'ShutdownTask' in s or 'MISSION_COMPLETE' in s:
            self.done = True


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
    rclpy.init()
    n = Monitor()
    while rclpy.ok() and not n.done and (time.time() - n.t0) < dur:
        rclpy.spin_once(n, timeout_sec=0.2)
    print("---")
    print("task sequence: " + " -> ".join(n.tasks_seen))
    if n.failed:
        print("FAIL: a mission task reported MISSION_FAILED")
    elif n.done:
        print("PASS: full course reached ShutdownTask")
    else:
        print("FAIL: timeout before ShutdownTask")
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
