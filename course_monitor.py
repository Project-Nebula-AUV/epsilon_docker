#!/usr/bin/env python3
"""Watch /sub/status until the mission ends.

Prints deduplicated task/state transitions (numbers collapsed). Exit code:
  0  mission completed (ShutdownTask / MISSION_COMPLETE) with NO timeout
     valves fired (no 'T!' marker ever seen)
  1  completed but at least one subtask bailed through a timeout valve
  2  MISSION_FAILED
  3  wall-clock timeout before the mission ended

    python3 course_monitor.py [duration_s]
"""
import re
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Monitor(Node):
    def __init__(self):
        super().__init__('course_monitor')
        self.create_subscription(String, '/sub/status', self.on_status, 10)
        self.t0 = time.time()
        self.last_key = None
        self.done = False
        self.failed = False
        self.timeouts_seen = False
        self.tasks_seen = []

    def on_status(self, msg):
        s = msg.data
        key = re.sub(r'[-+]?\d+(\.\d+)?', '#', s)
        if key != self.last_key:
            print(f"t={time.time() - self.t0:7.2f}s  {s}", flush=True)
            self.last_key = key
            task = s.split('|', 1)[0]
            if not self.tasks_seen or self.tasks_seen[-1] != task:
                self.tasks_seen.append(task)
        if 'T!' in s:
            self.timeouts_seen = True
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
        print("RESULT: MISSION_FAILED")
        code = 2
    elif n.done and n.timeouts_seen:
        print("RESULT: completed WITH timeout valves (not a clean pass)")
        code = 1
    elif n.done:
        print("RESULT: clean completion")
        code = 0
    else:
        print("RESULT: wall-clock timeout")
        code = 3
    n.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
