#!/usr/bin/env python3
"""Bench monitor for the depth fusion stack.

Subscribes /depth_raw (MS5837 driver), /depth (fused output) and
/depth_fusion/innovation for N seconds, then prints rate / gap / value
statistics for each so the fused stream can be compared against the raw one.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0


class Monitor(Node):
    def __init__(self):
        super().__init__('fusion_monitor')
        self.raw = []    # (t_mono, value)
        self.fused = []
        self.innov = []
        self.create_subscription(Float32, '/depth_raw',
                                 lambda m: self.raw.append((time.monotonic(), m.data)), 50)
        self.create_subscription(Float32, '/depth',
                                 lambda m: self.fused.append((time.monotonic(), m.data)), 50)
        self.create_subscription(Float32, '/depth_fusion/innovation',
                                 lambda m: self.innov.append((time.monotonic(), m.data)), 50)


def report(name, recs, elapsed):
    print('--- %s ---' % name)
    if not recs:
        print('  NO MESSAGES')
        return
    ts = [r[0] for r in recs]
    vs = [r[1] for r in recs]
    gaps = [ts[i] - ts[i - 1] for i in range(1, len(ts))]
    print('  n=%d  rate=%.2f Hz' % (len(recs), len(recs) / elapsed))
    if gaps:
        gs = sorted(gaps)
        ng = len(gs)
        print('  gap: mean=%.3fs  p50=%.3fs  p95=%.3fs  p99=%.3fs  max=%.3fs'
              % (sum(gaps) / ng, gs[ng // 2], gs[int(0.95 * (ng - 1))],
                 gs[int(0.99 * (ng - 1))], gs[-1]))
        k10 = max(1, int(0.10 * ng + 0.9999))
        k1 = max(1, int(0.01 * ng + 0.9999))
        print('  10%% worst (mean of largest 10%% of gaps): %.3fs   '
              '1%% worst (mean of largest 1%%): %.3fs'
              % (sum(gs[-k10:]) / k10, sum(gs[-k1:]) / k1))
    mean = sum(vs) / len(vs)
    var = sum((v - mean) ** 2 for v in vs) / len(vs)
    print('  value: min=%.4f  max=%.4f  mean=%.4f  stdev=%.4f'
          % (min(vs), max(vs), mean, var ** 0.5))


def main():
    rclpy.init()
    node = Monitor()
    start = time.monotonic()
    print('monitoring for %.0fs...' % DURATION)
    sys.stdout.flush()
    while time.monotonic() - start < DURATION:
        rclpy.spin_once(node, timeout_sec=0.2)
    elapsed = time.monotonic() - start

    print()
    report('/depth_raw (MS5837 driver)', node.raw, elapsed)
    report('/depth (fused)', node.fused, elapsed)
    report('/depth_fusion/innovation (meas - pred at accepted reads)', node.innov, elapsed)

    # Largest fused-value step between consecutive messages: continuity check.
    f = node.fused
    if len(f) > 2:
        steps = sorted((abs(f[i][1] - f[i - 1][1]), f[i][0] - start) for i in range(1, len(f)))
        print()
        print('largest fused steps (|dz| m @ t): %s'
              % ', '.join('%.4f@%.1fs' % s for s in steps[-5:]))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
