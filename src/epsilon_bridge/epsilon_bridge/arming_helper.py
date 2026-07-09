import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

USAGE = "usage: ros2 run epsilon_bridge arming_helper {arm|disarm|start|pause|reset|quit}"

# command -> (/sim/control string or None, arm bool or None)
ACTIONS = {
    'arm':    ('start', True),
    'disarm': ('pause', False),
    'start':  ('start', None),
    'pause':  ('pause', None),
    'reset':  ('reset', None),
    'quit':   ('quit', None),
}


class ArmingHelper(Node):
    """Unified safe switch: drive the nav brain (/sim/control) and the thruster_bridge
    arming gate (~/arm) together so they never desync.

    arm    = publish 'start' to /sim/control, THEN arm thruster_bridge.
    disarm = disarm thruster_bridge FIRST, THEN publish 'pause'.
    start/pause/reset/quit = /sim/control passthrough only (no arm change).
    """

    def __init__(self):
        super().__init__('arming_helper')
        self.ctrl = self.create_publisher(String, '/sim/control', 10)
        self.cli = self.create_client(SetBool, '/thruster_bridge/arm')

    def run(self, cmd):
        sim_cmd, arm = ACTIONS[cmd]
        if arm is False:          # disarm actuators before stopping nav
            self._arm(False)
        if sim_cmd is not None:
            self._publish(sim_cmd)
        if arm is True:           # arm actuators after nav is started
            self._arm(True)

    def _publish(self, s):
        msg = String()
        msg.data = s
        # WAIT for submarine_node's /sim/control subscription to be discovered
        # before publishing. arming_helper is a short-lived node; on the Pi's
        # slow DDS discovery the old fixed 0.3 s window published 'start' into
        # the void -> the mission stayed in WAITING, issued zero thruster
        # commands, watchdog held the motors at zero (2026-07-09: "motors not
        # spinning" on the bench). Volatile QoS both ends, so latching is out.
        waited = 0.0
        while self.ctrl.get_subscription_count() < 1 and waited < 12.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            waited += 0.1
        n = self.ctrl.get_subscription_count()
        for _ in range(5):        # RELIABLE + spin to flush to matched subs
            self.ctrl.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        if n < 1:
            self.get_logger().warn(
                "/sim/control '%s' published but NO subscriber discovered "
                "in %.1fs (mission may not start!)" % (s, waited))
        else:
            self.get_logger().info(
                "/sim/control <- '%s' (subs=%d, waited %.1fs)" % (s, n, waited))

    def _arm(self, val):
        # 15 s: fresh-node service discovery on the Pi is routinely >3 s
        # (stale-shm/WiFi era, 2026-07-09 poolside failure at 3.0 s).
        if not self.cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().warn('thruster_bridge /arm unavailable; arm gate NOT changed')
            return
        req = SetBool.Request()
        req.data = val
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        res = fut.result()
        if res is not None:
            self.get_logger().info('arm(%s) -> %s' % (val, res.message))
        else:
            self.get_logger().warn('arm(%s) call failed' % val)


def main(args=None):
    argv = sys.argv[1:] if args is None else list(args)
    cmd = next((a for a in argv if a in ACTIONS), None)
    rclpy.init(args=args)
    node = ArmingHelper()
    try:
        if cmd is None:
            node.get_logger().error(USAGE)
        else:
            node.run(cmd)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
