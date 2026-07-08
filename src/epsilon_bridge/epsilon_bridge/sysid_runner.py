import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray, String, Empty
from std_srvs.srv import SetBool
from ament_index_python.packages import get_package_share_directory


class SysidRunner(Node):
    """Execute a scripted open-loop thruster sequence for system identification.

    Publishes /thrust_control (Float64MultiArray[6], -100..100) DIRECTLY --
    thruster_bridge must NOT be running in the sysid stack (two publishers on
    /thrust_control would fight). Because it bypasses the bridge, this node
    carries its own arm gate with the same contract: ~/arm (SetBool), default
    DISARMED, zeros published whenever disarmed. Arming starts the sequence
    at t=0; disarming mid-run aborts it (zeros immediately, sequence will not
    resume). omni_control's own watchdog (0.25 s) backstops runner death.

    Sequence YAML:
        name: s3_heave_staircase
        mode: thrust            # 'thrust' (raw [t0..t5]) or 'wrench'
        max_abs: 50.0           # optional clip on |output|, default 100
        steps:
          - {dur: 5.0, thrust: [-10, -10, 0, 0, 0, 0]}
          - {dur: 5.0, thrust: [-20, -20, 0, 0, 0, 0], ramp: true}
          - {dur: 6.0, wrench: [0.2, 0, 0, 0, 0]}      # mode: wrench only
    'ramp: true' interpolates linearly from the previous output to this
    step's value across its duration (deadband ramps); default is a hold.
    Wrench steps are [surge, sway, heave, yaw, roll] in -1..1, mapped through
    allocation.yaml exactly like thruster_bridge (note: allocation-frame heave
    +1 drives motors 0/1 POSITIVE = ascend on this vehicle).

    A hard time bound (sum of step durations, +2 s grace) forces zeros even if
    stepping logic misbehaves. The final commanded value is always zeros: a
    zeros step is appended if the script does not end at zero (positive
    buoyancy = surfacing failsafe). Step transitions are announced on
    /sysid/marker for the logger; run state on /sysid/status (idle | armed |
    running:<step> | done | aborted) at 2 Hz for the wrapper script.
    """

    def __init__(self):
        super().__init__('sysid_runner')
        seq_path = str(self.declare_parameter('sequence_file', '').value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 50.0).value)
        # 2026-07-07: in-node arming countdown. The '~/go' trigger is fired
        # ONCE on land (WiFi up) by sysid_run.sh; the node then counts down
        # countdown_s internally and self-arms with NO further ROS/CLI/network
        # activity -- so submerging (which drops the sub's WiFi) mid-countdown
        # cannot break arming. countdown_s<=0 => '~/go' arms immediately (DRY).
        self.countdown_s = float(self.declare_parameter('countdown_s', 0.0).value)
        self._countdown_left = None   # int seconds remaining, or None when idle
        self._countdown_timer = None
        alloc_path = str(self.declare_parameter('allocation_file', '').value)
        if not seq_path:
            raise RuntimeError('sequence_file param is required')

        self.seq = self._load_sequence(seq_path)
        self.A = self._load_allocation(alloc_path)
        self.steps = self._compile_steps(self.seq)
        self.total_dur = sum(s['dur'] for s in self.steps)
        self.hard_limit = self.total_dur + 2.0

        self.armed = False
        self.t0 = None
        self.finished = None   # None | 'done' | 'aborted'
        self._last_out = [0.0] * 6
        self._cur_step = -1

        self.pub = self.create_publisher(Float64MultiArray, '/thrust_control', 10)
        self.pub_marker = self.create_publisher(String, '/sysid/marker', 10)
        self.pub_status = self.create_publisher(String, '/sysid/status', 10)
        self.create_service(SetBool, '~/arm', self.on_arm)
        # Topic-triggered start (2026-07-07): 'ros2 service call' was hitting
        # transient rcl/CLI discovery errors mid-water-session; topic pub/echo
        # stayed reliable all session, so sysid_run.sh now fires this topic to
        # start a run instead. '~/arm' is UNCHANGED and still works (manual
        # start/abort from another shell, e.g. mid-run abort).
        self.create_subscription(Empty, '~/go', self.on_go, 10)
        self.create_timer(1.0 / self.rate_hz, self.on_tick)
        self.create_timer(0.5, self.on_status)
        self.get_logger().info(
            'sysid_runner up DISARMED: %s (%d steps, %.1f s, mode=%s, max_abs=%g)'
            % (self.seq.get('name', seq_path), len(self.steps), self.total_dur,
               self.seq.get('mode', 'thrust'), self.max_abs))

    def _load_sequence(self, path):
        with open(path) as f:
            seq = yaml.safe_load(f)
        if not isinstance(seq, dict) or 'steps' not in seq or not seq['steps']:
            raise RuntimeError('sequence file %s has no steps' % path)
        self.max_abs = min(100.0, abs(float(seq.get('max_abs', 100.0))))
        return seq

    def _load_allocation(self, path):
        if not path:
            path = os.path.join(get_package_share_directory('epsilon_bridge'),
                                'config', 'allocation.yaml')
        with open(path) as f:
            cols = yaml.safe_load(f)['allocation']
        order = ['surge', 'sway', 'heave', 'yaw', 'roll']
        return [[float(cols[axis][t]) for axis in order] for t in range(6)]

    def _compile_steps(self, seq):
        """Normalize every step to {dur, target[6], ramp, label} with clipping."""
        mode = seq.get('mode', 'thrust')
        out = []
        for i, s in enumerate(seq['steps']):
            dur = float(s['dur'])
            if dur <= 0.0 or dur > 120.0:
                raise RuntimeError('step %d: dur %.1f outside (0, 120] s' % (i, dur))
            if 'thrust' in s:
                vals = [float(v) for v in s['thrust']]
                if len(vals) != 6:
                    raise RuntimeError('step %d: thrust needs 6 values' % i)
            elif 'wrench' in s:
                if mode != 'wrench':
                    raise RuntimeError('step %d: wrench step in thrust-mode file' % i)
                w = [float(v) for v in s['wrench']]
                if len(w) != 5:
                    raise RuntimeError('step %d: wrench needs 5 values' % i)
                vals = [sum(self.A[t][a] * w[a] for a in range(5)) * 100.0
                        for t in range(6)]
            else:
                raise RuntimeError('step %d: needs thrust or wrench' % i)
            vals = [max(-self.max_abs, min(self.max_abs, v)) for v in vals]
            out.append({'dur': dur, 'target': vals,
                        'ramp': bool(s.get('ramp', False)),
                        'label': str(s.get('label', 'step%d' % i))})
        if any(abs(v) > 1e-9 for v in out[-1]['target']):
            out.append({'dur': 1.0, 'target': [0.0] * 6, 'ramp': False,
                        'label': 'auto-zero-tail'})
        return out

    def _start_run(self):
        if self.finished is None and not self.armed:
            self.armed = True
            self.t0 = self.get_clock().now()
            self._cur_step = -1
            self._mark('RUN-START %s' % self.seq.get('name', ''))
            return True
        return False

    def _begin_countdown(self):
        """Start the in-node arming countdown (idempotent).  When it reaches
        zero the node self-arms -- no external command is needed, so loss of
        WiFi/SSH after this point is harmless."""
        if self.finished is not None or self.armed:
            return False
        if self._countdown_left is not None:
            return False   # already counting
        self._countdown_left = max(1, int(round(self.countdown_s)))
        self._mark('COUNTDOWN-START %ds (autonomous; no further commands needed)'
                   % self._countdown_left)
        self.get_logger().warn(
            '>>> AUTONOMOUS ARM COUNTDOWN: %d s. Place sub + STAND CLEAR. '
            'WiFi/SSH loss from here is safe. <<<' % self._countdown_left)
        self._countdown_timer = self.create_timer(1.0, self._on_countdown_tick)
        return True

    def _on_countdown_tick(self):
        if self.finished is not None:      # aborted during the countdown
            self._cancel_countdown()
            return
        self._countdown_left -= 1
        if self._countdown_left > 0:
            if self._countdown_left <= 10 or self._countdown_left % 10 == 0:
                self.get_logger().info('arming in %d s...' % self._countdown_left)
            self._mark('COUNTDOWN %ds' % self._countdown_left)
            return
        self._cancel_countdown()
        self.get_logger().warn('>>> COUNTDOWN COMPLETE -- ARMING NOW <<<')
        self._start_run()

    def _cancel_countdown(self):
        if self._countdown_timer is not None:
            self._countdown_timer.cancel()
            self.destroy_timer(self._countdown_timer)
            self._countdown_timer = None
        self._countdown_left = None

    def on_go(self, _msg):
        if self.countdown_s > 0.0:
            ok = self._begin_countdown()
            self.get_logger().info(
                'go-topic -> %s' % ('countdown started' if ok else
                                    'ignored (armed/finished/counting)'))
        else:
            started = self._start_run()
            self.get_logger().info(
                'go-topic -> %s' % ('started' if started else
                                    'REJECTED (already armed/finished)'))

    def on_arm(self, req, resp):
        # success must reflect whether the request was actually applied --
        # the wrapper script only checks this field to decide whether arming
        # took, so a rubber-stamped True here hides a no-op arm for 5 minutes.
        if req.data:
            resp.success = self._start_run()
        else:
            counting = self._countdown_left is not None
            if counting:
                self._cancel_countdown()
            if self.armed or counting:
                self.armed = False
                if self.finished is None:
                    self.finished = 'aborted'
                    self._mark('RUN-ABORTED' + (' (countdown cancelled)' if counting else ''))
            resp.success = True
        resp.message = ('armed' if self.armed else
                        'disarmed (%s)' % (self.finished or 'idle'))
        self.get_logger().info('arm(%s) -> %s (success=%s)'
                               % (req.data, resp.message, resp.success))
        return resp

    def _mark(self, text):
        self.pub_marker.publish(String(data=text))
        self.get_logger().info('[marker] %s' % text)

    def _output_at(self, t):
        """Commanded [6] at sequence-time t, handling holds and ramps."""
        acc = 0.0
        prev = [0.0] * 6
        for i, s in enumerate(self.steps):
            if t < acc + s['dur']:
                if i != self._cur_step:
                    self._cur_step = i
                    self._mark('STEP %d %s target=%s' % (i, s['label'],
                               ['%.0f' % v for v in s['target']]))
                if s['ramp']:
                    f = (t - acc) / s['dur']
                    return [p + (v - p) * f for p, v in zip(prev, s['target'])]
                return list(s['target'])
            acc += s['dur']
            prev = s['target']
        return None  # past the end

    def on_tick(self):
        out = [0.0] * 6
        if self.armed and self.finished is None:
            t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
            if t > self.hard_limit:
                self.finished = 'done'
                self._mark('RUN-HARD-LIMIT %.1fs' % t)
            else:
                vals = self._output_at(t)
                if vals is None:
                    self.finished = 'done'
                    self._mark('RUN-DONE %.1fs' % t)
                else:
                    out = vals
        self._last_out = out
        self.pub.publish(Float64MultiArray(data=out))

    def on_status(self):
        if self.finished:
            s = self.finished
        elif self.armed:
            s = 'running:%d/%d' % (max(0, self._cur_step), len(self.steps))
        else:
            s = 'idle'
        self.pub_status.publish(String(data=s))


def main(args=None):
    rclpy.init(args=args)
    node = SysidRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.pub.publish(Float64MultiArray(data=[0.0] * 6))
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
