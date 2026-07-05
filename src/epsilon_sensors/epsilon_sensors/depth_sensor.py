import rclpy
from epsilon_sensors import ms5837
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32
import time
import threading


class DepthInput(Node):
    def __init__(self):
        super().__init__('depth_node')

        # Ceiling on the read-attempt rate right after a SUCCESS (not a
        # floor): the loop always retries as soon as the previous attempt's
        # own timing (I2C waits inside ms5837.read()) allows, and only adds
        # extra sleep if that was somehow faster than 1/rate_hz. Raised from
        # a 10 Hz *unconditional* sleep (2026-07-04 fix): that add-on sleep
        # fired after every accepted read regardless of how long the read
        # itself took, which throttled a cooperating bus to 10 Hz right when
        # it was working -- a real read already takes ~35-40ms, so 20 Hz
        # (50ms) is a ceiling that doesn't bind today but still bounds
        # attempt rate if this bus is ever healthy enough for that to matter.
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        self.period = 1.0 / float(rate_hz)

        # Most read attempts fail at the I2C level on this bus (~12% survive;
        # see depth_sanity_test.py), so retry failures fast instead of
        # sleeping a full period -- this bounds the worst-case gap between
        # good reads, which is what the IMU dead-reckoning has to bridge.
        self.fail_delay = float(self.declare_parameter('fail_delay', 0.02).value)

        # Oversampling: 256..8192. Lower OSR = shorter conversion = more
        # attempts/s; on this marginal bus the valid-read RATE matters far
        # more than per-read ADC noise (bench @256: ~1.5 valid reads/s).
        osr = int(self.declare_parameter('osr', 256).value)
        osr_map = {256: ms5837.OSR_256, 512: ms5837.OSR_512,
                   1024: ms5837.OSR_1024, 2048: ms5837.OSR_2048,
                   4096: ms5837.OSR_4096, 8192: ms5837.OSR_8192}
        if osr not in osr_map:
            self.get_logger().warn(f'invalid osr {osr}, using 256')
            osr = 256
        self.osr = osr_map[osr]

        # DENSITY_FRESHWATER (997) or DENSITY_SALTWATER (1029)
        self.fluid_density = self.declare_parameter(
            'fluid_density', ms5837.DENSITY_FRESHWATER).value

        # Operating-envelope gate applied inside ms5837.read() (see ms5837.py):
        # anything outside is bus corruption, not a real reading, and is
        # rejected as read()==False. Pool defaults; widen for a deeper or
        # high-altitude venue without a code edit. Blocks the recurring
        # corruption signatures (18/50/113/264 m, -0.7 m, impossible temps)
        # at the driver so they never reach the fusion node at all.
        self.press_min_mbar = float(self.declare_parameter('press_min_mbar', 950.0).value)
        self.press_max_mbar = float(self.declare_parameter('press_max_mbar', 1800.0).value)
        self.temp_min_c = float(self.declare_parameter('temp_min_c', 10.0).value)
        self.temp_max_c = float(self.declare_parameter('temp_max_c', 40.0).value)

        # Per-conversion ADC sleep floor (s). Default 0.010 is proven-safe. On
        # this 10 kHz bus the block-read latency and I2C timeouts dominate read
        # time, so lowering it showed no measurable throughput gain in a bench
        # sweep (2026-07-04) -- exposed here only so a venue with a healthier /
        # faster bus can tune it without a code edit. Keep it well above the
        # ~0.6 ms OSR_256 conversion time or reads catch the ADC mid-convert.
        self.conv_sleep_floor = float(self.declare_parameter('conv_sleep_floor_s', 0.010).value)

        # Kernel i2c-dev per-transaction timeout (ms), via I2C_TIMEOUT ioctl
        # (see ms5837.set_i2c_timeout). Measured 2026-07-04: this bus's
        # DesignWare I2C driver blocks ~1.024s on every wedged/timed-out
        # transaction by default, while a genuine read completes in ~35-40ms.
        # 100ms leaves large margin over a real read without waiting a full
        # second to find out a transaction is dead. An on-bus interleaved A/B
        # test the same day showed this does NOT raise sustained valid-
        # reads/sec (the fault is a fixed physical duty-cycle -- retrying
        # faster just confirms "still wedged" faster, see
        # robosub-i2c-bus-health notes) -- it's a reconnect-responsiveness /
        # wasted-blocking-time fix, not a throughput fix. reconnect_after_s
        # below is time-based specifically so it stays correct regardless of
        # what this is tuned to.
        self.i2c_timeout_ms = float(self.declare_parameter('i2c_timeout_ms', 100.0).value)

        # Reconnect if I2C-level exceptions (not envelope-rejected reads) run
        # unbroken for this many seconds -- wall-clock, not attempt count, so
        # it stays meaningful regardless of i2c_timeout_ms (a raw consecutive-
        # attempt counter would fire ~10x more often just from shortening the
        # per-attempt timeout, for no reason related to the sensor actually
        # being gone). _reset_only makes an actual reconnect cheap (a single
        # RESET, no PROM re-derivation), so this doesn't need to be
        # conservative to avoid burning time on a wasted reconnect.
        self.reconnect_after_s = float(self.declare_parameter('reconnect_after_s', 50.0).value)

        self.publisher = self.create_publisher(Float32, '/depth', 10)

        self.sensor = None
        self._lock = threading.Lock()
        self._running = True
        self._verified_C = None  # calibration, once double-CRC-verified from the live PROM

        # Attempt initial connection
        self.connect()

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        self.get_logger().info('MS5837 depth node started')

    def _robust_init(self, sensor):
        """Bring the sensor up on a bus where most transactions fail.

        The library's init() is all-or-nothing (reset + 7 PROM reads must
        every one succeed in a single pass), which essentially never happens
        on this bus -- the node used to sit in a connect-retry loop forever
        while the bench test, which retries each step, read the same sensor
        fine. Retry the reset and every PROM word independently, and gate on
        the PROM CRC so a silently-corrupted calibration word can't poison
        every later depth calculation.
        """
        addr = ms5837.MS5837._MS5837_ADDR
        bus = sensor._bus

        for _ in range(8):
            try:
                bus.write_byte(addr, sensor._MS5837_RESET)
                break
            except OSError:
                time.sleep(0.05)
        else:
            return False
        time.sleep(0.01)

        # The CRC is only 4 bits, so on a bus this corrupt a bad word set has
        # a real (1/16) chance of passing it -- and a wrong calibration
        # silently poisons every depth afterwards (observed: a whole run
        # publishing ~0.65 m off). Require two CRC-passing passes that also
        # agree word-for-word; independent corruption twice identically is
        # effectively impossible.
        verified = None
        for _ in range(8):  # whole-PROM passes, CRC-gated + double-read
            C = []
            for i in range(7):
                word = None
                for _r in range(40):
                    try:
                        w = bus.read_word_data(addr, sensor._MS5837_PROM_READ + 2 * i)
                        # SMBus words are little-endian; PROM is big-endian.
                        word = ((w & 0xFF) << 8) | (w >> 8)
                        break
                    except OSError:
                        time.sleep(0.02)
                if word is None:
                    return False
                C.append(word)
            crc = (C[0] & 0xF000) >> 12
            if crc != sensor._crc4(list(C)):
                continue
            if verified == C:
                sensor._C = C
                return True
            verified = C
        return False

    def _reset_only(self, sensor):
        """Re-arm the sensor with just a RESET, no PROM traffic.

        Used on every reconnect after the first: the calibration is a
        property of this physical chip, not of the connection, so it
        doesn't need re-deriving once we've verified it live. Re-deriving
        it anyway on every reconnect is what turned ordinary bus noise into
        a 34s stall -- the double-CRC-verified 7-word PROM pass needs a
        cleaner bus than a single depth read does, right when a reconnect
        firing means the bus just proved it wasn't that clean.
        """
        addr = ms5837.MS5837._MS5837_ADDR
        bus = sensor._bus
        for _ in range(8):
            try:
                bus.write_byte(addr, sensor._MS5837_RESET)
                return True
            except OSError:
                time.sleep(0.05)
        return False

    def connect(self):
        try:
            sensor = ms5837.MS5837_30BA(bus=2)
            if sensor._bus is None:
                raise RuntimeError('I2C bus 2 not available')

            # Apply the operating-envelope gate to this instance. read() uses
            # self._PRESSURE_MIN_MBAR etc.; setting them as instance attrs
            # overrides the ms5837 class pool defaults from the ROS params,
            # so a deeper/high-altitude venue widens the gate with no code edit.
            sensor._PRESSURE_MIN_MBAR = self.press_min_mbar
            sensor._PRESSURE_MAX_MBAR = self.press_max_mbar
            sensor._TEMP_MIN_C = self.temp_min_c
            sensor._TEMP_MAX_C = self.temp_max_c
            sensor._conv_floor = self.conv_sleep_floor
            sensor.set_i2c_timeout(self.i2c_timeout_ms)

            if self._verified_C is not None:
                if not self._reset_only(sensor):
                    raise RuntimeError('reset failed (bus errors)')
                sensor._C = self._verified_C
            else:
                if not self._robust_init(sensor):
                    raise RuntimeError('PROM init failed (bus errors/CRC)')
                self._verified_C = sensor._C

            sensor.setFluidDensity(self.fluid_density)
            time.sleep(0.1)

            with self._lock:
                self.sensor = sensor
            self.get_logger().info('MS5837 connected')
            return True
        except (OSError, ValueError, RuntimeError) as e:
            with self._lock:
                self.sensor = None
            self.get_logger().warn(f'MS5837 connect failed: {e}; will retry')
            return False

    def _read_loop(self):
        # A single failed transaction does NOT mean the sensor is gone -- on
        # this bus most transactions fail and the sensor keeps working (its
        # calibration PROM is already cached), so re-init on every OSError
        # just burns seconds of reset+PROM traffic. Only reconnect after a
        # long unbroken RUN OF WALL-CLOCK TIME spent erroring (a genuinely
        # wedged/unplugged sensor), not a raw attempt count -- keeps this
        # threshold meaningful regardless of i2c_timeout_ms.
        first_error_t = None

        while self._running:
            with self._lock:
                sensor = self.sensor

            if sensor is None:
                self.connect()
                time.sleep(1.0)
                continue

            t_attempt = time.monotonic()
            try:
                ok = sensor.read(self.osr)
                first_error_t = None
                if not ok:
                    self.get_logger().warn('MS5837 read() returned False',
                                           throttle_duration_sec=5.0)
                    time.sleep(self.fail_delay)
                    continue

                msg = Float32()
                msg.data = sensor.depth()
                self.publisher.publish(msg)

            except (OSError, ValueError, RuntimeError) as e:
                now = time.monotonic()
                if first_error_t is None:
                    first_error_t = now
                if now - first_error_t >= self.reconnect_after_s:
                    self.get_logger().warn(
                        f'MS5837: I2C errors for {now - first_error_t:.0f}s ({e}); reconnecting')
                    first_error_t = None
                    with self._lock:
                        self.sensor = None
                else:
                    self.get_logger().warn(f'MS5837 read failed: {e}',
                                           throttle_duration_sec=5.0)
                time.sleep(self.fail_delay)
                continue

            # Cap (not floor): only add sleep if this attempt was somehow
            # faster than the configured ceiling. A real read already takes
            # ~35-40ms, so at the 20 Hz (50ms) default this rarely fires --
            # unlike the old unconditional sleep(period), it never adds dead
            # time on top of an attempt that already took as long as period.
            remaining = self.period - (time.monotonic() - t_attempt)
            if remaining > 0.0:
                time.sleep(remaining)

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = DepthInput()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
