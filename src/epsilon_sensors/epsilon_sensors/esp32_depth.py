import json
import math
import threading
import time

import serial
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32, Bool


class Esp32Depth(Node):
    """MS5837 depth via the Xiao ESP32-C3 bridge (USB CDC serial).

    The Pi 5's own I2C buses proved unreliable with this sensor (June-July
    2026: 0 valid reads across ~16k attempts); the sensor now hangs off a
    Xiao ESP32-C3 which streams one JSON line per reading at ~10 Hz:

        {"pressure":992.40,"temperature":26.70,"depth":-0.21}

    (pressure mbar, temperature C, depth m against the FIRMWARE's sea-level
    baseline -- not used; we re-reference to captured surface pressure).

    QUIRK (measured 2026-07-06): after the port is opened the stream does not
    flow until the chip is hard-reset via the modem lines (Espressif
    USB-JTAG-serial: RTS pulses EN). connect() therefore always performs the
    reset sequence, waits out the ROM boot banner, and expects JSON within a
    few seconds; a reader that goes silent for reconnect_after_s is torn down
    and reconnected the same way. Unplug/replug is survived the same way.

    Filtering (the bus is clean but the firmware occasionally emits an
    obviously-bad line): (1) JSON parse, (2) pressure/temperature range
    gates, (3) median-of-3, (4) rate gate vs the last accepted depth with a
    3-in-a-row consensus re-anchor so a genuine step change cannot be locked
    out (miniature of the old depth_fusion consensus logic, no IMU needed).

    Topics (names chosen so downstream needs no code changes):
      /depth_raw                Float32, every parsed line, surface-referenced,
                                UNFILTERED (outliers visible -- the sysid
                                logger already records this topic)
      /depth                    Float32, filtered, 20 Hz zero-order hold of
                                the 10 Hz stream (nav consumes via
                                sensor_bridge passthrough, unchanged)
      /esp32_depth/velocity_z   Float32, low-passed derivative (m/s, down+)
      /esp32_depth/stale        Bool, True when no accepted reading recently
                                (launch remaps sensor_bridge's old
                                /depth_fusion/* subscriptions to these)
    """

    def __init__(self):
        super().__init__('esp32_depth')
        self.port = str(self.declare_parameter('port', '/dev/ttyACM0').value)
        self.baud = int(self.declare_parameter('baud', 115200).value)
        self.publish_hz = float(self.declare_parameter('publish_hz', 20.0).value)
        # Plausibility gates. Pressure window covers surface ambient at any
        # sane venue altitude (measured bench ambient 972-993 mbar) down to
        # ~8 m of freshwater over it.
        self.press_min = float(self.declare_parameter('press_min_mbar', 900.0).value)
        self.press_max = float(self.declare_parameter('press_max_mbar', 1800.0).value)
        self.temp_min = float(self.declare_parameter('temp_min_c', 0.0).value)
        self.temp_max = float(self.declare_parameter('temp_max_c', 45.0).value)
        # Rate gate + consensus re-anchor.
        self.max_speed = float(self.declare_parameter('max_speed', 1.5).value)
        self.rate_margin = float(self.declare_parameter('rate_margin_m', 0.10).value)
        self.consensus_n = int(self.declare_parameter('consensus_count', 3).value)
        self.consensus_tol = float(self.declare_parameter('consensus_tol_m', 0.15).value)
        # Surface reference: captured at boot (sub starts at the surface /
        # in air -- same convention as sensor_bridge's level capture), or
        # fixed via surface_pressure_mbar when capture is off.
        self.capture_surface = bool(self.declare_parameter('capture_surface_on_start', True).value)
        self.capture_secs = float(self.declare_parameter('surface_capture_secs', 3.0).value)
        self.surface_pressure = float(self.declare_parameter('surface_pressure_mbar', 1013.25).value)
        self.fluid_density = float(self.declare_parameter('fluid_density', 997.0).value)
        self.stale_after = float(self.declare_parameter('stale_after_s', 1.5).value)
        self.reconnect_after = float(self.declare_parameter('reconnect_after_s', 5.0).value)

        self.m_per_mbar = 100.0 / (self.fluid_density * 9.80665)

        self.pub_raw = self.create_publisher(Float32, '/depth_raw', 10)
        self.pub_depth = self.create_publisher(Float32, '/depth', 10)
        self.pub_vz = self.create_publisher(Float32, '/esp32_depth/velocity_z', 10)
        self.pub_stale = self.create_publisher(Bool, '/esp32_depth/stale', 10)

        self._lock = threading.Lock()
        self._depth = None          # last ACCEPTED filtered depth (m, down+)
        self._vz = 0.0
        self._last_accept_mono = None
        self._last_accept_z = None
        self._median3 = []
        self._pending = []          # consensus run of out-of-gate readings
        self._cap_samples = []
        self._cap_started = None
        self._surface_p = None if self.capture_surface else self.surface_pressure

        self._n_parsed = 0
        self._n_malformed = 0
        self._n_gate_rej = 0
        self._n_rate_rej = 0
        self._n_reanchor = 0
        self._n_reconnects = 0
        self._last_line_mono = None

        self._stop = False
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

        self.create_timer(1.0 / self.publish_hz, self.on_publish)
        self.create_timer(10.0, self.on_status)
        self.get_logger().info(
            'esp32_depth up: %s, gates p[%g,%g] t[%g,%g], surface %s, %g Hz publish'
            % (self.port, self.press_min, self.press_max, self.temp_min, self.temp_max,
               'capture@boot %.1fs' % self.capture_secs if self.capture_surface
               else 'fixed %.1f mbar' % self.surface_pressure, self.publish_hz))

    # ---------- serial reader thread ----------

    def _connect(self):
        """Open the port and hard-reset the chip (stream does not flow on a
        plain open -- see class docstring). Returns the Serial or None."""
        try:
            s = serial.Serial(self.port, self.baud, timeout=1.0)
            s.dtr = False
            s.rts = True     # EN low: chip in reset
            time.sleep(0.1)
            s.rts = False    # release: boots into the app
            time.sleep(0.3)
            s.reset_input_buffer()
            self.get_logger().info('connected + reset %s' % self.port)
            return s
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn('open %s failed: %s' % (self.port, e),
                                   throttle_duration_sec=10.0)
            return None

    def _reader(self):
        ser = None
        while not self._stop:
            if ser is None:
                ser = self._connect()
                if ser is None:
                    time.sleep(1.0)
                    continue
                self._last_line_mono = time.monotonic()
            try:
                raw = ser.readline()
            except (serial.SerialException, OSError) as e:
                self.get_logger().warn('serial read failed (%s); reconnecting' % e)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                self._n_reconnects += 1
                time.sleep(1.0)
                continue

            now = time.monotonic()
            line = raw.decode(errors='replace').strip()
            if line:
                self._last_line_mono = now
                self._handle_line(line, now)
            elif (now - (self._last_line_mono or now)) > self.reconnect_after:
                # Port open but silent (firmware wedged / replug enumerated
                # fresh): tear down and redo the reset dance.
                self.get_logger().warn('stream silent >%.0fs; resetting connection'
                                       % self.reconnect_after)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                self._n_reconnects += 1
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    # ---------- per-line pipeline ----------

    def _handle_line(self, line, now):
        try:
            d = json.loads(line)
        except ValueError:
            self._n_malformed += 1   # boot banner lines land here
            return
        if isinstance(d, dict) and 'error' in d:
            # Firmware tries the MS5837 once per boot and prints this on
            # failure, then goes silent -- the silence trips the reconnect
            # (= chip reset = fresh init attempt), so this retries forever.
            self.get_logger().warn('firmware: %s (sensor init failed this '
                                   'boot; will keep resetting)' % d['error'],
                                   throttle_duration_sec=10.0)
            self._n_malformed += 1
            return
        try:
            p = float(d['pressure'])
            t = float(d['temperature'])
        except (ValueError, KeyError, TypeError):
            self._n_malformed += 1
            return
        self._n_parsed += 1

        if not (self.press_min <= p <= self.press_max
                and self.temp_min <= t <= self.temp_max):
            self._n_gate_rej += 1
            return

        # Surface capture phase: accumulate, publish nothing yet.
        if self._surface_p is None:
            if self._cap_started is None:
                self._cap_started = now
            self._cap_samples.append(p)
            if now - self._cap_started >= self.capture_secs:
                self._cap_samples.sort()
                self._surface_p = self._cap_samples[len(self._cap_samples) // 2]
                self.get_logger().info(
                    'surface pressure captured: %.2f mbar (%d samples)'
                    % (self._surface_p, len(self._cap_samples)))
            return

        z_raw = (p - self._surface_p) * self.m_per_mbar
        self.pub_raw.publish(Float32(data=float(z_raw)))

        # median-of-3 kills isolated spikes at the cost of ~1 sample lag
        self._median3.append(z_raw)
        if len(self._median3) > 3:
            self._median3.pop(0)
        z = sorted(self._median3)[len(self._median3) // 2]

        with self._lock:
            if self._depth is not None and self._last_accept_mono is not None:
                dt = max(1e-3, now - self._last_accept_mono)
                gate = self.max_speed * dt + self.rate_margin
                if abs(z - self._depth) > gate:
                    # out-of-gate: extend/reset the consensus run
                    if (self._pending
                            and abs(z - self._pending[-1][1]) <= self.consensus_tol
                            and now - self._pending[-1][0] <= 1.5):
                        self._pending.append((now, z))
                    else:
                        self._pending = [(now, z)]
                    if len(self._pending) < self.consensus_n:
                        self._n_rate_rej += 1
                        return
                    # consensus: the reading stream is right, our state is wrong
                    self._n_reanchor += 1
                    t0, z0 = self._pending[0]
                    self._vz = ((z - z0) / max(1e-3, now - t0)
                                if now - t0 > 0.05 else 0.0)
                    self.get_logger().warn(
                        're-anchor to %.3f m on %d-read consensus (was %.3f)'
                        % (z, self.consensus_n, self._depth))
                else:
                    v_inst = (z - self._depth) / dt
                    v_inst = max(-self.max_speed, min(self.max_speed, v_inst))
                    self._vz += 0.5 * (v_inst - self._vz)   # light low-pass
            self._pending = []
            self._depth = z
            self._last_accept_mono = now
            self._last_accept_z = z

    # ---------- timers ----------

    def on_publish(self):
        with self._lock:
            depth = self._depth
            vz = self._vz
            last = self._last_accept_mono
        stale = last is None or (time.monotonic() - last) > self.stale_after
        if depth is not None:
            self.pub_depth.publish(Float32(data=float(depth)))
            self.pub_vz.publish(Float32(data=float(0.0 if stale else vz)))
        self.pub_stale.publish(Bool(data=bool(stale)))

    def on_status(self):
        with self._lock:
            depth = self._depth
        self.get_logger().info(
            'depth %s | parsed %d, malformed %d, gate-rej %d, rate-rej %d, '
            're-anchors %d, reconnects %d'
            % ('%.3f m' % depth if depth is not None else '(no fix)',
               self._n_parsed, self._n_malformed, self._n_gate_rej,
               self._n_rate_rej, self._n_reanchor, self._n_reconnects))


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Depth()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._stop = True
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
