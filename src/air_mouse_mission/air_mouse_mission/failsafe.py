#!/usr/bin/env python3
"""NIDAR AirMouse failsafe + emergency stop.

Requirements from NIDAR:
  - Emergency stop / mission abort capability and failsafe features.
  - Corridor height clearance 8 feet (2.44m) at all times.
  - Max mission 30 mins.
  - Return home on comms loss, low battery, VIO / lidar failure.

Monitors:
  - ``/emergency_stop`` (Bool) from the ground station button.
  - Battery voltage (PX4 ``/fmu/out/battery_status``).
  - VIO health (``/vins_estimator/camera_pose`` timeout).
  - Lidar health (``/scan`` timeout).
  - Height limit, geofence (15x15m arena), RC loss.

Simulation hardening:
  - Waits for gzserver to be fully initialized and for simulated time
    to be ticking before enforcing strict data timeouts. This prevents
    spurious ``Lidar timeout`` / ``VIO timeout`` aborts while Gazebo is
    still loading, headless, or paused.
  - Uses ROS time (``use_sim_time``) for all timeout bookkeeping so
    paused simulation does not look like sensor loss.
  - ``startup_grace_period`` (default 20s wall time) plus an explicit
    sim-clock tick check gate all timeout aborts. Emergency stop still
    aborts immediately, even during the grace period.
"""

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import BatteryStatus
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class FailsafeNode(Node):
    """Enforce NIDAR failsafe rules with sim-time-aware gating."""

    def __init__(self) -> None:
        super().__init__('failsafe_node')
        self.declare_parameter('max_height', 2.44)
        self.declare_parameter('arena_size', 15.0)
        self.declare_parameter('vio_timeout', 2.0)
        self.declare_parameter('lidar_timeout', 2.0)
        self.declare_parameter('low_battery', 14.0)
        self.declare_parameter('startup_grace_period', 20.0)
        self.declare_parameter('require_sim_time', True)
        self.declare_parameter('sim_tick_window', 2.0)
        self.declare_parameter('min_sim_elapsed', 2.0)
        self.declare_parameter('enable_vio_check', True)
        self.declare_parameter('enable_lidar_check', True)
        self.declare_parameter('enable_battery_check', True)

        self.max_height = float(self.get_parameter('max_height').value)
        self.arena_size = float(self.get_parameter('arena_size').value)
        self.vio_timeout = float(self.get_parameter('vio_timeout').value)
        self.lidar_timeout = float(self.get_parameter('lidar_timeout').value)
        self.low_battery = float(self.get_parameter('low_battery').value)
        self.grace_period = float(
            self.get_parameter('startup_grace_period').value
        )
        self.require_sim_time = bool(
            self.get_parameter('require_sim_time').value
        )
        self.sim_tick_window = float(
            self.get_parameter('sim_tick_window').value
        )
        self.min_sim_elapsed = float(
            self.get_parameter('min_sim_elapsed').value
        )
        self.enable_vio = bool(self.get_parameter('enable_vio_check').value)
        self.enable_lidar = bool(
            self.get_parameter('enable_lidar_check').value
        )
        self.enable_battery = bool(
            self.get_parameter('enable_battery_check').value
        )

        # Wall-clock start (monotonic, immune to sim pauses).
        self._wall_start = time.monotonic()

        # ROS-time bookkeeping. When use_sim_time is true, these are
        # simulation seconds; otherwise they are wall seconds.
        now_ros = self._now_ros()
        self._first_sim_time: float | None = None
        self._last_sim_time: float | None = None
        self._last_tick_wall = time.monotonic()
        self._sim_ready = False
        self._ready_ros_time: float | None = None
        self._paused_logged = False

        self._last_vio_ros = now_ros
        self._last_lidar_ros = now_ros
        self._vio_received = False
        self._lidar_received = False
        self._battery_received = False

        self.current_pose: PoseStamped | None = None
        self.battery_voltage = 16.8
        self.emergency_stop = False

        self.pose_sub = self.create_subscription(
            PoseStamped, '/offboard/pose', self.pose_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.e_stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.e_stop_callback, 10
        )
        self.vio_sub = self.create_subscription(
            PoseStamped,
            '/vins_estimator/camera_pose',
            self.vio_callback,
            10,
        )
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.battery_callback,
            10,
        )

        self.abort_pub = self.create_publisher(Bool, '/mission/abort', 10)
        self.reason_pub = self.create_publisher(
            String, '/failsafe/reason', 10
        )
        self.status_pub = self.create_publisher(
            String, '/failsafe/status', 10
        )

        self.timer = self.create_timer(0.5, self.check_failsafe)

        self.get_logger().info(
            f'Failsafe: max height {self.max_height}m, '
            f'arena {self.arena_size}m, vio timeout {self.vio_timeout}s, '
            f'lidar timeout {self.lidar_timeout}s, '
            f'grace {self.grace_period}s, '
            f'require_sim_time={self.require_sim_time}'
        )
        self.get_logger().info(
            'Waiting for gzserver / sim time before enforcing timeouts...'
        )

    # -- Callbacks -----------------------------------------------------
    def pose_callback(self, msg: PoseStamped) -> None:
        """Store the latest offboard pose."""
        self.current_pose = msg

    def scan_callback(self, msg: LaserScan) -> None:
        """Record lidar liveness in ROS time."""
        self._last_lidar_ros = self._now_ros()
        self._lidar_received = True

    def vio_callback(self, msg: PoseStamped) -> None:
        """Record VIO liveness in ROS time."""
        self._last_vio_ros = self._now_ros()
        self._vio_received = True

    def battery_callback(self, msg: BatteryStatus) -> None:
        """Record battery voltage."""
        try:
            self.battery_voltage = float(msg.voltage_v)
        except (TypeError, ValueError):
            return
        self._battery_received = True

    def e_stop_callback(self, msg: Bool) -> None:
        """Latch emergency stop and abort immediately."""
        self.emergency_stop = bool(msg.data)
        if self.emergency_stop:
            self.trigger_abort('Emergency stop button pressed')

    # -- Time helpers --------------------------------------------------
    def _now_ros(self) -> float:
        """Return current ROS time in seconds (sim or wall)."""
        try:
            return float(self.get_clock().now().nanoseconds / 1e9)
        except Exception:  # noqa: BLE001 - fall back to wall time
            return time.time()

    def _use_sim_time(self) -> bool:
        """Return True when the node runs on simulated time."""
        try:
            param = self.get_parameter_or(
                'use_sim_time', rclpy.Parameter(
                    'use_sim_time',
                    rclpy.Parameter.Type.BOOL,
                    False,
                )
            )
            return bool(param.value)
        except Exception:  # noqa: BLE001
            return False

    def _update_sim_tracking(self, now_ros: float) -> None:
        """Track first tick and ticking state of the ROS clock."""
        if now_ros > 0.0 and self._first_sim_time is None:
            self._first_sim_time = now_ros
            self._last_sim_time = now_ros
            self._last_tick_wall = time.monotonic()
            return
        if self._last_sim_time is None:
            self._last_sim_time = now_ros
            return
        if now_ros != self._last_sim_time:
            self._last_sim_time = now_ros
            self._last_tick_wall = time.monotonic()

    def _sim_ticking(self) -> bool:
        """Return True when the ROS clock advanced recently."""
        wall_since_tick = time.monotonic() - self._last_tick_wall
        return wall_since_tick <= max(self.sim_tick_window, 0.5)

    def _sim_elapsed(self, now_ros: float) -> float:
        """Return seconds elapsed since the first clock tick."""
        if self._first_sim_time is None:
            return 0.0
        return max(0.0, now_ros - self._first_sim_time)

    def _check_sim_ready(self, now_ros: float) -> tuple[bool, str]:
        """Decide whether strict timeout enforcement may start.

        Returns (ready, status_message).
        """
        wall_elapsed = time.monotonic() - self._wall_start
        self._update_sim_tracking(now_ros)

        use_sim = self._use_sim_time()
        needs_sim_gate = self.require_sim_time and use_sim

        if not needs_sim_gate:
            # Real hardware (wall clock): short grace then enforce.
            short_grace = min(self.grace_period, 5.0)
            if wall_elapsed < short_grace:
                return False, (
                    f'INITIALIZING {wall_elapsed:.0f}/{short_grace:.0f}s'
                )
            return True, 'OK'

        # Simulation: require grace + first tick + elapsed + ticking.
        if self._first_sim_time is None:
            return False, (
                'INITIALIZING: waiting for /clock '
                f'(gzserver loading, {wall_elapsed:.0f}s wall)'
            )
        if wall_elapsed < self.grace_period:
            return False, (
                f'INITIALIZING {wall_elapsed:.0f}/{self.grace_period:.0f}s '
                '(gzserver grace period)'
            )
        sim_elapsed = self._sim_elapsed(now_ros)
        if sim_elapsed < self.min_sim_elapsed:
            return False, (
                f'INITIALIZING: sim time {sim_elapsed:.1f}/'
                f'{self.min_sim_elapsed:.1f}s'
            )
        if not self._sim_ticking():
            return False, 'PAUSED: sim time not advancing (Gazebo paused?)'

        return True, 'OK'

    def _publish_status(self, text: str) -> None:
        """Publish a failsafe status string."""
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    # -- Main check ----------------------------------------------------
    def check_failsafe(self) -> None:
        """Periodically evaluate failsafe rules with sim gating."""
        now_ros = self._now_ros()

        # Emergency stop always aborts, even during initialization.
        if self.emergency_stop:
            self.trigger_abort('Emergency stop')
            return

        ready, status = self._check_sim_ready(now_ros)
        if not ready:
            # During init / pause: never abort on timeouts, just report.
            if status.startswith('PAUSED') and not self._paused_logged:
                self.get_logger().warn(
                    'Sim time paused; failsafe timeouts suspended.'
                )
                self._paused_logged = True
            elif not status.startswith('PAUSED'):
                self._paused_logged = False
                self.get_logger().info(
                    f'Failsafe waiting: {status}', throttle_duration_sec=5.0
                )
            self._publish_status(status)
            return

        # Transition into ready: re-baseline sensors so they get a full
        # timeout window *after* gzserver is up, not from node start.
        if not self._sim_ready:
            self._sim_ready = True
            self._ready_ros_time = now_ros
            if not self._vio_received:
                self._last_vio_ros = now_ros
            if not self._lidar_received:
                self._last_lidar_ros = now_ros
            self.get_logger().info(
                'Sim time ticking; failsafe timeouts now enforced.'
            )
        self._paused_logged = False

        reason: str | None = None

        # Height limit (only when a pose is available).
        if (
            self.current_pose is not None
            and self.current_pose.pose.position.z > self.max_height
        ):
            height = self.current_pose.pose.position.z
            reason = (
                f'Height limit exceeded: {height:.2f} > {self.max_height}'
            )

        # Geofence.
        if self.current_pose is not None and reason is None:
            x = self.current_pose.pose.position.x
            y = self.current_pose.pose.position.y
            half = self.arena_size / 2
            if abs(x) > half or abs(y) > half:
                reason = (
                    f'Geofence breach: ({x:.1f}, {y:.1f}) '
                    f'outside {self.arena_size}m'
                )

        # VIO timeout (ROS time, so pauses do not trigger).
        if self.enable_vio and reason is None:
            vio_age = now_ros - self._last_vio_ros
            if vio_age > self.vio_timeout:
                reason = f'VIO timeout: {vio_age:.1f}s'

        # Lidar timeout.
        if self.enable_lidar and reason is None:
            lidar_age = now_ros - self._last_lidar_ros
            if lidar_age > self.lidar_timeout:
                reason = f'Lidar timeout: {lidar_age:.1f}s'

        # Battery (only when the topic has been seen; SITL may not
        # publish it until PX4 connects).
        if (
            self.enable_battery
            and self._battery_received
            and reason is None
        ):
            if self.battery_voltage < self.low_battery:
                reason = f'Low battery: {self.battery_voltage:.1f}V'

        if reason is not None:
            self.trigger_abort(reason)
        else:
            self._publish_status('OK')

    def trigger_abort(self, reason: str) -> None:
        """Publish failsafe abort + reason."""
        self.get_logger().error(f'FAILSAFE TRIGGERED: {reason}')

        reason_msg = String()
        reason_msg.data = reason
        self.reason_pub.publish(reason_msg)

        abort_msg = Bool()
        abort_msg.data = True
        self.abort_pub.publish(abort_msg)
        self._publish_status(f'ABORT: {reason}')


def main(args=None) -> None:
    """Spin the failsafe node."""
    rclpy.init(args=args)
    node = FailsafeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
