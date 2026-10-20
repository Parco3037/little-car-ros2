"""Pure safety logic shared by ROS nodes and unit tests."""

from dataclasses import dataclass
import math
from typing import Iterable

@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0

    @property
    def is_zero(self) -> bool:
        return self.linear_x == 0.0 and self.linear_y == 0.0 and self.angular_z == 0.0


ZERO_COMMAND = VelocityCommand()


class MotionEnableLease:
    """Small, explicit motion-enable lease for the velocity guard.

    This is intentionally only a freshness/identity record.  It does not
    inspect subsystem health, localization, DDS, hardware or task state.
    ``expires_at`` is a monotonic timestamp from the same host.
    """

    FIELDS = {'boot_id', 'mission_id', 'enabled', 'expires_at'}

    def __init__(self) -> None:
        self.boot_id = None
        self.mission_id = None
        self.enabled = False
        self.expires_at = None

    def receive(self, message, monotonic_now: float) -> bool:
        """Accept exactly one current motion-enable schema, fail closed."""

        self.enabled = False
        self.expires_at = None
        try:
            if (not isinstance(message, dict)
                    or set(message) != self.FIELDS
                    or not isinstance(message['boot_id'], str)
                    or not message['boot_id']
                    or not isinstance(message['mission_id'], str)
                    or not message['mission_id']
                    or type(message['enabled']) is not bool
                    or type(message['expires_at']) not in (int, float)
                    or not math.isfinite(float(message['expires_at']))
                    or not math.isfinite(float(monotonic_now))):
                return False
            self.boot_id = message['boot_id']
            self.mission_id = message['mission_id']
            self.enabled = message['enabled']
            self.expires_at = float(message['expires_at'])
            return True
        except (TypeError, ValueError, OverflowError):
            return False

    def valid_at(self, monotonic_now: float) -> bool:
        try:
            now = float(monotonic_now)
            return (self.enabled and self.expires_at is not None
                    and math.isfinite(now)
                    and now <= self.expires_at)
        except (TypeError, ValueError, OverflowError):
            return False


class AccelerationLimiter:
    """Bound command changes; safety stops bypass this limiter."""

    def __init__(self, max_linear_accel: float, max_angular_accel: float) -> None:
        if not math.isfinite(max_linear_accel) or max_linear_accel <= 0.0:
            raise ValueError('max_linear_accel must be positive and finite')
        if not math.isfinite(max_angular_accel) or max_angular_accel <= 0.0:
            raise ValueError('max_angular_accel must be positive and finite')
        self.max_linear_accel = float(max_linear_accel)
        self.max_angular_accel = float(max_angular_accel)
        self.previous = ZERO_COMMAND
        self.previous_at = None

    def reset(self, now: float | None = None) -> None:
        self.previous = ZERO_COMMAND
        self.previous_at = now

    def apply(self, command: VelocityCommand, now: float) -> VelocityCommand:
        try:
            now = float(now)
            if not math.isfinite(now):
                self.reset()
                return ZERO_COMMAND
            if self.previous_at is None:
                self.previous_at = now
                return ZERO_COMMAND
            dt = now - self.previous_at
            if not math.isfinite(dt) or dt <= 0.0:
                self.reset(now)
                return ZERO_COMMAND
            # A long gap means the previous output is stale; begin the new
            # command from zero instead of allowing a jump after a pause.
            if dt > 1.0:
                self.previous = ZERO_COMMAND
            linear_step = self.max_linear_accel * dt
            angular_step = self.max_angular_accel * dt
            result = VelocityCommand(
                linear_x=max(self.previous.linear_x - linear_step,
                             min(self.previous.linear_x + linear_step,
                                 command.linear_x)),
                linear_y=max(self.previous.linear_y - linear_step,
                             min(self.previous.linear_y + linear_step,
                                 command.linear_y)),
                angular_z=max(self.previous.angular_z - angular_step,
                              min(self.previous.angular_z + angular_step,
                                  command.angular_z)),
            )
            self.previous = result
            self.previous_at = now
            return result
        except (TypeError, ValueError, OverflowError):
            self.reset()
            return ZERO_COMMAND


def _finite_clamp(value: float, limit: float) -> float:
    if not math.isfinite(value):
        return 0.0
    return max(-limit, min(limit, value))


def sanitize_command(values: Iterable[float], max_linear: float, max_angular: float) -> VelocityCommand:
    """Reject non-finite values and clamp all motion axes to fixed safety limits."""
    linear_x, linear_y, angular_z = values
    return VelocityCommand(
        linear_x=_finite_clamp(float(linear_x), max_linear),
        linear_y=_finite_clamp(float(linear_y), max_linear),
        angular_z=_finite_clamp(float(angular_z), max_angular),
    )


class WatchdogState:
    """Monotonic-time command watchdog. It starts and fails in the stopped state."""

    def __init__(self, timeout_sec: float) -> None:
        if timeout_sec <= 0.0:
            raise ValueError('timeout_sec must be positive')
        self.timeout_sec = timeout_sec
        self._last_command = ZERO_COMMAND
        self._last_update: float | None = None

    def update(self, command: VelocityCommand, now: float) -> None:
        self._last_command = command
        self._last_update = now

    def command_at(self, now: float) -> VelocityCommand:
        if self._last_update is None:
            return ZERO_COMMAND
        age = now - self._last_update
        if age < 0.0 or age > self.timeout_sec:
            return ZERO_COMMAND
        return self._last_command

    def is_stale(self, now: float) -> bool:
        return self.command_at(now).is_zero and (
            self._last_update is None or now - self._last_update > self.timeout_sec
        )


class LocalizationLease:
    """Fail-closed, session-bound source freshness; receipt alone is not health.

    This is a local odometry interlock, not AMCL/navigation acceptance. A lost
    producer must start a new explicitly configured session, never replay one.
    """
    LIMITS = {'laser': .45, 'imu': .3, 'ekf': .3}

    def __init__(self, session: str):
        if not session:
            raise ValueError('An explicit localization session is required')
        self.session = session
        self.sequence = -1
        self.received = None
        self.stamps = None
        self.last_ros = None

    def receive(self, message, monotonic_now, ros_now):
        self.received = None
        try:
            if (message['session'] != self.session or message['valid'] is not True
                    or message['state'] != 'ACTIVE' or type(message['sequence']) is not int
                    or message['sequence'] <= self.sequence):
                return False
            stamps = message['source_stamps']
            if set(stamps) != set(self.LIMITS):
                return False
            if not math.isfinite(monotonic_now) or not math.isfinite(ros_now):
                return False
            if self.last_ros is not None and ros_now < self.last_ros:
                return False
            for name, limit in self.LIMITS.items():
                value = stamps[name]
                if type(value) not in (int, float) or not math.isfinite(value):
                    return False
                if not 0 <= ros_now-value <= limit:
                    return False
                if self.stamps is not None and value < self.stamps[name]:
                    return False
            self.sequence = message['sequence']
            self.stamps = dict(stamps)
            self.received = monotonic_now
            self.last_ros = ros_now
            return True
        except (TypeError, ValueError, KeyError, OverflowError):
            return False

    def valid_at(self, monotonic_now, ros_now):
        if self.received is None or self.stamps is None:
            return False
        valid = (math.isfinite(monotonic_now) and math.isfinite(ros_now)
                 and 0 <= monotonic_now-self.received <= .3
                 and ros_now >= self.last_ros
                 and all(0 <= ros_now-self.stamps[k] <= v for k, v in self.LIMITS.items()))
        if not valid:
            self.received = None
        else:
            self.last_ros = ros_now
        return valid
