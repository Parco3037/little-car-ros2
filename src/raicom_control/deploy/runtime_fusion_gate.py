"""Bounded production source pause without accepting stale odometry.

The experiment gate remains unchanged. A short EKF/IMU delivery delay, scan-
processing delay, or bounded raw-scan receipt gap closes the motion lease. The
SAME tracker and EKF resume only after three new EKF and IMU samples plus two
new accepted laser measurements. No origin reset, changed stamp, or stale
output is allowed. A pause over one second, a clock fault, tracker reset, or
independently confirmed loss stays latched.
"""
import math
from scan_fusion_gate import FusionGate


def source_sample_is_fresh(age, limit):
    """Classify finite, non-future source age without restamping it."""
    try:
        age = float(age)
        limit = float(limit)
    except (ValueError, TypeError, OverflowError) as error:
        raise ValueError('Invalid source age') from error
    if not math.isfinite(age) or not math.isfinite(limit) or age < 0 or limit <= 0:
        raise ValueError('Invalid source age')
    return age <= limit


class RuntimeFusionGate(FusionGate):
    # Keep the scan tracker's accepted-measurement gap and the runtime's
    # fail-closed recovery budget identical.  The old tracker default was
    # 0.6 s, so it latched LOST before this documented one-second recovery
    # window could finish after a short run of rejected stationary scans.
    MAX_RECOVERY_SEC = 1.0

    def __init__(self):
        super().__init__()
        self.pause = None
        self.counts = dict(laser=0, imu=0, ekf=0)
        self.pauses = 0
        self.resumptions = 0
        self.input_scan_stamp = None

    def observe_scan_input(self, stamp):
        """Receipt proves transport only, NEVER an accepted laser observation."""
        try:
            stamp = float(stamp)
            if not math.isfinite(stamp): raise ValueError('Nonfinite scan stamp')
        except (ValueError, TypeError, OverflowError):
            self.fail('Invalid received scan stamp')
            return False
        if self.input_scan_stamp is not None and stamp < self.input_scan_stamp:
            self.fail('Received scan clock moved backwards')
            return False
        self.input_scan_stamp = stamp
        return self.failure is None

    def observe(self, source, stamp):
        accepted = super().observe(source, stamp)
        if accepted: self.counts[source] += 1
        return accepted

    def evaluate(self, now, tracker_status):
        if self.failure is not None or not self.established:
            return super().evaluate(now, tracker_status)
        try:
            ages = {k: float(now)-v for k,v in self.stamps.items()}
            input_age = None if self.input_scan_stamp is None else float(now)-self.input_scan_stamp
            if input_age is not None and input_age < 0:
                self.fail('Future received scan stamp')
                return super().evaluate(now, tracker_status)
            laser_fresh = 0 <= ages['laser'] <= self.LIMITS['laser']
            # A fresh raw scan permits computation to catch up for the same
            # one-second budget as the tracker, but NEVER opens output.
            processing = (self.LIMITS['laser'] < ages['laser'] <= self.MAX_RECOVERY_SEC
                          and input_age is not None and 0 <= input_age <= .3)
            bounded_scan_transport = (
                0 <= ages['laser'] <= self.MAX_RECOVERY_SEC
                and input_age is not None
                and 0 <= input_age <= self.MAX_RECOVERY_SEC
            )
            bounded_imu = 0 <= ages['imu'] <= self.MAX_RECOVERY_SEC
            bounded_ekf = 0 <= ages['ekf'] <= self.MAX_RECOVERY_SEC
            all_normally_fresh = (laser_fresh
                                  and ages['imu'] <= self.LIMITS['imu']
                                  and ages['ekf'] <= self.LIMITS['ekf'])
            # A raw scan can itself be delayed by scheduler pressure.  Enter
            # the same zero-output recovery pause while its last receipt is
            # still inside the one-second tracker budget.  Previously this was
            # allowed only after another source had already opened a pause,
            # so a 382.6 ms receipt gap could latch LOST at the 450 ms laser
            # edge even though no physical output was enabled.
            eligible = (tracker_status == 'TRACKING'
                        and (laser_fresh or processing or bounded_scan_transport)
                        and bounded_imu and bounded_ekf
                        and not all_normally_fresh)
        except (ValueError, TypeError, OverflowError):
            return super().evaluate(now, tracker_status)
        if not eligible and self.pause is None:
            return super().evaluate(now, tracker_status)
        if self.pause is None:
            delayed = []
            if processing:
                delayed.append('laser_processing')
            elif not laser_fresh and bounded_scan_transport:
                delayed.append('scan_transport')
            if ages['imu'] > self.LIMITS['imu']:
                delayed.append('imu')
            if ages['ekf'] > self.LIMITS['ekf']:
                delayed.append('ekf')
            self.pause = dict(start=now, laser=self.counts['laser'],
                              imu=self.counts['imu'], ekf=self.counts['ekf'],
                              sources=delayed)
            self.pauses += 1
        # Suppress only the permanent stale latch while output remains CLOSED.
        # All accepted-data ages and all clock/reset checks remain intact.
        self.established = False
        result = super().evaluate(now, tracker_status)
        self.established = True
        # A pause can begin while the last accepted laser is still fresh (for
        # example because IMU/EKF delivery crossed 300 ms), then the same CPU
        # scheduling event can delay raw scan receipt beyond 300 ms.  Once the
        # output is already closed, keep that pause bounded by the existing
        # one-second budget instead of latching at the next 450 ms laser edge.
        # Re-opening still requires two newly accepted laser measurements plus
        # three new IMU and EKF samples below.
        if (not laser_fresh and not processing and bounded_scan_transport
                and 'scan_transport' not in self.pause['sources']):
            self.pause['sources'].append('scan_transport')
        if (tracker_status != 'TRACKING' or ages['ekf'] > self.MAX_RECOVERY_SEC
                or now-self.pause['start'] > self.MAX_RECOVERY_SEC
                or not (laser_fresh or processing or bounded_scan_transport)
                or not 0 <= ages['imu'] <= self.MAX_RECOVERY_SEC):
            self.fail('Processing pause exceeded bounded recovery or independent sensor lost')
        if self.failure:
            return super().evaluate(now, tracker_status)
        if (result['valid'] and self.counts['ekf']-self.pause['ekf'] >= 3
                and self.counts['imu']-self.pause['imu'] >= 3
                and self.counts['laser']-self.pause['laser'] >= 2):
            self.pause = None
            self.resumptions += 1
            return result
        sources = ','.join(self.pause['sources']) or 'recovery_confirmation'
        result.update(valid=False, state='PAUSED',
                      reason='Bounded source pause ('+sources+'); zero lease until fresh source confirmations',
                      pause_sources=list(self.pause['sources']))
        return result
