"""Source-time validity gate for a PRIVATE scan/gyro EKF experiment.

A newly stamped EKF prediction does not prove that laser observations are fresh.
This class never publishes, restamps, extrapolates or resets the tracker origin.
Once an established stream fails, create a new explicit experiment to recover.
"""
import math


class FusionGate:
    LIMITS = {'laser': .45, 'imu': .3, 'ekf': .3}

    def __init__(self):
        self.stamps = {name: None for name in self.LIMITS}
        self.last_now = None
        self.established = False
        self.failure = None

    def fail(self, reason):
        if self.failure is None:
            self.failure = reason

    def observe(self, source, stamp):
        if source not in self.stamps:
            raise ValueError('Unknown source '+source)
        try:
            stamp = float(stamp)
            if not math.isfinite(stamp):
                raise ValueError('Nonfinite stamp')
        except (TypeError, ValueError, OverflowError):
            self.fail('Invalid source stamp: '+source)
            return False
        old = self.stamps[source]
        if old is not None and stamp < old:
            self.fail('Source clock moved backwards: '+source)
            return False
        if old == stamp or self.failure is not None:
            return False
        self.stamps[source] = stamp
        return True

    def evaluate(self, now, tracker_status):
        try:
            now = float(now)
            if not math.isfinite(now):
                raise ValueError('Nonfinite clock')
        except (TypeError, ValueError, OverflowError):
            self.fail('Invalid evaluation clock')
            now = None
        if now is not None:
            if self.last_now is not None and now < self.last_now:
                self.fail('Evaluation clock moved backwards')
            self.last_now = now
        if tracker_status == 'LOST':
            self.fail('Tracker LOST; explicit new experiment required')
        elif tracker_status not in ('TRACKING', 'UNINITIALIZED'):
            self.fail('Unknown tracker state')
        elif self.established and tracker_status != 'TRACKING':
            self.fail('Tracker origin reset during established fusion')
        ages = {key: None if stamp is None or now is None else now-stamp
                for key, stamp in self.stamps.items()}
        reason = None
        for source, limit in self.LIMITS.items():
            age = ages[source]
            if age is not None and age < 0:
                self.fail('Future source stamp: '+source)
            if age is None or age > limit:
                reason = reason or ('Missing source: ' if age is None else 'Stale source: ')+source
        if self.established and reason:
            self.fail(reason)
        valid = self.failure is None and reason is None and tracker_status == 'TRACKING'
        if valid:
            self.established = True
        return dict(valid=valid, state='LOST' if self.failure else 'VALID' if valid else 'WAITING',
                    reason=self.failure or reason or ('Fresh independent sources' if valid else 'Waiting for tracker'),
                    source_stamps=dict(self.stamps), source_ages=ages)
