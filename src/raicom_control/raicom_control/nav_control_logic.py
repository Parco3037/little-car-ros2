"""Pure fail-closed state for the navigation command bridge."""


class NavigationCommandGate:
    """Require a fresh manual-input suppression acknowledgement.

    A stop request is permanently latched for this process. Re-arming requires
    restarting the bridge, so a later Nav2 command cannot undo operator Space.
    """

    def __init__(self, acknowledgement_timeout_sec=0.3):
        self.acknowledgement_timeout_sec = float(acknowledgement_timeout_sec)
        if not 0.1 <= self.acknowledgement_timeout_sec <= 0.5:
            raise ValueError('acknowledgement timeout must be 0.1 to 0.5 seconds')
        self.acknowledged_at = None
        self.stopped = False

    def observe_manual_suppressed(self, suppressed, now):
        self.acknowledged_at = float(now) if bool(suppressed) else None

    def request_stop(self):
        changed = not self.stopped
        self.stopped = True
        return changed

    def permits_motion(self, now):
        return (not self.stopped and self.acknowledged_at is not None
                and 0.0 <= float(now) - self.acknowledged_at
                <= self.acknowledgement_timeout_sec + 1e-9)
