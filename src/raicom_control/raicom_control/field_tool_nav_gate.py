"""Pure fail-closed state for the long-lived navigation adapter."""

import math


class NavigationCommandGate:
    """Require manual suppression and a fresh boot/mission/goal context.

    The adapter is long-lived.  A new goal context clears the previous goal's
    stop state; an inactive context clears only the current goal.  No process
    restart is used as a lifecycle transition.
    """

    GOAL_FIELDS = {'boot_id', 'mission_id', 'goal_id', 'active', 'expires_at'}

    def __init__(self, acknowledgement_timeout_sec=0.3, expected_boot_id=None):
        self.acknowledgement_timeout_sec = float(acknowledgement_timeout_sec)
        if not 0.1 <= self.acknowledgement_timeout_sec <= 0.5:
            raise ValueError('acknowledgement timeout must be 0.1 to 0.5 seconds')
        self.expected_boot_id = expected_boot_id or None
        self.acknowledged_at = None
        self.stopped = False
        self.current_goal = None
        self.completed_goals = set()

    def observe_manual_suppressed(self, suppressed, now):
        self.acknowledged_at = float(now) if bool(suppressed) else None

    @staticmethod
    def _valid_identity(message):
        return (isinstance(message, dict)
                and set(message) == NavigationCommandGate.GOAL_FIELDS
                and all(isinstance(message[key], str) and message[key]
                        for key in ('boot_id', 'mission_id', 'goal_id'))
                and type(message['active']) is bool
                and type(message['expires_at']) in (int, float)
                and math.isfinite(float(message['expires_at'])))

    def receive_goal_context(self, message, now):
        """Accept a current context, or clear the current goal explicitly."""

        now = float(now)
        if not math.isfinite(now) or not self._valid_identity(message):
            return False
        if self.expected_boot_id and message['boot_id'] != self.expected_boot_id:
            return False
        key = (message['boot_id'], message['mission_id'], message['goal_id'])
        if not message['active']:
            if self.current_goal is None or key != self.current_goal['key']:
                return False
            self.completed_goals.add(key)
            self.current_goal = None
            self.stopped = False
            return True
        expires_at = float(message['expires_at'])
        if expires_at <= now:
            return False
        if self.current_goal is not None and key == self.current_goal['key']:
            # A refresh may extend the same goal, but never revive it with an
            # older deadline after an out-of-order/replayed message.
            if expires_at < self.current_goal['expires_at']:
                return False
            self.current_goal['expires_at'] = expires_at
            return True
        if key in self.completed_goals:
            return False
        if self.current_goal is not None:
            self.completed_goals.add(self.current_goal['key'])
        self.current_goal = {'key': key, 'expires_at': expires_at}
        self.stopped = False
        return True

    def has_active_goal(self, now):
        if self.current_goal is None:
            return False
        if float(now) > self.current_goal['expires_at']:
            self.completed_goals.add(self.current_goal['key'])
            self.current_goal = None
            self.stopped = False
            return False
        return True

    def request_stop(self):
        changed = not self.stopped
        if self.current_goal is not None:
            self.completed_goals.add(self.current_goal['key'])
            self.current_goal = None
        self.stopped = True
        return changed

    def permits_motion(self, now):
        return (not self.stopped and self.acknowledged_at is not None
                and 0.0 <= float(now) - self.acknowledged_at
                <= self.acknowledgement_timeout_sec + 1e-9
                and self.has_active_goal(now))
