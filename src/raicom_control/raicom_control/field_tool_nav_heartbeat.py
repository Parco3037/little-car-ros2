"""Thread-safe navigation authority; no ROS or GUI dependency.

The GUI owns the action state machine. An independent worker may only refresh an
explicit, still-live authorization; it never creates or revives a goal.
"""
from dataclasses import dataclass, replace
import math
import threading
import time


@dataclass(frozen=True)
class _Authorization:
    goal_id: str
    mission_id: str
    accepted: bool
    deadline: float
    stop_generation: int
    started_at: float
    result_future: object = None

    @property
    def key(self):
        return self.mission_id, self.goal_id


class NavigationHeartbeat:
    """Serialize refreshes and withdrawal, retaining retired goal identities."""

    def __init__(self, node):
        self.node = node
        self._lock = threading.RLock()
        self._authorization = None
        self._ui_at = None
        self._retired = set()
        self._faults = {}

    def _generation(self):
        value = getattr(self.node, 'external_stop_generation', 0)
        return value if type(value) is int else 0

    def touch_ui(self, now=None):
        """Record UI activity for diagnostics, not autonomous-goal authority."""
        now = time.monotonic() if now is None else float(now)
        if not math.isfinite(now):
            raise ValueError('GUI heartbeat time must be finite')
        with self._lock:
            if self._ui_at is None or now >= self._ui_at:
                self._ui_at = now

    def authorize(self, goal_id, mission_id, accepted, deadline, result_future=None):
        with self._lock:
            now = time.monotonic()
            key = mission_id, goal_id
            if key in self._retired:
                return False
            if (not isinstance(goal_id, str) or not goal_id
                    or not isinstance(mission_id, str) or not mission_id
                    or type(accepted) is not bool or type(deadline) not in (int, float)
                    or not math.isfinite(deadline)):
                raise ValueError('invalid navigation heartbeat authorization')
            previous = self._authorization
            if previous is not None and previous.key != key:
                reason = 'navigation heartbeat identity changed before withdrawal'
                self._fault_locked('heartbeat_unavailable', reason)
                self._retired.add(key)
                self._faults[goal_id] = ('heartbeat_unavailable', reason)
                return False
            if previous is None:
                self._authorization = _Authorization(
                    goal_id, mission_id, accepted, float(deadline),
                    self._generation(), now, result_future)
            else:
                if previous.accepted and not accepted:
                    return self._fault_locked('heartbeat_unavailable',
                                              'navigation heartbeat accepted goal reverted to pending')
                # Repeated polling cannot extend an acceptance/action timeout.
                next_deadline = (float(deadline) if accepted and not previous.accepted
                                 else min(previous.deadline, float(deadline)))
                self._authorization = replace(
                    previous, accepted=previous.accepted or accepted,
                    deadline=next_deadline,
                    result_future=(previous.result_future if previous.result_future is not None
                                   else result_future))
            return self.tick()

    def _withdraw_locked(self):
        authorization = self._authorization
        self._authorization = None
        actions = []
        if authorization is not None:
            self._retired.add(authorization.key)
            actions.append(lambda: self.node.publish_goal_context(
                authorization.goal_id, active=False, mission_id=authorization.mission_id))
        actions.extend((lambda: self.node.publish_lease(False),
                        lambda: self.node.publish_manual_suppressed(False)))
        failures = []
        for action in actions:
            try:
                action()
            except Exception as exc:
                failures.append(str(exc) or type(exc).__name__)
        if failures and authorization is not None:
            kind, reason = self._faults.get(authorization.goal_id, (
                'heartbeat_unavailable', 'navigation heartbeat withdrawal failed'))
            self._faults[authorization.goal_id] = (kind, reason + '; ' + '; '.join(failures))
        elif failures:
            raise RuntimeError('navigation heartbeat withdrawal failed: ' + '; '.join(failures))

    def _fault_locked(self, kind, reason):
        authorization = self._authorization
        if authorization is not None:
            self._faults[authorization.goal_id] = (kind, reason)
        self._withdraw_locked()
        return False

    def revoke(self):
        """Explicit withdrawal also disables an idle/manual lease once."""
        with self._lock:
            self._withdraw_locked()

    def on_external_stop(self):
        with self._lock:
            self.node.external_stop_generation = self._generation() + 1
            self.node.external_stop_requested = True
            self._fault_locked('operator_stop', 'navigation heartbeat revoked by external stop')

    def fault_for(self, goal_id):
        with self._lock:
            return self._faults.get(goal_id)

    def worker_failed(self, reason):
        with self._lock:
            self._fault_locked('heartbeat_unavailable', reason)

    def _valid_locked(self, authorization, now):
        if self._authorization is not authorization:
            return False
        if (self._generation() != authorization.stop_generation
                or getattr(self.node, 'external_stop_requested', False) is True):
            return self._fault_locked('operator_stop', 'navigation heartbeat external stop changed')
        if authorization.result_future is not None and authorization.result_future.done() is True:
            self._withdraw_locked()
            return False  # Let the controller consume success/cancel/failure itself.
        # No extra UI-liveness or producer-gap cutoff. The independent worker
        # renews this finite action; the original bridge/guard retain their
        # authoritative command/lease expiry and emergency-stop protection.
        if now > authorization.deadline:
            return self._fault_locked('action_timeout', 'navigation heartbeat action deadline expired')
        return True

    def tick(self, now=None):
        with self._lock:
            # Check the finite action deadline using the time after any wait
            # for another refresh, not an obsolete pre-lock timestamp.
            now = time.monotonic() if now is None else float(now)
            authorization = self._authorization
            if authorization is None:
                return False  # An idle timer must not interfere with manual control.
            if not self._valid_locked(authorization, now):
                return False
            # Nav2 owns TF/costmap validity while computing velocity. A delayed
            # workbench listener must not withdraw a healthy Nav2 action. This
            # worker publishes authorization only, never cached velocity.
            try:
                self.node.publish_goal_context(
                    authorization.goal_id, active=True, mission_id=authorization.mission_id)
                if self._authorization is not authorization:
                    return False
                self.node.publish_manual_suppressed(True)
                if self._authorization is not authorization:
                    return False
                self.node.publish_lease(authorization.accepted)
                if self._authorization is not authorization:
                    return False
            except Exception as exc:
                return self._fault_locked('heartbeat_unavailable',
                                          'navigation heartbeat publication failed: ' + str(exc))
            return True


class NavigationHeartbeatWorker:
    """Keep renewal independent of both Tk and the general ROS executor.

    The getter follows workspace controller replacement. Shutdown joins before
    withdrawing authority and before the owning node destroys publishers.
    """

    def __init__(self, get_heartbeat, report_error):
        self._get_heartbeat = get_heartbeat
        self._report_error = report_error
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name='navigation-heartbeat', daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        while not self._stop.wait(.05):
            try:
                self._get_heartbeat().tick()
            except Exception as exc:
                self._stop.set()
                self._report_error('Navigation heartbeat worker failed: ' + str(exc))
                try:
                    self._get_heartbeat().worker_failed('Navigation heartbeat worker failed: ' + str(exc))
                except Exception as revoke_error:
                    self._report_error('Navigation heartbeat withdrawal failed: ' + str(revoke_error))
                return

    def close(self):
        self._stop.set()
        self.thread.join(timeout=2.)
        if self.thread.is_alive():
            raise RuntimeError('Navigation heartbeat worker did not stop; publishers must remain alive')
        self._get_heartbeat().revoke()
